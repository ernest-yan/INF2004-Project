/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 *
 *	IS33 (Buddy 1, subsystem 1) changes -- agreed with team + instructor
 *	as TEAM.md section 3 requires. Search for "IS33" to find each one:
 *	  1. last will ("offline") + short keepalive, so the broker announces
 *	     when the car disappears
 *	  2. on connect: publish retained "online" and subscribe to the
 *	     command topic (instead of the demo heartbeat topic)
 *	  3. a 20 ms "pump" timer replaces the demo heartbeat: it publishes
 *	     the frames that subsystem 1's telemetry task queued
 *	  4. commands keep their raw bytes and go to tlm_bridge_handle_cmd(),
 *	     whose reply is published as an ACK
 *	All telemetry logic lives in app_program/subsystem1/ (tlm_bridge.c);
 *	this file only does the lwIP calls.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>
/* IS33: plain-C doorway into subsystem 1 (no tk/tkernel.h inside) */
#include "../../../../app_program/subsystem1/tlm_bridge.h"

#define MQTT_UTK_RETRY_MS       3000

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Anything longer than the buffer is
 * truncated and flagged rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

/* IS33: buffers for the bridge's frames */
static u8_t		will_msg[TLM_WILL_MAX];
static u8_t		tx_frame[TLM_FRAME_MAX];
static u8_t		ack_frame[TLM_CMD_MAX];

static void pump_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

/* IS33: is this exactly the command topic? */
static int is_cmd_topic(const char *topic)
{
	const char	*cmd = TLM_TOPIC_CMD;
	u32_t		i;

	for(i = 0; cmd[i] != '\0'; i++) {
		if(topic[i] != cmd[i]) return 0;
	}
	return topic[i] == '\0';
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	/* IS33: keep the RAW bytes -- commands are binary. */
	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if(is_cmd_topic(rx_topic)) {
			/* IS33: subsystem 1 checks it, acts on it, builds the ACK */
			u16_t ack_len = tlm_bridge_handle_cmd(rx_data,
				rx_truncated? (u16_t)(MQTT_UTK_RX_DATA_MAX): rx_len,
				ack_frame, (u16_t)sizeof(ack_frame));
			if(ack_len > 0 && mqtt_client_is_connected(client)) {
				(void)mqtt_publish(client, TLM_TOPIC_ACK, ack_frame,
						   ack_len, 1 /* qos */, 0 /* retain */,
						   publish_done_cb, NULL);
			}
		} else {
			tm_printf((UB*)"mqtt: RX [%s] %d bytes%s\n", rx_topic,
				  (int)rx_len, rx_truncated? " (truncated)": "");
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " TLM_TOPIC_CMD "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

/* IS33: replaces the demo heartbeat. Publishes up to TLM_PUMP_MAX_PER_TICK
 * frames that subsystem 1 queued, then re-arms itself. */
static void pump_timeout(void *arg)
{
	u8_t	tid;
	u8_t	qos;
	u8_t	retain;
	u16_t	len;
	u32_t	n;

	(void)arg;

	for(n = 0; n < TLM_PUMP_MAX_PER_TICK; n++) {
		const char *topic;

		if(client == NULL || !mqtt_client_is_connected(client)) break;
		len = tlm_bridge_next_frame(&tid, tx_frame, (u16_t)sizeof(tx_frame));
		if(len == 0) break;
		topic = tlm_bridge_topic(tid, &qos, &retain);
		if(topic == NULL) continue;
		if(mqtt_publish(client, topic, tx_frame, len, qos, retain,
				publish_done_cb, NULL) != ERR_OK) {
			break;		/* output buffer full: try again next tick */
		}
	}

	sys_timeout(TLM_PUMP_PERIOD_MS, pump_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;
		u16_t	online_len;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks must be registered before subscribing, or the
		 * first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		/* IS33: subscribe to commands (QoS 1) */
		err = mqtt_subscribe(client, TLM_TOPIC_CMD, 1 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* IS33: tell the bridge, then publish retained "online" */
		online_len = tlm_bridge_connected(tx_frame, (u16_t)sizeof(tx_frame));
		if(online_len > 0) {
			(void)mqtt_publish(client, TLM_TOPIC_ONLINE, tx_frame,
					   online_len, 1 /* qos */, 1 /* retain */,
					   publish_done_cb, NULL);
		}

		/* Drop any chain left over from a previous connection before
		 * arming a new one, so there is only ever one pump chain (each
		 * one holds a MEMP_SYS_TIMEOUT slot). */
		sys_untimeout(pump_timeout, NULL);
		sys_timeout(TLM_PUMP_PERIOD_MS, pump_timeout, NULL);
	} else {
		/* IS33: stop accepting commands straight away */
		tlm_bridge_disconnected();

		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;
	u16_t					will_len;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	/* IS33: new session + "offline" last will for this connection */
	will_len = tlm_bridge_begin_connect(will_msg, (u16_t)sizeof(will_msg));

	client_info.client_id    = MQTT_CLIENT_ID;
	client_info.keep_alive   = TLM_KEEPALIVE_S;		/* IS33: was 60 */
	client_info.will_topic   = TLM_TOPIC_ONLINE;		/* IS33 */
	client_info.will_msg     = (const char *)will_msg;	/* IS33 */
	client_info.will_msg_len = (u8_t)will_len;		/* IS33 */
	client_info.will_qos     = 1;				/* IS33 */
	client_info.will_retain  = 1;				/* IS33 */

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
