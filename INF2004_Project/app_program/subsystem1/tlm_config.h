/*
 *	tlm_config.h
 *	Subsystem 1 (Buddy 1) -- WiFi telemetry and commands: every tuning
 *	number in one place (no magic numbers anywhere else).
 *
 *	Plain C only (stdint), so it can be included from BOTH sides:
 *	  - our own files (which also include tk/tkernel.h), and
 *	  - lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c (which includes lwIP).
 *	Never include tk/tkernel.h, lwIP or pico-sdk headers from here.
 */

#ifndef TLM_CONFIG_H
#define TLM_CONFIG_H

#include <stdint.h>

/* ---- MQTT connection (broker IP/port/client ID: config/wifi_secrets.h) */
#define TLM_KEEPALIVE_S         (5u)    /* broker declares us gone after ~7.5 s */

/* ---- Topics ---------------------------------------------------------- */
#define TLM_TOPIC_ONLINE        "is33/car/online"      /* retained + last will */
#define TLM_TOPIC_STATUS        "is33/car/status"      /* 5 Hz,   QoS 0 */
#define TLM_TOPIC_HEARTBEAT     "is33/car/heartbeat"   /* 1 Hz,   QoS 0 */
#define TLM_TOPIC_EVENT         "is33/car/event"       /* change, QoS 1 */
#define TLM_TOPIC_ACK           "is33/car/ack"         /* per cmd, QoS 1 */
#define TLM_TOPIC_CMD           "is33/car/cmd"         /* we subscribe, QoS 1 */

/* Topic IDs: the first byte of every frame in the outgoing message
 * buffer, so the network side knows where to publish it. */
#define TLM_TID_STATUS          (1u)
#define TLM_TID_HEARTBEAT       (2u)
#define TLM_TID_EVENT           (3u)

/* ---- Rates ----------------------------------------------------------- */
#define TLM_TASK_TICK_MS        (50u)   /* telemetry task wakes this often */
#define TLM_STATUS_PERIOD_MS    (200u)  /* 5 Hz  (NFR3) */
#define TLM_HEARTBEAT_PERIOD_MS (1000u) /* 1 Hz  (NFR3) */
#define TLM_PUMP_PERIOD_MS      (20u)   /* network side empties the buffer */
#define TLM_PUMP_MAX_PER_TICK   (4u)    /* frames published per pump tick */

/* ---- Buffers --------------------------------------------------------- */
#define TLM_FRAME_MAX           (240u)  /* largest payload (JSON status) */
#define TLM_MBF_MSG_MAX         (TLM_FRAME_MAX + 1u)  /* + topic ID byte */
#define TLM_MBF_BYTES           (2048u) /* outgoing message buffer size */
#define TLM_CMD_MAX             (16u)   /* commands are 5 bytes */
#define TLM_WILL_MAX            (16u)

/* 1 = publish STATUS as JSON (readable "before" for the Minimum-Bit
 * comparison), 0 = bit-packed binary (the real format). */
#ifndef TLM_USE_JSON_STATUS
#define TLM_USE_JSON_STATUS     (0)
#endif

#endif /* TLM_CONFIG_H */
