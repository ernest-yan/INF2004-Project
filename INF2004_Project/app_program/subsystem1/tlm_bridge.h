/*
 *	tlm_bridge.h
 *	Subsystem 1 (Buddy 1) -- the ONLY doorway between the WiFi service
 *	task (mqtt_utk.c) and our telemetry code.
 *
 *	Every function here is called ONLY from mqtt_utk.c, i.e. from inside
 *	the kernel's cyw43_utk_task (TEAM.md section 3: no student task may
 *	call cyw43/lwIP/MQTT). The bridge never calls lwIP itself; it hands
 *	bytes and topic names back and mqtt_utk.c does the publishing.
 *
 *	Plain C types only (stdint), so mqtt_utk.c can include this file
 *	without pulling in tk/tkernel.h.
 */

#ifndef TLM_BRIDGE_H
#define TLM_BRIDGE_H

#include <stdint.h>
#include "tlm_config.h"

/* About to call mqtt_client_connect(): pick a new session number and
 * write the "last will" (offline) message. Returns its length. */
uint16_t tlm_bridge_begin_connect(uint8_t *p_will, uint16_t cap);

/* Broker accepted us: throw away stale queued frames, go online, and
 * write the retained "online" message. Returns its length. */
uint16_t tlm_bridge_connected(uint8_t *p_online, uint16_t cap);

/* Connection lost or refused: stop accepting commands. */
void tlm_bridge_disconnected(void);

/* Next queued frame (never waits). Returns payload length, 0 if none.
 * *p_topic_id says which topic it belongs to (TLM_TID_...). */
uint16_t tlm_bridge_next_frame(uint8_t *p_topic_id, uint8_t *p_buf, uint16_t cap);

/* Topic name, QoS and retain flag for a topic ID (NULL if unknown). */
const char *tlm_bridge_topic(uint8_t topic_id, uint8_t *p_qos, uint8_t *p_retain);

/* A complete message arrived on TLM_TOPIC_CMD. Check it, act on it,
 * and write the reply (ACK) for TLM_TOPIC_ACK. Returns ACK length. */
uint16_t tlm_bridge_handle_cmd(const uint8_t *p_data, uint16_t len,
                               uint8_t *p_ack, uint16_t cap);

#endif /* TLM_BRIDGE_H */
