#ifndef SUBSYSTEM5_ULTRASONIC_H
#define SUBSYSTEM5_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "profile.h"

/* Pin allocation (moved off GP4/GP5 to avoid the S3 centre line sensor on GP4).
 * !! GP0/GP1 are also UART0: do NOT build with CRASH_REPORT=1 or CONSOLE=uart
 * !! while the ultrasonic sensor is wired here.  Never use GP23, GP24, GP25
 * !! or GP29 (Pico W radio).
 * If TRIG and ECHO are wired the other way round, swap the two numbers.
 * !! HC-SR04 ECHO is a 5 V output: fit a divider (e.g. 1k from ECHO to GP1,
 * !! 2k from GP1 to GND) -- the RP2040 pins are NOT 5 V tolerant. */
#define subsystem5_TRIG_GPIO          (0U)
#define subsystem5_ECHO_GPIO          (1U)
#define subsystem5_SERVO_GPIO         (12U)

/* PROVISIONAL calibration values. Not validated stopping distances. */
#define subsystem5_DETECT_MM          (350U)
#define subsystem5_URGENT_MM          (150U)
#define subsystem5_OBSTACLE_ZONE_MM   (500U)
#define subsystem5_SIDE_CLEAR_MM      (600U)

/* PROVISIONAL obstacle bypass path (APP_MISSION_ENABLE only). The sideways
 * leg is half the obstacle width + MARGIN, the leg past it is the distance to
 * the obstacle + PASS, and the way back looks for the line for the sideways
 * distance + SEEK before giving up. */
#define subsystem5_BYPASS_MARGIN_MM   (150U)
#define subsystem5_BYPASS_PASS_MM     (300U)
#define subsystem5_BYPASS_SEEK_MM     (200U)
#define subsystem5_BYPASS_SPEED_MM_S  (80U)
#define subsystem5_BYPASS_TIMEOUT_MS  (10000U)

typedef enum
{
    subsystem5_STATE_MONITOR = 0,
    subsystem5_STATE_WAIT_STOP,
    subsystem5_STATE_COARSE,
    subsystem5_STATE_FINE,
    subsystem5_STATE_PROFILE_READY,
    subsystem5_STATE_BYPASS,
    subsystem5_STATE_FAULT
} subsystem5_state_t;

typedef enum
{
    subsystem5_EVENT_STOP_REQUIRED = 0,
    subsystem5_EVENT_PROFILE_READY,
    subsystem5_EVENT_FAULT
} subsystem5_event_type_t;

typedef enum
{
    subsystem5_FAULT_NONE = 0,
    subsystem5_FAULT_MONITOR_SENSOR,
    subsystem5_FAULT_SCAN_INVALID,
    subsystem5_FAULT_STOP_TIMEOUT,
    subsystem5_FAULT_EVENT_DELIVERY,
    subsystem5_FAULT_NO_SAFE_SIDE,
    subsystem5_FAULT_BYPASS
} subsystem5_fault_t;

typedef struct
{
    subsystem5_event_type_t type;
    subsystem5_fault_t fault;
    uint16_t front_mm;
    subsystem5_profile_t profile;
} subsystem5_event_t;

typedef struct
{
    subsystem5_state_t state;
    subsystem5_fault_t fault;
    uint16_t front_mm;
    uint8_t front_valid;
    uint8_t stop_required;  /* Sticky until recovery/reset; 0 during BYPASS. */
    uint8_t scan_complete;
    subsystem5_profile_t profile;
} subsystem5_status_t;

/* Queue adapter must be bounded/non-blocking (returns false if full).
 * Called from subsystem5 task, NEVER from the GPIO ISR. */
typedef bool (*subsystem5_event_publish_fn)(const subsystem5_event_t *event, void *context);

/* PORT: now returns a kernel error code (E_OK = 0, negative = error) because it
 * creates a mutex.  Call once from subsystem5_init(). */
int subsystem5_ultrasonic_init(subsystem5_event_publish_fn publish, void *context);

/* PORT: the echo pulse is now timed by polling the configured ECHO GPIO (the template has no GPIO
 * interrupt API), so subsystem5_echo_gpio_irq() no longer exists. */

/* Call every ~2 ms from exactly one micro T-Kernel task (not an IRQ).
 * PORT: while a ping is in flight this call spins on the echo pin for up to
 * subsystem5_ECHO_TIMEOUT_US (30 ms) so the pulse is timed to 1 us. */
void subsystem5_poll(void);

/* VehicleController calls AFTER Buddy 2 confirms wheels stopped. */
void subsystem5_confirm_robot_stopped(void);

/* Optional stationary scan; caller must ensure motors are stopped. */
bool subsystem5_request_stationary_scan(void);

/* Only after authorized bypass + Buddy 3's line-recovered event. */
void subsystem5_ack_line_recovered(void);

/* Deliberate fault reset while motors are physically stopped. */
void subsystem5_reset_fault_while_stopped(void);

/* Read-only consistent snapshot, e.g. for Buddy 1 MQTT telemetry. */
void subsystem5_get_status(subsystem5_status_t *result);

/* ---------------------------------------------------------------------
 *  Subsystem 5 glue (public interface)  -- merged in from subsystem5.h
 * --------------------------------------------------------------------- */
#define S5_PRI_MAIN		20	/* priority band 20-21 (TEAM.md) */
#define S5_POLL_PERIOD_MS	2	/* poll task period */

/* IF-02: S5 -> VehicleController / anyone.  Message buffer carrying one
   subsystem5_event_t per event (STOP_REQUIRED, PROFILE_READY, FAULT).
   Receive with:  subsystem5_event_t ev;
                  tk_rcv_mbf(mbf_s5_events, &ev, TMO_FEVR);
   The buffer is bounded: if nobody reads it, new events are refused and the
   driver latches FAULT_EVENT_DELIVERY (so also poll subsystem5_get_status). */
IMPORT ID	mbf_s5_events;

#endif /* SUBSYSTEM5_ULTRASONIC_H */