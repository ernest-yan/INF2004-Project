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
/* Free distance needed along the 30/150-degree side ray to pass on that
 * side. Was 600: on a real table the diagonal ray almost always found
 * something nearer (2 of 3 runs on 17 Nov faulted "no safe side"). The
 * bypass only moves half the width + 15 cm sideways. PROVISIONAL. */
#define subsystem5_SIDE_CLEAR_MM      (400U)
/* If no side is clear: back off this far and scan once more. */
#define subsystem5_REVERSE_MM         (100U)
#define subsystem5_MAX_REVERSES       (1U)

/* Sensor-driven obstacle bypass (APP_MISSION_ENABLE only), see
 * subsystem5_bypass_step() in ultrasonic.c. PROVISIONAL, tune on the car. */
#define subsystem5_BYPASS_SPEED_MM_S      (100U) /* straight legs */
#define subsystem5_BYPASS_TURN_MM_S       (60U)  /* each wheel while pivoting */
#define subsystem5_REVERSE_SPEED_MM_S     (80U)
#define subsystem5_BYPASS_CLEAR_EXTRA_MM  (200U) /* front ray "past it" beyond
                                                    closest + this */
#define subsystem5_BYPASS_MIN_TURN_DEG    (20U)  /* estimates from the encoder */
#define subsystem5_BYPASS_EXTRA_TURN_DEG  (15U)  /* after clear: car body */
#define subsystem5_BYPASS_MAX_TURN_DEG    (130U)
#define subsystem5_CAR_HALF_WIDTH_MM      (75U)
#define subsystem5_CAR_LENGTH_MM          (180U) /* sensor to rear of the car */
#define subsystem5_SENSOR_TO_AXLE_MM      (100U) /* MEASURE: line sensors to
                                                    the drive wheel axle */
#define subsystem5_BYPASS_MARGIN_MM       (80U)  /* gap beside the obstacle */
#define subsystem5_BYPASS_OUT_MAX_MM      (600U)
#define subsystem5_BYPASS_PASS_BLIND_MM   (400U) /* never seen beside: passed */
#define subsystem5_BYPASS_PASS_MAX_MM     (900U)
#define subsystem5_BYPASS_SEEK_EXTRA_MM   (250U)
#define subsystem5_BYPASS_TIMEOUT_MS      (20000U)
/* Encoder scale, from S2's calibration (930 ticks per 210 mm wheel turn,
 * 120 mm track). Only sizes the legs and the turn limits: the turns that
 * matter are matched to each other by count, not by angle. */
#define subsystem5_TICKS_PER_M            (4429U)
#define subsystem5_TICKS_PER_DEG_X10      (46U)

typedef enum
{
    subsystem5_STATE_MONITOR = 0,
    subsystem5_STATE_WAIT_STOP,
    subsystem5_STATE_COARSE,
    subsystem5_STATE_FINE,
    subsystem5_STATE_PROFILE_READY,
    subsystem5_STATE_BYPASS,
    subsystem5_STATE_FAULT,
    subsystem5_STATE_REVERSE    /* backing off to rescan (no safe side) */
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
    uint16_t profile_count;  /* +1 per valid profile with an obstacle, even
                                when no bypass follows (telemetry trigger) */
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