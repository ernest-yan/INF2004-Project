#ifndef SUBSYSTEM5_ULTRASONIC_H
#define SUBSYSTEM5_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "profile.h"
#include "../subsystem2/motion_cal.h"

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
/* 20 Nov: 300 -> 230 (team: stop about 20 cm from the obstacle). The logs
 * show the car comes to rest 1-6 cm inside the trigger distance. From there
 * it turns on the spot (BP_TURN) before driving round. */
#define subsystem5_DETECT_MM          (230U)
#define subsystem5_URGENT_MM          (150U)
#define subsystem5_OBSTACLE_ZONE_MM   (500U)
/* Free distance needed along the 30/150-degree side ray to pass on that
 * side. Was 600: on a real table the diagonal ray almost always found
 * something nearer (2 of 3 runs on 17 Nov faulted "no safe side"). The
 * bypass only moves half the width + 15 cm sideways. PROVISIONAL. */
#define subsystem5_SIDE_CLEAR_MM      (400U)
/* 19 Nov: going around is preferred over reversing. If neither side has
 * SIDE_CLEAR_MM, the bypass still takes the more open side when it has at
 * least this much (car width 150 mm + margin). PROVISIONAL. */
#define subsystem5_SIDE_MIN_MM        (250U)
/* Last resort, only if both sides are under SIDE_MIN_MM: back off this far
 * and scan once more. */
#define subsystem5_REVERSE_MM         (100U)
#define subsystem5_MAX_REVERSES       (1U)

/* Obstacle bypass (APP_MISSION_ENABLE only), see subsystem5_bypass_step()
 * in ultrasonic.c. 19 Nov rewrite: the car steers on its own position,
 * worked out live from the wheel encoders, toward targets taken from the
 * measured obstacle profile. Both wheels always drive forward: no pivots,
 * no fixed turn angles or counts. Car geometry and wheel calibration come
 * from S2's motion_cal.h. PROVISIONAL tuning values, set on the car. */
#define subsystem5_BYPASS_SPEED_MM_S      (100U) /* centre of the car */
#define subsystem5_BYPASS_TURN_SPEED_MM_S (60U)  /* BP_TURN: each wheel */
#define subsystem5_BYPASS_TURN_MAX_DEG    (100U) /* BP_TURN: never saw past it */
#define subsystem5_BYPASS_STOPPED_MM_S    (10U)  /* measured speed counted as
                                                    stopped (after BP_TURN) */
/* Course obstacle, measured from photos 20 Nov against the 18 mm line:
 * about 230 mm wide, 120 mm deep (measured 20 Nov), 200 mm high. Only the depth is used, and
 * only when the side sensor cannot see the obstacle end (servo). */
#define subsystem5_OBSTACLE_DEPTH_MM      (120U)  /* 20 Nov: 140 (photo) -> 120 (ruler) */
#define subsystem5_BYPASS_PASS_AFTER_MM   (0U)   /* axle this far past its back
                                                    before turning back. 20 Nov
                                                    (team): 50 -> 0, the car's
                                                    front was ~19 cm past the
                                                    box before it turned back.
                                                    Turning toward the line
                                                    swings the rear away from
                                                    the box */
#define subsystem5_BYPASS_STEER_GAIN      (3U)   /* wheel speed difference,
                                                    mm/s per degree of
                                                    heading error */
#define subsystem5_BYPASS_SLEW_MM_S       (15U)  /* most a wheel request moves
                                                    per update (< S2's 20 mm/s
                                                    integral reset step) */
#define subsystem5_BYPASS_UPDATE_MS       (40U)  /* steering update period */
#define subsystem5_BYPASS_MAX_HEADING_DEG (60U)  /* never steer further off
                                                    the line direction (was
                                                    70: too sharp a swing) */
#define subsystem5_BYPASS_STEER_DIFF_PCT  (50U)  /* OUT/PASS/RETURN: wheel
                                                    difference at most this %
                                                    of the speed, so the inner
                                                    wheel keeps >= 1/3 of the
                                                    outer. 20 Nov 12:19: at 60 %
                                                    the inner wheel was asked
                                                    for 40 mm/s, did not turn,
                                                    and S2 braked (stall) */
#define subsystem5_BYPASS_LOOKAHEAD_MM    (200U) /* pursuit look-ahead: a
                                                    longer one gives a
                                                    flatter path */
#define subsystem5_BYPASS_LANE_TOL_MM     (20U)  /* lane reached within this */
#define subsystem5_BYPASS_ALIGN_TOL_DEG   (10U)  /* hand over to S3 within */
#define subsystem5_BYPASS_ALIGN_SPEED_MM_S (60U) /* onto the line, slowly */
#define subsystem5_BYPASS_MAX_LOOKS       (2U)   /* ALIGN: looks before S3 takes over */
#define subsystem5_BYPASS_ALIGN_TURNS     (4U)   /* ALIGN: turns on the spot onto
                                                    the line (first + coast-backs) */
#define subsystem5_BYPASS_LOOK_DEG        (60U)  /* LOOK: turn this far each
                                                    way searching for the line */
#define subsystem5_BYPASS_APPROACH_DEG    (35U)  /* 20 Nov: 30 -> 35 (team: shorter). RETURN: least angle at which the
                                                    car closes on the line */
#define subsystem5_LEFT_IR_OFFSET_MM      (37U)  /* left IR sensor, left of the
                                                    centre one (photo 20 Nov) */
#define subsystem5_LINE_WIDTH_MM          (18U)  /* black line width: Barcode /
                                                    Line Specification (ref 03) */
#define subsystem5_BYPASS_LINE_GATE_MM    (100U) /* black this near where the
                                                    line is expected counts as
                                                    the line ... */
#define subsystem5_BYPASS_LINE_GATE_PCT   (20U)  /* ... plus this % of the
                                                    distance driven (encoder
                                                    pose drift) */
#define subsystem5_REVERSE_SPEED_MM_S     (80U)
/* 19 Nov 22:17: reversing from standstill at 80 mm/s, a wheel did not turn
 * within S2's 500 ms stall window (about 21 % PWM) and S2 latched the brake
 * (also 19:39, a right wheel started backwards). Each wheel is asked for
 * this until it is measured moving, then REVERSE_SPEED. */
#define subsystem5_REVERSE_START_MM_S     (150U)
#define subsystem5_CAR_HALF_WIDTH_MM      (75U)
/* (CAR_LENGTH_MM, sensor to rear 180 mm, no longer used: 20 Nov) */
#define subsystem5_SENSOR_TO_AXLE_MM      (120U) /* line sensors to the drive
                                                    wheel axle, measured on
                                                    the car 19 Nov (12 cm) */
#define subsystem5_BYPASS_MARGIN_MM       (40U)  /* 20 Nov: 60 -> 40 (team: shorter; sim, 23x12 box, 35 deg: 81/90, no touch, line ~22 cm behind vs ~25). Gap beside the
                                                    obstacle, as close as is safe
                                                    (team). Full-run simulation,
                                                    course obstacle, 20 cm stop,
                                                    30 deg approach, wheels within
                                                    5 % (track now measured):
                                                    6 cm 72/72 with no contact;
                                                    uncalibrated 6 cm touched 4/72
                                                    against 20/72 at 5 cm. Was 10 cm.
                                                    Earlier note:
                                                    pass as close as is safe.
                                                    Was 80 when the lane came
                                                    only from the scan; PASS
                                                    now measures the side */
#define subsystem5_BYPASS_PREFER_LEFT     (1U)   /* 1 = pass on the left
                                                    whenever it has room */
#define subsystem5_BYPASS_SIDE_GAIN_PCT   (25U)  /* PASS: share of each side
                                                    reading taken into the
                                                    edge estimate (filter) */
#define subsystem5_BYPASS_SIDE_STEP_MM    (60U)  /* PASS: most one reading may
                                                    move the edge (outliers) */
#define subsystem5_BYPASS_OUT_MAX_MM      (600U) /* furthest lane from the line */
#define subsystem5_BYPASS_PASS_BLIND_MM   (400U) /* never seen beside: passed */
#define subsystem5_BYPASS_PASS_MAX_MM     (900U)
#define subsystem5_BYPASS_TIMEOUT_MS      (40000U) /* safety net only: each
                                                    phase has a distance
                                                    limit. 1.2-1.6 m of
                                                    path at ~45 mm/s real */

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

/* CR1: fault bitmask, bit (f - 1) for each subsystem5_fault_t f != NONE. */
#define subsystem5_FAULT_BIT(f)  ((uint8_t)(1U << ((uint32_t)(f) - 1U)))

/* IF-02 message. CR1: 32 -> 20 bytes. */
typedef struct
{
    subsystem5_profile_t profile;
    uint16_t front_mm : 12;     /* 0..4000, subsystem5_UNKNOWN_MM */
    uint16_t type : 2;          /* subsystem5_event_type_t, 3 values */
    uint16_t : 2;
    uint8_t fault : 3;          /* subsystem5_fault_t of THIS event */
    uint8_t : 5;
} subsystem5_event_t;

/* CR1: 38 -> 20 bytes. */
typedef struct
{
    subsystem5_profile_t profile;
    uint16_t front_mm : 12;     /* 0..4000, subsystem5_UNKNOWN_MM */
    uint16_t state : 3;         /* subsystem5_state_t, 8 values */
    uint16_t front_valid : 1;
    uint8_t fault_mask : 6;     /* subsystem5_FAULT_BIT() per active fault */
    uint8_t stop_required : 1;  /* Sticky until recovery/reset; 0 during BYPASS. */
    uint8_t scan_complete : 1;
    uint8_t profile_count;      /* +1 per valid profile with an obstacle, even
                                   when no bypass follows (telemetry trigger);
                                   wraps, readers only compare for change */
    uint8_t bypass_phase : 3;   /* 20 Nov: 0 none, 1 out, 2 pass, 3 return,
                                   4 align, 5 turn on the spot, 6 look both
                                   ways for the line. Kept after a
                                   FAULT so telemetry shows where a bypass
                                   failed; cleared back in MONITOR */
    uint8_t : 5;
    uint8_t coarse_cm[subsystem5_COARSE_COUNT]; /* 20 Nov, diagnostics: the
                                   last coarse scan, 30..150 deg, in cm;
                                   254 = open space, 255 = no echo at all */
} subsystem5_status_t;

/* Lowest active fault as an enum value, for the unchanged 3-bit MQTT field. */
static inline uint8_t subsystem5_first_fault(uint8_t fault_mask)
{
    uint8_t f;
    for (f = 1U; f <= (uint8_t)subsystem5_FAULT_BYPASS; f++)
    {
        if ((fault_mask & subsystem5_FAULT_BIT(f)) != 0U)
        {
            return f;
        }
    }
    return (uint8_t)subsystem5_FAULT_NONE;
}

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