#ifndef B2_MOTION_H
#define B2_MOTION_H
#include <stdbool.h>
#include <stdint.h>
#define B2_ENCODER_LEFT_GP    (6U)
#define B2_ENCODER_RIGHT_GP   (7U)
#define B2_MOTOR_LEFT_A_GP    (8U)
#define B2_MOTOR_LEFT_B_GP    (9U)
#define B2_MOTOR_RIGHT_A_GP   (10U)
#define B2_MOTOR_RIGHT_B_GP   (11U)

typedef enum
{
    B2_READY = 0, B2_TRACK_SPEED, B2_EXECUTE_GOAL,
    B2_BRAKED, B2_FAULT
} b2_state_t;
typedef enum
{
    B2_FAULT_NONE = 0, B2_FAULT_ENCODER_STALL,
    B2_FAULT_GOAL_TIMEOUT, B2_FAULT_BAD_CONFIG, B2_FAULT_EVENT_LOSS
} b2_fault_t;
typedef enum
{
    B2_CMD_SPEED = 0, B2_CMD_MOVE_FORWARD, B2_CMD_MOVE_BACKWARD,
    B2_CMD_TURN_LEFT, B2_CMD_TURN_RIGHT
} b2_cmd_type_t;
typedef enum
{
    B2_EVENT_GOAL_DONE = 0, B2_EVENT_STOPPED, B2_EVENT_BRAKED,
    B2_EVENT_FAULT
} b2_event_type_t;
/* Robot calibration and controller gains. Constant: lives in flash. */
typedef struct
{
    uint16_t ticks_per_rev_left;    /* encoder edges per wheel turn */
    uint16_t ticks_per_rev_right;
    uint16_t wheel_circumference_mm;
    uint16_t wheel_track_mm;
    uint16_t max_speed_mm_s;
    uint16_t max_pwm_permil;
    uint16_t encoder_stall_ms;
    uint16_t pwm_frequency_hz;
    uint16_t kp_q8;                 /* PID: permil per mm/s of error * 256 */
    uint16_t ki_q8;                 /* PID: per control loop (~20 ms) * 256 */
    uint16_t kd_q8;                 /* PID: permil per mm/s change per loop
                                     * * 256, on the measured speed; 0 = PI */
    uint16_t straight_kp_q8;        /* straight-line correction: mm/s of
                                     * set-point trim per tick of L/R
                                     * difference * 256; 0 = off */
    uint16_t straight_trim_max_mm_s; /* largest trim either way */
} b2_config_t;
/* CR1 minimum-bit representation (testing/cr1/DATA_REPRESENTATION.md).
 * Barr C forbids signed bit-fields, so signed values are stored
 * offset-binary: field = value + B2_SPEED_OFFSET (10 bits: -512..511 mm/s). */
#define B2_SPEED_OFFSET       (512)
#define B2_SPEED_FIELD_MAX    (1023)
#define B2_SPEED_LIMIT_MM_S   (511U)    /* largest speed a field can hold */
#define B2_DISTANCE_MAX_MM    (8191U)   /* 13-bit MOVE distance */
#define B2_TURN_MAX_DEG       (360U)    /*  9-bit TURN angle */
#define B2_TIMEOUT_UNIT_MS    (10U)     /* goal timeout stored in 10 ms units */
#define B2_TIMEOUT_MAX_MS     (10230U)  /* 10 bits x 10 ms */
#define B2_PWM_MAX_PERMIL     (1000U)   /* 10-bit PWM duty */
/* Fault bitmask: bit (f - 1) for each b2_fault_t f other than NONE. */
#define B2_FAULT_BIT(f)       ((uint8_t)(1U << ((uint32_t)(f) - 1U)))

/* A request, filled in on the CALLER's stack. Plain types on purpose:
 * b2_submit_command() range-checks every field and refuses bad requests
 * BEFORE packing, so an out-of-range value is never silently truncated by
 * a bit-field. Only the packed copy is shared between tasks. */
typedef struct
{
    b2_cmd_type_t type;
    int16_t left_mm_s;              /* B2_CMD_SPEED only; negative = reverse. */
    int16_t right_mm_s;
    uint16_t distance_mm;           /* MOVE goals, 1..B2_DISTANCE_MAX_MM. */
    uint16_t turn_degrees;          /* TURN goals, 1..B2_TURN_MAX_DEG. */
    uint16_t goal_speed_mm_s;       /* MOVE / TURN goals. */
    uint16_t timeout_ms;            /* MOVE / TURN goals, 1..B2_TIMEOUT_MAX_MS. */
} b2_command_t;

/* Shared snapshot (b2_get_status). Read the speeds with b2_speed_value(). */
typedef struct
{
    uint32_t left_encoder_edges;    /* ISR counter copy: full word, wraps. */
    uint32_t right_encoder_edges;
    uint32_t last_update_us;        /* 32-bit us, wraps every 71.6 min. */
    int16_t left_total_mm;          /* Signed estimate from commanded direction, */
    int16_t right_total_mm;         /* saturates at +-32767 mm. */
    uint32_t left_speed_ob : 10;    /* speed + B2_SPEED_OFFSET */
    uint32_t right_speed_ob : 10;
    uint32_t left_pwm_permil : 10;  /* 0..1000 */
    uint32_t brake_latched : 1;
    uint32_t : 1;
    uint16_t right_pwm_permil : 10;
    uint16_t state : 3;             /* b2_state_t, 5 values */
    uint16_t : 3;
    uint16_t deadline_misses;       /* saturates */
    uint8_t fault_mask : 4;         /* B2_FAULT_BIT() per active fault */
    uint8_t : 4;
    uint8_t goals_done;             /* MOVE / TURN goals finished, wraps. */
} b2_status_t;

typedef struct
{
    uint8_t type : 2;               /* b2_event_type_t, 4 values */
    uint8_t fault : 3;              /* b2_fault_t, 5 values */
    uint8_t : 3;
} b2_event_t;

typedef struct
{
    uint32_t started_us;
    uint8_t goals_done_at_start;
    uint8_t seen_running : 1;
    uint8_t : 7;
} b2_goal_t;

static inline int32_t b2_speed_value(uint32_t field)
{
    return (int32_t)field - B2_SPEED_OFFSET;
}
typedef enum
{
    B2_GOAL_RUNNING = 0, B2_GOAL_DONE, B2_GOAL_FAILED
} b2_goal_result_t;
typedef bool (*b2_event_publish_fn)(const b2_event_t *ev, void *ctx);
/* Call once before task startup. Configuration must reflect measured hardware. */
bool b2_init(const b2_config_t *config, b2_event_publish_fn publish, void *ctx);
/* GP6/GP7 rising-edge handler called by the team's SINGLE GPIO dispatcher. */
void b2_encoder_gpio_irq(unsigned int gpio, uint32_t events);
/* Task-only: call nominally every 10 ms, never inside an ISR. */
void b2_poll(void);
/* Latest-speed command replaces older speed requests; one goal at a time. */
bool b2_submit_command(const b2_command_t *cmd);
/* Normal STOP immediately zeros both motor PWM channels without latching. */
void b2_stop(void);
/* URGENT path: callable from VehicleController task, does not depend on queue. */
void b2_emergency_brake(void);
/* Must only be called after authorised reset AND physical safety checks. */
bool b2_authorized_reset_while_stopped(void);
void b2_get_status(b2_status_t *out);
/* Start a MOVE / TURN goal and follow it from a task without blocking.
 * FAILED means it was stopped, braked or faulted before reaching its goal. */
bool b2_goal_start(b2_goal_t *goal, const b2_command_t *cmd);
b2_goal_result_t b2_goal_check(b2_goal_t *goal);

/* Motion API in the write-up's terms (moveForward(distance),
 * turnLeft(angle), ... ; stop() is b2_stop()). Each one starts an
 * encoder-measured goal at a fixed speed and returns at once:
 *   - TRUE: accepted; follow it with b2_goal_check(p_goal) if p_goal is
 *     not NULL, or watch b2_status_t.goals_done.
 *   - FALSE: refused, because the robot is not READY (call b2_stop()
 *     first if it is tracking a speed), the brake is latched, or the
 *     amount is 0 / too large to finish inside the 10.23 s goal timeout
 *     (about 1.1 m for a move, any angle up to 360 deg for a turn).
 * The timeout is set automatically to twice the expected time + 1 s. */
#define B2_API_MOVE_SPEED_MM_S  (120U)
#define B2_API_TURN_SPEED_MM_S  (80U)
bool b2_move_forward(uint16_t distance_mm, b2_goal_t *p_goal);
bool b2_move_backward(uint16_t distance_mm, b2_goal_t *p_goal);
bool b2_turn_left(uint16_t angle_deg, b2_goal_t *p_goal);
bool b2_turn_right(uint16_t angle_deg, b2_goal_t *p_goal);
#endif
