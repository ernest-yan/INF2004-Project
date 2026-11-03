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
typedef struct
{
    uint16_t ticks_per_wheel_rev;
    uint16_t wheel_circumference_mm;
    uint16_t wheel_track_mm;
    uint16_t max_speed_mm_s;
    uint16_t max_pwm_permil;
    uint16_t encoder_stall_ms;
    uint16_t pwm_frequency_hz;
    uint16_t kp_q8;                 /* PI: proportional gain * 256. */
    uint16_t ki_q8;                 /* Integral per 10ms * 256. */
} b2_config_t;
typedef struct
{
    b2_cmd_type_t type;
    int16_t left_mm_s;              /* B2_CMD_SPEED only; negative = reverse. */
    int16_t right_mm_s;
    uint16_t distance_mm;           /* MOVE goals. */
    uint16_t turn_degrees;          /* TURN goals. */
    uint16_t goal_speed_mm_s;       /* MOVE / TURN goals. */
    uint32_t timeout_ms;            /* MOVE / TURN goals, must be nonzero. */
} b2_command_t;
typedef struct
{
    b2_state_t state;
    b2_fault_t fault;
    uint8_t brake_latched;
    uint32_t left_encoder_edges;
    uint32_t right_encoder_edges;
    int32_t left_total_mm;          /* Signed estimate from commanded direction. */
    int32_t right_total_mm;
    int32_t left_speed_mm_s;
    int32_t right_speed_mm_s;
    uint16_t left_pwm_permil;
    uint16_t right_pwm_permil;
    uint32_t deadline_misses;
    uint64_t last_update_us;
} b2_status_t;
typedef struct
{
    b2_event_type_t type;
    b2_fault_t fault;
} b2_event_t;
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
#endif
