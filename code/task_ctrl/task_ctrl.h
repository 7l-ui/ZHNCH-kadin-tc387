#ifndef _TASK_CTRL_H_
#define _TASK_CTRL_H_

#include "zf_common_headfile.h"

#define CTRL_PWM_TEST_DUTY_DEFAULT      (0)
#define CTRL_PWM_TEST_DUTY_STEP         (200)
#define CTRL_PWM_TEST_DUTY_MAX          (3000)
#define CTRL_PWM_TEST_DUTY_MIN          (-3000)

#define TASK_CTRL_STATE_IDLE      (0U)
#define TASK_CTRL_STATE_RUNNING   (1U)
#define TASK_CTRL_STATE_FINISHED  (2U)

#define CTRL_TASK3_FOLLOW_PHASE_IDLE       (0U)
#define CTRL_TASK3_FOLLOW_PHASE_WAIT       (1U)
#define CTRL_TASK3_FOLLOW_PHASE_TRACK      (2U)
#define CTRL_TASK3_FOLLOW_PHASE_LOST       (3U)
#define CTRL_TASK3_FOLLOW_PHASE_STOPPING   (4U)
#define CTRL_TASK3_FOLLOW_PHASE_SAVE_FAIL  (5U)

#define CTRL_TASK3_FOLLOW_ERR_NONE         (0U)
#define CTRL_TASK3_FOLLOW_ERR_BUSY         (1U)
#define CTRL_TASK3_FOLLOW_ERR_CAMERA       (2U)
#define CTRL_TASK3_FOLLOW_ERR_VOICE        (3U)
#define CTRL_TASK3_FOLLOW_ERR_SD_BUSY      (4U)
#define CTRL_TASK3_FOLLOW_ERR_CALIBRATION  (5U)
#define CTRL_TASK3_FOLLOW_ERR_INS          (6U)
#define CTRL_TASK3_FOLLOW_ERR_PATH         (7U)
#define CTRL_TASK3_FOLLOW_ERR_SAVE         (8U)

#define CTRL_STEER_SCAN_PHASE_IDLE        (0U)
#define CTRL_STEER_SCAN_PHASE_LEFT        (1U)
#define CTRL_STEER_SCAN_PHASE_WAIT        (2U)
#define CTRL_STEER_SCAN_PHASE_RIGHT       (3U)
#define CTRL_STEER_SCAN_PHASE_DONE        (4U)
#define CTRL_STEER_SCAN_PHASE_SENSOR_ERR  (5U)
#define CTRL_STEER_SCAN_PHASE_TIMEOUT     (6U)
#define CTRL_STEER_SCAN_PHASE_RETURN      (7U)

extern volatile uint8 g_task_ctrl_state;
extern volatile uint8 g_task_ctrl_id;
extern volatile int16 g_pwm_test_duty;
extern volatile uint8 g_steer_scan_phase;
extern volatile uint8 g_steer_scan_success;
extern volatile int16 g_steer_scan_left_raw;
extern volatile int16 g_steer_scan_right_raw;
extern volatile int16 g_steer_scan_center_raw;
extern volatile int16 g_steer_scan_span_counts;
extern volatile uint8 g_task1_reverse_enable;
extern volatile uint8 g_ctrl_drive_forward_only;

typedef struct
{
    uint8 motion;
    uint8 last_cmd;
    uint8 motion_cmd;
    uint8 route_cmd;
    uint8 route_step_index;
    uint8 route_step_count;
    float progress;
    float target;
    const char *motion_name;
    const char *stop_reason;
} ctrl_task2_status_t;

typedef struct
{
    uint8 valid;
    uint8 motion;
    uint8 route_cmd;
    uint8 route_step_index;
    uint8 route_step_count;
    uint8 path_index;
    uint8 path_count;
    uint8 final_approach;
    uint8 return_path;
    float target_x_m;
    float target_y_m;
    float preview_x_m;
    float preview_y_m;
    float desired_yaw_deg;
    float yaw_error_deg;
    float lateral_error_m;
    float projection_m;
    float dist_to_target_m;
    float dist_to_park_m;
    float target_angle_deg;
    float target_speed_mps;
} ctrl_task2_path_debug_t;

typedef struct
{
    uint8 active;
    uint8 record_path;
    uint8 phase;
    uint8 error;
    uint8 target_valid;
    uint8 target_stable;
    uint8 confidence_pct;
    uint8 distance_valid;
    int16 bearing_deg_x10;
    uint16 bbox_height;
    uint16 distance_cm;
    uint16 path_samples;
    uint32 frame_age_ms;
    float target_speed_mps;
    float target_angle_deg;
} ctrl_task3_follow_status_t;

void ctrl_cmd_set_pid(float target_speed_mps, float target_angle_deg, uint8 steer_closed_loop);
void ctrl_cmd_set_pid_brake_to_stop(float target_angle_deg, uint8 steer_closed_loop);
void ctrl_cmd_set_pid_force_brake_to_stop(float target_angle_deg, uint8 steer_closed_loop);
void ctrl_cmd_set_pwm(int16 duty);
void ctrl_cmd_set_steer_pwm(int16 duty);
void ctrl_pwm_test_reset(void);
void ctrl_pwm_test_step(void);
void ctrl_steer_scan_start(void);
void ctrl_steer_scan_stop(void);

uint8 ctrl_logger_start(void);
uint8 ctrl_logger_start_task(uint8 task_id);
uint8 ctrl_logger_restart_task_src(uint8 task_id, uint8 target_src);
void ctrl_logger_stop(void);
uint8 ctrl_logger_is_running(void);

void ctrl_task_start(uint8 task_id);
void ctrl_task_stop(void); 
void ctrl_task_pause_for_external_source(void);
void ctrl_task3_remote_ch6_request(void);
uint8 ctrl_task3_follow_start(void);
uint8 ctrl_task3_follow_test_start(void);
void ctrl_task3_follow_finish_request(void);
uint8 ctrl_task3_follow_is_active(void);
void ctrl_task3_follow_get_status(ctrl_task3_follow_status_t *out_status);
void ctrl_task_update(void);
uint8 ctrl_task_is_running(void);
void ctrl_task1_reverse_set(uint8 enable);
void ctrl_task1_reverse_toggle(void);
uint8 ctrl_task1_reverse_get(void);
uint8 ctrl_task2_handle_voice_command(uint8 cmd);
void ctrl_task2_get_status(ctrl_task2_status_t *out_status);
void ctrl_task2_get_path_debug(ctrl_task2_path_debug_t *out_debug);
void ctrl_task2_sequence_set_enable(uint8 enable);
void ctrl_task2_sequence_clear(void);
uint8 ctrl_task2_sequence_push(uint8 cmd);
void ctrl_task2_sequence_update(void);
uint8 ctrl_task2_sequence_is_busy(void);
uint8 ctrl_task2_output_pop(uint8 *cmd);

void ctrl_mode_update(void);
 
#endif
