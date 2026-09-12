#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

typedef enum
{
    PID_CTRL_REAR_RIGHT_SPEED = 0,
    PID_CTRL_REAR_LEFT_SPEED,
    PID_CTRL_FRONT_STEER_ANGLE,
    PID_CTRL_MAX
} pid_ctrl_id_t;

#define PID_DRIVE_BRAKE_TO_STOP_NONE   (0U)
#define PID_DRIVE_BRAKE_TO_STOP_ABS    (1U)
#define PID_DRIVE_BRAKE_TO_STOP_FORCE  (2U)

typedef struct
{
    float kp;
    float ki;
    float kd;

    float i_limit;
    float output_limit;
    float d_filter_alpha;

    float target;
    float feedback;
    float error;
    float integral;
    float derivative;
    float derivative_filt;
    float p_term;
    float i_term;
    float d_term;
    float output;
    float error_prev;
    float error_prev2;
    float output_prev;
    uint8 feedback_valid;
    uint16 accel_boost_ticks;
    uint8 direction_guard_active;
} pid_channel_state_t;

typedef struct
{
    uint8 initialized;
    pid_channel_state_t ch[PID_CTRL_MAX];
} pid_status_t;

void pid_init(void);
void pid_reset(pid_ctrl_id_t id);
void pid_reset_all(void);
void pid_set_gain(pid_ctrl_id_t id, float kp, float ki, float kd);
void pid_set_limit(pid_ctrl_id_t id, float i_limit, float out_limit);
void pid_set_d_filter(pid_ctrl_id_t id, float alpha);
void pid_set_target(pid_ctrl_id_t id, float target);
void pid_set_feedback(pid_ctrl_id_t id, float feedback);
float pid_calc(pid_ctrl_id_t id, float target, float feedback, float dt_s);
float pid_calc_drive(pid_ctrl_id_t id,
                     float target,
                     float feedback,
                     float dt_s,
                     uint8 brake_to_stop);
void pid_calc_all(float rear_r_target_speed,
                  float rear_r_feedback_speed,
                  float rear_l_target_speed,
                  float rear_l_feedback_speed,
                  float steer_target_angle,
                  float steer_feedback_angle,
                  float dt_s);
float pid_get_output(pid_ctrl_id_t id);
void pid_get_status(pid_status_t *status);
void pid_apply_to_dianji(void);

#endif
