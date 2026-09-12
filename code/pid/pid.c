#include "pid.h"

extern volatile uint8 g_ctrl_drive_forward_only;
#include "pid_config.h"
#include "dianji.h"

static pid_status_t g_pid = {0};
static int16 g_pid_drive_duty_r = 0;
static int16 g_pid_drive_duty_l = 0;
static uint8 g_pid_steer_breakaway_active = 0U;
static float g_pid_steer_breakaway_sign = 0.0f;

typedef struct
{
    uint8 valid;
    float prev_feedback;
    float brake_limit;
} pid_drive_abs_state_t;

static pid_drive_abs_state_t g_pid_drive_abs[PID_CTRL_MAX] = {0};

static float pid_absf(float x);
static float pid_clipf(float v, float min_v, float max_v);
static float pid_blend_ratio(float value, float low, float high);
static float pid_safe_limit(float x);
static float pid_wrap_error_cyclic(float err, float cycle);
static int16 pid_clip_i32_to_i16(int32 v);
static void pid_steer_breakaway_reset(void);
static void pid_apply_steer_fuzzy_gain(float err,
                                       float d_raw,
                                       float *kp_eff,
                                       float *kd_eff);
static void pid_apply_drive_fuzzy_gain(float err,
                                       float *kp_eff,
                                       float *ki_eff);
static float pid_apply_steer_breakaway_comp(pid_channel_state_t *ch);
static void pid_drive_abs_reset(pid_ctrl_id_t id);
static float pid_apply_drive_accel_assist(pid_ctrl_id_t id, pid_channel_state_t *ch);
static float pid_apply_drive_abs_brake_limit(pid_ctrl_id_t id,
                                             pid_channel_state_t *ch,
                                             float dt_s,
                                             uint8 brake_to_stop);
static float pid_apply_drive_direction_guard(pid_ctrl_id_t id, pid_channel_state_t *ch);
static float pid_calc_position(pid_channel_state_t *ch, float target, float feedback, float dt_s, uint8 cyclic_error);
static float pid_calc_incremental(pid_channel_state_t *ch,
                                  float target,
                                  float feedback,
                                  float dt_s,
                                  uint8 zero_target_brake);

static uint8 pid_id_valid(pid_ctrl_id_t id)
{
    return ((uint8)id < (uint8)PID_CTRL_MAX);
}

static float pid_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float pid_clipf(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        return min_v;
    }
    if (v > max_v)
    {
        return max_v;
    }
    return v;
}

static float pid_blend_ratio(float value, float low, float high)
{
    if (value <= low)
    {
        return 0.0f;
    }
    if (value >= high)
    {
        return 1.0f;
    }
    return (value - low) / (high - low);
}

static float pid_safe_limit(float x)
{
    float y = pid_absf(x);
    if (y < 1e-6f)
    {
        y = 1e-6f;
    }
    return y;
}

static void pid_apply_drive_fuzzy_gain(float err,
                                       float *kp_eff,
                                       float *ki_eff)
{
#if PID_DRIVE_FUZZY_ENABLE
    float blend;
    float kp_scale;
    float ki_scale;

    if ((kp_eff == NULL) || (ki_eff == NULL))
    {
        return;
    }

    blend = pid_blend_ratio(pid_absf(err),
                            PID_DRIVE_FUZZY_ERR_LOW_ECOD,
                            PID_DRIVE_FUZZY_ERR_HIGH_ECOD);
    kp_scale = PID_DRIVE_FUZZY_KP_LOW_SCALE +
               (PID_DRIVE_FUZZY_KP_HIGH_SCALE -
                PID_DRIVE_FUZZY_KP_LOW_SCALE) * blend;
    ki_scale = PID_DRIVE_FUZZY_KI_LOW_SCALE +
               (PID_DRIVE_FUZZY_KI_HIGH_SCALE -
                PID_DRIVE_FUZZY_KI_LOW_SCALE) * blend;
    *kp_eff *= kp_scale;
    *ki_eff *= ki_scale;
#else
    (void)err;
    (void)kp_eff;
    (void)ki_eff;
#endif
}

static float pid_wrap_error_cyclic(float err, float cycle)
{
    float half_cycle;

    if (cycle <= 0.0f)
    {
        return err;
    }

    half_cycle = cycle * 0.5f;
    while (err > half_cycle)
    {
        err -= cycle;
    }
    while (err < -half_cycle)
    {
        err += cycle;
    }

    return err;
}

static int16 pid_clip_i32_to_i16(int32 v)
{
    if (v > 32767)
    {
        return 32767;
    }
    if (v < -32768)
    {
        return -32768;
    }
    return (int16)v;
}

static void pid_steer_breakaway_reset(void)
{
    g_pid_steer_breakaway_active = 0U;
    g_pid_steer_breakaway_sign = 0.0f;
}

static void pid_apply_steer_fuzzy_gain(float err,
                                       float d_raw,
                                       float *kp_eff,
                                       float *kd_eff)
{
#if PID_STEER_FUZZY_ENABLE
    float err_abs;
    float derr_abs;
    float large_err_blend;
    float small_err_blend;
    float rate_blend;
    float kp_scale;
    float kd_scale;

    if ((kp_eff == NULL) || (kd_eff == NULL))
    {
        return;
    }

    err_abs = pid_absf(err);
    derr_abs = pid_absf(d_raw);
    large_err_blend = pid_blend_ratio(err_abs,
                                      PID_STEER_FUZZY_ERR_LOW_DEG,
                                      PID_STEER_FUZZY_ERR_HIGH_DEG);
    small_err_blend = 1.0f - large_err_blend;
    rate_blend = pid_clipf(derr_abs / PID_STEER_FUZZY_DERR_HIGH_DPS,
                           0.0f,
                           1.0f);

    kp_scale = 1.0f +
               (PID_STEER_FUZZY_KP_LARGE_SCALE - 1.0f) * large_err_blend;
    if (err_abs < PID_STEER_FUZZY_ERR_LOW_DEG)
    {
        kp_scale = PID_STEER_FUZZY_KP_SMALL_SCALE;
    }

    kd_scale = 1.0f +
               (PID_STEER_FUZZY_KD_RATE_SCALE - 1.0f) * rate_blend +
               (PID_STEER_FUZZY_KD_SMALL_SCALE - 1.0f) * small_err_blend;
    kd_scale = pid_clipf(kd_scale,
                         1.0f,
                         PID_STEER_FUZZY_KD_RATE_SCALE +
                         PID_STEER_FUZZY_KD_SMALL_SCALE - 1.0f);

    *kp_eff *= kp_scale;
    *kd_eff *= kd_scale;
#else
    (void)err;
    (void)d_raw;
    (void)kp_eff;
    (void)kd_eff;
#endif
}

static float pid_apply_steer_breakaway_comp(pid_channel_state_t *ch)
{
#if PID_STEER_BREAKAWAY_COMP_ENABLE
    float min_duty;
    float out_limit;
    float abs_error;
    float error_sign;

    if (NULL == ch)
    {
        return 0.0f;
    }

    abs_error = pid_absf(ch->error);
    error_sign = (ch->error >= 0.0f) ? 1.0f : -1.0f;

    if (0U != g_pid_steer_breakaway_active)
    {
        if ((abs_error <= PID_STEER_BREAKAWAY_RELEASE_ERROR_DEG) ||
            ((g_pid_steer_breakaway_sign * ch->error) <= 0.0f))
        {
            pid_steer_breakaway_reset();
        }
    }

    if ((0U == g_pid_steer_breakaway_active) &&
        (abs_error >= PID_STEER_BREAKAWAY_ERROR_DEG))
    {
        g_pid_steer_breakaway_active = 1U;
        g_pid_steer_breakaway_sign = error_sign;
    }

    if (0U == g_pid_steer_breakaway_active)
    {
        return ch->output;
    }

    min_duty = pid_absf(PID_STEER_BREAKAWAY_DUTY);
    out_limit = pid_safe_limit(ch->output_limit);
    if (min_duty > out_limit)
    {
        min_duty = out_limit;
    }

    if (pid_absf(ch->output) < min_duty)
    {
        ch->output = (ch->error >= 0.0f) ? min_duty : -min_duty;
        ch->output_prev = ch->output;
    }
#endif

    return (NULL != ch) ? ch->output : 0.0f;
}

static void pid_drive_abs_reset(pid_ctrl_id_t id)
{
    if (0U == pid_id_valid(id))
    {
        return;
    }

    g_pid_drive_abs[(uint8)id].valid = 0U;
    g_pid_drive_abs[(uint8)id].prev_feedback = 0.0f;
    g_pid_drive_abs[(uint8)id].brake_limit = PID_DRIVE_ABS_MIN_BRAKE_PWM;
}

static float pid_drive_accel_assist_base(pid_ctrl_id_t id)
{
    if (id == PID_CTRL_REAR_LEFT_SPEED)
    {
        return (float)PID_DRIVE_L_BREAKAWAY_DUTY;
    }

    return (float)PID_DRIVE_R_BREAKAWAY_DUTY;
}

static float pid_apply_drive_accel_assist(pid_ctrl_id_t id, pid_channel_state_t *ch)
{
#if PID_DRIVE_ACCEL_ASSIST_ENABLE
    float abs_target;
    float abs_feedback;
    float assist_duty;
    float out_limit;
    float output_sign;

    if ((NULL == ch) ||
        ((id != PID_CTRL_REAR_RIGHT_SPEED) && (id != PID_CTRL_REAR_LEFT_SPEED)))
    {
        return (NULL != ch) ? ch->output : 0.0f;
    }

    abs_target = pid_absf(ch->target);
    if (abs_target < PID_DRIVE_ACCEL_ASSIST_MIN_TARGET_ECOD)
    {
        return ch->output;
    }

    if ((ch->target * ch->error) <= 0.0f)
    {
        return ch->output;
    }

    abs_feedback = pid_absf(ch->feedback);
    if (abs_feedback >= (abs_target * PID_DRIVE_ACCEL_ASSIST_EXIT_RATIO))
    {
        return ch->output;
    }

    assist_duty = pid_drive_accel_assist_base(id) +
                  (abs_target * PID_DRIVE_ACCEL_ASSIST_GAIN);
    out_limit = pid_safe_limit(ch->output_limit);
    assist_duty = pid_clipf(assist_duty, 0.0f, PID_DRIVE_ACCEL_ASSIST_MAX_DUTY);
    if (assist_duty > out_limit)
    {
        assist_duty = out_limit;
    }

    output_sign = (ch->target >= 0.0f) ? 1.0f : -1.0f;
    if (pid_absf(ch->output) < assist_duty)
    {
        ch->output = output_sign * assist_duty;
        ch->output_prev = ch->output;
    }
#else
    (void)id;
#endif

    return (NULL != ch) ? ch->output : 0.0f;
}

static float pid_apply_drive_abs_brake_limit(pid_ctrl_id_t id,
                                             pid_channel_state_t *ch,
                                             float dt_s,
                                             uint8 brake_to_stop)
{
#if PID_DRIVE_ABS_BRAKE_ENABLE
    pid_drive_abs_state_t *abs_state;
    float speed_eps;
    float min_limit;
    float max_limit;
    float near_zero_speed;
    float near_zero_limit;
    float release_decel;
    float recover_decel;
    float step_up;
    float step_down;
    float launch_noise_speed;
    float abs_feedback;
    float abs_prev_feedback;
    float decel;
    float limit;
    uint8 near_zero_launch_noise = 0U;
    uint8 braking = 0U;
#endif

    if (NULL == ch)
    {
        return 0.0f;
    }

#if PID_DRIVE_ABS_BRAKE_ENABLE
    if ((id != PID_CTRL_REAR_RIGHT_SPEED) && (id != PID_CTRL_REAR_LEFT_SPEED))
    {
        return ch->output;
    }

    if (dt_s <= 1e-6f)
    {
        dt_s = PID_CTRL_DT_DEFAULT_S;
    }

    abs_state = &g_pid_drive_abs[(uint8)id];
    speed_eps = pid_absf(PID_DRIVE_ABS_SPEED_EPS_ECOD);
    min_limit = pid_absf(PID_DRIVE_ABS_MIN_BRAKE_PWM);
    max_limit = pid_absf(PID_DRIVE_ABS_MAX_BRAKE_PWM);
    if (brake_to_stop == PID_DRIVE_BRAKE_TO_STOP_FORCE)
    {
        min_limit = pid_absf(PID_DRIVE_ABS_STRONG_MIN_BRAKE_PWM);
        max_limit = pid_absf(PID_DRIVE_ABS_STRONG_MAX_BRAKE_PWM);
    }
    near_zero_speed = pid_absf(PID_DRIVE_ABS_NEAR_ZERO_ECOD);
    near_zero_limit = pid_absf(PID_DRIVE_ABS_NEAR_ZERO_PWM);
    launch_noise_speed = pid_absf(PID_DRIVE_ABS_LAUNCH_NOISE_ECOD);
    release_decel = pid_absf(PID_DRIVE_ABS_RELEASE_DECEL_ECOD_S2);
    recover_decel = pid_absf(PID_DRIVE_ABS_RECOVER_DECEL_ECOD_S2);
    step_up = pid_absf(PID_DRIVE_ABS_LIMIT_STEP_UP_PWM);
    step_down = pid_absf(PID_DRIVE_ABS_LIMIT_STEP_DOWN_PWM);

    if (max_limit < min_limit)
    {
        max_limit = min_limit;
    }
    if (near_zero_limit > max_limit)
    {
        near_zero_limit = max_limit;
    }

    if (0U == abs_state->valid)
    {
        abs_state->valid = 1U;
        abs_state->prev_feedback = ch->feedback;
        abs_state->brake_limit = min_limit;
    }

    if (launch_noise_speed < near_zero_speed)
    {
        launch_noise_speed = near_zero_speed;
    }

    // Ignore only near-zero sign chatter during launch; real reverse braking still uses ABS.
    if ((((ch->target > speed_eps) && (ch->output > 0.0f) && (ch->feedback < -speed_eps)) ||
         ((ch->target < -speed_eps) && (ch->output < 0.0f) && (ch->feedback > speed_eps))) &&
        (pid_absf(ch->feedback) <= launch_noise_speed))
    {
        near_zero_launch_noise = 1U;
    }

    if ((0U == near_zero_launch_noise) &&
        (((ch->feedback > speed_eps) && (ch->output < 0.0f)) ||
         ((ch->feedback < -speed_eps) && (ch->output > 0.0f))))
    {
        braking = 1U;
    }

    if (0U != braking)
    {
        abs_feedback = pid_absf(ch->feedback);
        abs_prev_feedback = pid_absf(abs_state->prev_feedback);
        decel = 0.0f;
        limit = abs_state->brake_limit;

        if (((ch->feedback > speed_eps) && (abs_state->prev_feedback > speed_eps)) ||
            ((ch->feedback < -speed_eps) && (abs_state->prev_feedback < -speed_eps)))
        {
            if (abs_prev_feedback > abs_feedback)
            {
                decel = (abs_prev_feedback - abs_feedback) / dt_s;
            }
        }

        if (abs_feedback <= near_zero_speed)
        {
            limit = near_zero_limit;
        }
        else
        {
            if (limit < min_limit)
            {
                limit = min_limit;
            }
            else if (limit > max_limit)
            {
                limit = max_limit;
            }

            if (decel > release_decel)
            {
                limit -= step_down;
            }
            else if (decel < recover_decel)
            {
                limit += step_up;
            }

            limit = pid_clipf(limit, min_limit, max_limit);
        }

        abs_state->brake_limit = limit;

        if ((ch->feedback > speed_eps) && (ch->output < -limit))
        {
            ch->output = -limit;
            ch->output_prev = ch->output;
        }
        else if ((ch->feedback < -speed_eps) && (ch->output > limit))
        {
            ch->output = limit;
            ch->output_prev = ch->output;
        }
    }
    else
    {
        abs_state->brake_limit = min_limit;
    }

    abs_state->prev_feedback = ch->feedback;
#else
    (void)id;
    (void)dt_s;
    (void)brake_to_stop;
#endif

    return (NULL != ch) ? ch->output : 0.0f;
}

static float pid_apply_drive_direction_guard(pid_ctrl_id_t id, pid_channel_state_t *ch)
{
#if PID_DRIVE_DIRECTION_GUARD_ENABLE
    float speed_eps;
    float low_target_eps;
    float abs_target;
    float abs_feedback;
#if PID_DRIVE_OVERSPEED_COAST_ENABLE
    float coast_start;
    float coast_end;
    float overspeed;
    float coast_scale;
#endif
    uint8 target_opposes_feedback = 0U;
    uint8 low_target_overspeed = 0U;

    if ((NULL == ch) ||
        ((id != PID_CTRL_REAR_RIGHT_SPEED) && (id != PID_CTRL_REAR_LEFT_SPEED)))
    {
        return (NULL != ch) ? ch->output : 0.0f;
    }

    speed_eps = pid_absf(PID_DRIVE_DIRECTION_GUARD_FEEDBACK_ECOD);
    low_target_eps = pid_absf(PID_DRIVE_DIRECTION_GUARD_LOW_TARGET_ECOD);
    abs_target = pid_absf(ch->target);
    abs_feedback = pid_absf(ch->feedback);

    if ((abs_feedback <= speed_eps) || ((ch->output * ch->feedback) <= 0.0f))
    {
        return ch->output;
    }

    if (((ch->target > speed_eps) && (ch->feedback < -speed_eps)) ||
        ((ch->target < -speed_eps) && (ch->feedback > speed_eps)))
    {
        target_opposes_feedback = 1U;
    }

    if ((abs_target <= low_target_eps) &&
        (abs_feedback > (abs_target + speed_eps)))
    {
        low_target_overspeed = 1U;
    }

    if ((0U != target_opposes_feedback) || (0U != low_target_overspeed))
    {
        ch->output = 0.0f;
        ch->output_prev = 0.0f;
        ch->integral = 0.0f;
        ch->i_term = 0.0f;
        ch->accel_boost_ticks = 0U;
        ch->direction_guard_active = 1U;
        pid_drive_abs_reset(id);
    }
#if PID_DRIVE_OVERSPEED_COAST_ENABLE
    else if (((ch->target * ch->feedback) > 0.0f) &&
             (abs_feedback > abs_target))
    {
        coast_start = PID_DRIVE_OVERSPEED_COAST_START_RATIO * abs_target;
        coast_end = PID_DRIVE_OVERSPEED_COAST_END_RATIO * abs_target;
        if (coast_start < PID_DRIVE_OVERSPEED_COAST_START_ECOD)
        {
            coast_start = PID_DRIVE_OVERSPEED_COAST_START_ECOD;
        }
        if (coast_end < PID_DRIVE_OVERSPEED_COAST_END_ECOD)
        {
            coast_end = PID_DRIVE_OVERSPEED_COAST_END_ECOD;
        }
        if (coast_end <= coast_start)
        {
            coast_end = coast_start + 1.0f;
        }

        overspeed = abs_feedback - abs_target;
        if (overspeed > coast_start)
        {
            coast_scale = 1.0f - (overspeed - coast_start) /
                          (coast_end - coast_start);
            coast_scale = pid_clipf(coast_scale, 0.0f, 1.0f);
            ch->output *= coast_scale;
            /* Coast is an actuator-side limiter. Keeping the controller's
               previous output avoids compounding the reduction every tick. */
            ch->accel_boost_ticks = 0U;
        }
    }
#endif
#else
    (void)id;
#endif

    return (NULL != ch) ? ch->output : 0.0f;
}

static float pid_calc_position(pid_channel_state_t *ch, float target, float feedback, float dt_s, uint8 cyclic_error)
{
    float err;
    float p_term;
    float i_term;
    float d_raw;
    float kp_eff;
    float kd_eff;
    float alpha;
    float out_limit;
    float i_limit;
    uint8 hold_integral = 0U;

    if (NULL == ch)
    {
        return 0.0f;
    }

    if (dt_s <= 1e-6f)
    {
        dt_s = PID_CTRL_DT_DEFAULT_S;
    }

    err = target - feedback;
    if (0U != cyclic_error)
    {
        err = pid_wrap_error_cyclic(err, PID_STEER_CYCLE_DEG);
    }

    i_limit = pid_safe_limit(ch->i_limit);
    out_limit = pid_safe_limit(ch->output_limit);

    if (((ch->output >= out_limit) && (err > 0.0f)) ||
        ((ch->output <= -out_limit) && (err < 0.0f)))
    {
        hold_integral = 1U;
    }

    if (0U == hold_integral)
    {
        ch->integral += err * dt_s;
        ch->integral = pid_clipf(ch->integral, -i_limit, i_limit);
    }

    if (0U == ch->feedback_valid)
    {
        d_raw = 0.0f;
        ch->derivative_filt = 0.0f;
        ch->feedback_valid = 1U;
    }
    else
    {
        d_raw = -(feedback - ch->feedback) / dt_s;
    }
    alpha = pid_clipf(ch->d_filter_alpha, 0.0f, 1.0f);
    ch->derivative_filt += alpha * (d_raw - ch->derivative_filt);

    kp_eff = ch->kp;
    kd_eff = ch->kd;
    pid_apply_steer_fuzzy_gain(err, d_raw, &kp_eff, &kd_eff);

    p_term = kp_eff * err;
    i_term = ch->ki * ch->integral;

    ch->derivative = d_raw;
    ch->p_term = p_term;
    ch->i_term = i_term;
    ch->d_term = kd_eff * ch->derivative_filt;
    ch->output = ch->p_term + ch->i_term + ch->d_term;
    ch->output = pid_clipf(ch->output, -out_limit, out_limit);

    ch->target = target;
    ch->feedback = feedback;
    ch->error = err;
    ch->error_prev = err;
    ch->error_prev2 = err;
    ch->output_prev = ch->output;

    return ch->output;
}

static float pid_calc_incremental(pid_channel_state_t *ch,
                                  float target,
                                  float feedback,
                                  float dt_s,
                                  uint8 zero_target_brake)
{
    float err;
    float err_prev;
    float err_prev2;
    float delta_p;
    float delta_i;
    float delta_d;
    float output;
    float out_limit;
    float i_limit;
    float brake_request;
    float kp_eff;
    float ki_eff;

    if (NULL == ch)
    {
        return 0.0f;
    }

    if (dt_s <= 1e-6f)
    {
        dt_s = PID_CTRL_DT_DEFAULT_S;
    }

    ch->direction_guard_active = 0U;

    if (pid_absf(target) < 1e-6f)
    {
        if ((0U != zero_target_brake) &&
            ((feedback > PID_DRIVE_ABS_SPEED_EPS_ECOD) ||
             (feedback < -PID_DRIVE_ABS_SPEED_EPS_ECOD)))
        {
            out_limit = pid_safe_limit(ch->output_limit);
            brake_request =
                (zero_target_brake == PID_DRIVE_BRAKE_TO_STOP_FORCE) ?
                pid_absf(PID_DRIVE_ABS_STRONG_MAX_BRAKE_PWM) :
                pid_absf(PID_DRIVE_ABS_MAX_BRAKE_PWM);
            if (brake_request > out_limit)
            {
                brake_request = out_limit;
            }
            err = -feedback;
            ch->target = target;
            ch->feedback = feedback;
            ch->error = err;
            ch->error_prev = err;
            ch->error_prev2 = err;
            ch->integral = 0.0f;
            ch->derivative = 0.0f;
            ch->derivative_filt = 0.0f;
            ch->p_term = 0.0f;
            ch->i_term = 0.0f;
            ch->d_term = 0.0f;
            ch->output = (feedback > 0.0f) ? -brake_request : brake_request;
            ch->output_prev = ch->output;
            return ch->output;
        }

        ch->target = target;
        ch->feedback = feedback;
        ch->error = 0.0f;
        ch->error_prev = 0.0f;
        ch->error_prev2 = 0.0f;
        ch->integral = 0.0f;
        ch->derivative = 0.0f;
        ch->derivative_filt = 0.0f;
        ch->p_term = 0.0f;
        ch->i_term = 0.0f;
        ch->d_term = 0.0f;
        ch->output = 0.0f;
        ch->output_prev = 0.0f;
        return 0.0f;
    }

    err = target - feedback;
    err_prev = ch->error_prev;
    err_prev2 = ch->error_prev2;
    i_limit = pid_safe_limit(ch->i_limit);
    out_limit = pid_safe_limit(ch->output_limit);

    ch->integral += err * dt_s;
    ch->integral = pid_clipf(ch->integral, -i_limit, i_limit);

    kp_eff = ch->kp;
    ki_eff = ch->ki;
    pid_apply_drive_fuzzy_gain(err, &kp_eff, &ki_eff);
    delta_p = kp_eff * (err - err_prev);
    delta_i = ki_eff * err * dt_s;
    delta_d = ch->kd * ((err - (2.0f * err_prev) + err_prev2) / dt_s);
    output = ch->output_prev + delta_p + delta_i + delta_d;
    output = pid_clipf(output, -out_limit, out_limit);

    ch->target = target;
    ch->feedback = feedback;
    ch->error = err;
    ch->error_prev2 = err_prev;
    ch->error_prev = err;
    ch->p_term = delta_p;
    ch->i_term = delta_i;
    ch->d_term = delta_d;
    ch->derivative = (err - err_prev) / dt_s;
    ch->derivative_filt = ch->derivative;
    ch->output = output;
    ch->output_prev = output;

    return output;
}

void pid_init(void)
{
    uint8 i;

    memset(&g_pid, 0, sizeof(g_pid));
    pid_steer_breakaway_reset();

    for (i = 0U; i < (uint8)PID_CTRL_MAX; i++)
    {
        g_pid.ch[i].d_filter_alpha = PID_D_FILTER_ALPHA_DEFAULT;
        g_pid.ch[i].output_limit = PID_OUTPUT_LIMIT_DEFAULT;
        pid_drive_abs_reset((pid_ctrl_id_t)i);
    }

    g_pid.ch[PID_CTRL_REAR_RIGHT_SPEED].kp = PID_DRIVE_KP_DEFAULT;
    g_pid.ch[PID_CTRL_REAR_RIGHT_SPEED].ki = PID_DRIVE_KI_DEFAULT;
    g_pid.ch[PID_CTRL_REAR_RIGHT_SPEED].kd = PID_DRIVE_KD_DEFAULT;
    g_pid.ch[PID_CTRL_REAR_RIGHT_SPEED].i_limit = PID_DRIVE_I_LIMIT_DEFAULT;

    g_pid.ch[PID_CTRL_REAR_LEFT_SPEED].kp = PID_DRIVE_KP_DEFAULT;
    g_pid.ch[PID_CTRL_REAR_LEFT_SPEED].ki = PID_DRIVE_KI_DEFAULT;
    g_pid.ch[PID_CTRL_REAR_LEFT_SPEED].kd = PID_DRIVE_KD_DEFAULT;
    g_pid.ch[PID_CTRL_REAR_LEFT_SPEED].i_limit = PID_DRIVE_I_LIMIT_DEFAULT;

    g_pid.ch[PID_CTRL_FRONT_STEER_ANGLE].kp = PID_STEER_KP_DEFAULT;
    g_pid.ch[PID_CTRL_FRONT_STEER_ANGLE].ki = PID_STEER_KI_DEFAULT;
    g_pid.ch[PID_CTRL_FRONT_STEER_ANGLE].kd = PID_STEER_KD_DEFAULT;
    g_pid.ch[PID_CTRL_FRONT_STEER_ANGLE].i_limit = PID_STEER_I_LIMIT_DEFAULT;
    g_pid.ch[PID_CTRL_FRONT_STEER_ANGLE].output_limit = PID_STEER_OUT_LIMIT_DEFAULT;

    g_pid.initialized = 1U;
}

void pid_reset(pid_ctrl_id_t id)
{
    pid_channel_state_t *ch;

    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return;
    }

    ch = &g_pid.ch[id];
    ch->target = 0.0f;
    ch->feedback = 0.0f;
    ch->error = 0.0f;
    ch->error_prev = 0.0f;
    ch->error_prev2 = 0.0f;
    ch->integral = 0.0f;
    ch->derivative = 0.0f;
    ch->derivative_filt = 0.0f;
    ch->p_term = 0.0f;
    ch->i_term = 0.0f;
    ch->d_term = 0.0f;
    ch->output = 0.0f;
    ch->output_prev = 0.0f;
    ch->feedback_valid = 0U;
    ch->accel_boost_ticks = 0U;
    ch->direction_guard_active = 0U;
    pid_drive_abs_reset(id);

    if (id == PID_CTRL_REAR_RIGHT_SPEED)
    {
        g_pid_drive_duty_r = 0;
    }
    else if (id == PID_CTRL_REAR_LEFT_SPEED)
    {
        g_pid_drive_duty_l = 0;
    }
    else if (id == PID_CTRL_FRONT_STEER_ANGLE)
    {
        pid_steer_breakaway_reset();
    }
}

void pid_reset_all(void)
{
    uint8 i;

    if (0U == g_pid.initialized)
    {
        return;
    }

    for (i = 0U; i < (uint8)PID_CTRL_MAX; i++)
    {
        pid_reset((pid_ctrl_id_t)i);
    }
}

void pid_set_gain(pid_ctrl_id_t id, float kp, float ki, float kd)
{
    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return;
    }

    g_pid.ch[id].kp = kp;
    g_pid.ch[id].ki = ki;
    g_pid.ch[id].kd = kd;
}

void pid_set_limit(pid_ctrl_id_t id, float i_limit, float out_limit)
{
    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return;
    }

    g_pid.ch[id].i_limit = pid_safe_limit(i_limit);
    g_pid.ch[id].output_limit = pid_safe_limit(out_limit);
}

void pid_set_d_filter(pid_ctrl_id_t id, float alpha)
{
    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return;
    }

    g_pid.ch[id].d_filter_alpha = pid_clipf(alpha, 0.0f, 1.0f);
}

void pid_set_target(pid_ctrl_id_t id, float target)
{
    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return;
    }

    g_pid.ch[id].target = target;
}

void pid_set_feedback(pid_ctrl_id_t id, float feedback)
{
    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return;
    }

    g_pid.ch[id].feedback = feedback;
}

float pid_calc(pid_ctrl_id_t id, float target, float feedback, float dt_s)
{
    uint8 cyclic_error = 0U;

    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return 0.0f;
    }

    if (id == PID_CTRL_FRONT_STEER_ANGLE)
    {
#if PID_STEER_USE_CYCLIC_ERROR
        cyclic_error = 1U;
#else
        cyclic_error = 0U;
#endif
        (void)pid_calc_position(&g_pid.ch[id], target, feedback, dt_s, cyclic_error);
        return pid_apply_steer_breakaway_comp(&g_pid.ch[id]);
    }

    (void)pid_calc_incremental(&g_pid.ch[id], target, feedback, dt_s, 0U);
    (void)pid_apply_drive_accel_assist(id, &g_pid.ch[id]);
    (void)pid_apply_drive_abs_brake_limit(id,
                                          &g_pid.ch[id],
                                          dt_s,
                                          PID_DRIVE_BRAKE_TO_STOP_NONE);
    return pid_apply_drive_direction_guard(id, &g_pid.ch[id]);
}

float pid_calc_drive(pid_ctrl_id_t id, float target, float feedback, float dt_s, uint8 brake_to_stop)
{
    float output;

    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return 0.0f;
    }

    if ((id != PID_CTRL_REAR_RIGHT_SPEED) && (id != PID_CTRL_REAR_LEFT_SPEED))
    {
        return pid_calc(id, target, feedback, dt_s);
    }

    (void)pid_calc_incremental(&g_pid.ch[id], target, feedback, dt_s, brake_to_stop);
    (void)pid_apply_drive_accel_assist(id, &g_pid.ch[id]);
    (void)pid_apply_drive_abs_brake_limit(id,
                                          &g_pid.ch[id],
                                          dt_s,
                                          brake_to_stop);
    output = pid_apply_drive_direction_guard(id, &g_pid.ch[id]);
    if ((g_ctrl_drive_forward_only != 0U) &&
        (output < 0.0f) &&
        (feedback <= PID_DRIVE_ABS_SPEED_EPS_ECOD))
    {
        g_pid.ch[id].output = 0.0f;
        g_pid.ch[id].output_prev = 0.0f;
        g_pid.ch[id].integral = 0.0f;
        g_pid.ch[id].i_term = 0.0f;
        g_pid.ch[id].accel_boost_ticks = 0U;
        pid_drive_abs_reset(id);
        output = 0.0f;
    }
    return output;
}

void pid_calc_all(float rear_r_target_speed,
                  float rear_r_feedback_speed,
                  float rear_l_target_speed,
                  float rear_l_feedback_speed,
                  float steer_target_angle,
                  float steer_feedback_angle,
                  float dt_s)
{
    (void)pid_calc(PID_CTRL_REAR_RIGHT_SPEED, rear_r_target_speed, rear_r_feedback_speed, dt_s);
    (void)pid_calc(PID_CTRL_REAR_LEFT_SPEED, rear_l_target_speed, rear_l_feedback_speed, dt_s);
    (void)pid_calc(PID_CTRL_FRONT_STEER_ANGLE, steer_target_angle, steer_feedback_angle, dt_s);
}

float pid_get_output(pid_ctrl_id_t id)
{
    if ((0U == g_pid.initialized) || (0U == pid_id_valid(id)))
    {
        return 0.0f;
    }

    return g_pid.ch[id].output;
}

void pid_get_status(pid_status_t *status)
{
    if (NULL == status)
    {
        return;
    }

    *status = g_pid;
}

void pid_apply_to_dianji(void)
{
    int16 duty_front;

    if (0U == g_pid.initialized)
    {
        return;
    }

    g_pid_drive_duty_r = pid_clip_i32_to_i16((int32)pid_clipf(g_pid.ch[PID_CTRL_REAR_RIGHT_SPEED].output,
                                                              -(float)DIANJI_DUTY_LIMIT,
                                                              (float)DIANJI_DUTY_LIMIT));
    g_pid_drive_duty_l = pid_clip_i32_to_i16((int32)pid_clipf(g_pid.ch[PID_CTRL_REAR_LEFT_SPEED].output,
                                                              -(float)DIANJI_DUTY_LIMIT,
                                                              (float)DIANJI_DUTY_LIMIT));
    duty_front = pid_clip_i32_to_i16((int32)(g_pid.ch[PID_CTRL_FRONT_STEER_ANGLE].output * PID_STEER_OUTPUT_SIGN));

    dianji_set_motor_duty(PID_KART_MOTOR_REAR_RIGHT, g_pid_drive_duty_r);
    dianji_set_motor_duty(PID_KART_MOTOR_REAR_LEFT, g_pid_drive_duty_l);
    dianji_set_motor_duty(PID_KART_MOTOR_FRONT_STEER, duty_front);
}
