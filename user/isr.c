#include "isr_config.h"
#include "isr.h"
#include "../code/IMU/dici.h"
#include "pid.h"
#include "pid_config.h"
#include "dianji.h"
#include <math.h>
#include "../code/yaokong/yaokong.h"
#include "../code/lora/lora_atk_mw1278d.h"
#include "tld7002_project_config.h"
#if TLD7002_PROJECT_ENABLE
#include "zf_device_tld7002.h"
#endif
#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
#include "zf_device_dot_matrix_screen.h"
#endif

#define PID_STEER_DT_S          (0.001f)
#define PID_SPEED_PERIOD_TICKS  (5U)
#define PID_SPEED_DT_S          (0.005f)
#define CTRL_MAIN_LOOP_STALL_MS  (100U)
#define CTRL_CMD_STALE_MS        (200U)
#define CTRL_TASK_CMD_STALE_MS   (500U)
#define CTRL_STOP_REASON_DRIVE_STALL_LOCAL (7U)
#define CTRL_PID_LOW_TARGET_ECOD (500.0f)
#define CTRL_PID_LOW_OUTPUT_DUTY (500.0f)
#define CTRL_PARALLEL_STEER_DIFF_ASSIST_START_DEG (20.0f)
#define CTRL_PARALLEL_STEER_DIFF_ASSIST_FULL_DEG (25.0f)
#define CTRL_PARALLEL_STEER_DIFF_ASSIST_MAX_GAIN (1.12f)

extern volatile int16 g_ecod1_speed;
extern volatile int16 g_ecod2_speed;
extern volatile int16 g_ecod1_speed_filt;
extern volatile int16 g_ecod2_speed_filt;
extern volatile int16 g_steer_delta_counts;

volatile float g_pid_fixed_target_speed_mps = 0.35f;
volatile float g_pid_pedal_target_speed_mps = 0.0f;
volatile float g_pid_used_target_speed_mps = 0.0f;
volatile uint8 g_pid_target_src = TARGET_SRC_TASK;
volatile uint8 g_pid_target_use_pedal = 0U;
volatile float g_steer_fixed_target_angle_deg = 0.0f;
volatile float g_steer_target_angle_deg = 0.0f;
volatile float g_steer_feedback_angle_deg = 0.0f;
volatile float g_steer_feedback_speed_counts_s = 0.0f;
volatile uint8 g_steer_feedback_valid = 0U;
volatile uint32 g_steer_feedback_update_ms = 0U;
volatile uint8 g_steer_sensor_fault = 0U;
volatile uint8 g_steer_stall_fault = 0U;
volatile uint32 g_steer_stall_fault_time_ms = 0U;
volatile int16 g_steer_raw_read = 0;
volatile int16 g_steer_raw_last_valid = 0;
volatile int16 g_steer_raw_jump_counts = 0;
volatile uint8 g_steer_raw_last_reject_reason = CTRL_STEER_RAW_REJECT_NONE;
volatile uint8 g_steer_raw_last_reject_streak = 0U;
volatile uint32 g_steer_raw_reject_count = 0U;
volatile uint32 g_steer_raw_last_reject_ms = 0U;
volatile int16 g_steer_raw_last_reject_raw = 0;
volatile int16 g_steer_raw_last_reject_ref_raw = 0;
volatile int16 g_steer_raw_last_reject_jump_counts = 0;
volatile uint32 g_ctrl_main_loop_heartbeat_ms = 0U;
volatile uint32 g_ctrl_cpu0_heartbeat_ms = 0U;
volatile uint32 g_ctrl_cpu0_age_ms = 0U;
volatile uint8 g_ctrl_cpu0_stage = CTRL_CPU0_STAGE_BOOT;
volatile uint8 g_ctrl_cpu0_stall_stage = CTRL_CPU0_STAGE_BOOT;
volatile uint32 g_ctrl_cpu0_stall_count = 0U;
volatile uint32 g_ctrl_cpu0_stall_max_age_ms = 0U;
volatile uint32 g_ctrl_safety_pause_until_ms = 0U;
volatile uint8 g_ctrl_stop_reason = CTRL_STOP_REASON_NONE;
volatile uint8 g_ctrl_safety_pause_active = 0U;
volatile uint32 g_ctrl_main_loop_age_ms = 0U;
volatile uint32 g_ctrl_cmd_publish_ms = 0U;
volatile uint32 g_ctrl_cmd_age_ms = 0U;
volatile uint8 g_ctrl_pid_reset_reason = CTRL_PID_RESET_REASON_NONE;
volatile uint32 g_ctrl_pid_reset_time_ms = 0U;
volatile uint8 g_ctrl_pid_low_reason = CTRL_PID_LOW_REASON_NONE;
volatile uint32 g_ctrl_pid_low_time_ms = 0U;
volatile float g_ctrl_pid_low_target_r = 0.0f;
volatile float g_ctrl_pid_low_feedback_r = 0.0f;
volatile float g_ctrl_pid_low_output_r = 0.0f;
volatile float g_ctrl_pid_low_target_l = 0.0f;
volatile float g_ctrl_pid_low_feedback_l = 0.0f;
volatile float g_ctrl_pid_low_output_l = 0.0f;
volatile uint32 g_imu_pit_count = 0U;
volatile uint32 g_imu_update_count = 0U;
volatile uint32 g_imu_update_ms = 0U;
volatile uint32 g_imu_read_fail_count = 0U;
volatile ctrl_cmd_t g_ctrl_cmd = {1U, 1U, 0U, 0.0f, 0.0f, 0, 0U, 0U};
volatile uint32 g_ctrl_cmd_seq = 0U;
static volatile ctrl_cmd_t g_ctrl_cmd_buf[2] = {{1U, 1U, 0U, 0.0f, 0.0f, 0, 0U, 0U}, {1U, 1U, 0U, 0.0f, 0.0f, 0, 0U, 0U}};
static volatile uint8 g_ctrl_cmd_active_idx = 0U;
static float g_drive_target_r_pid_slew = 0.0f;
static float g_drive_target_l_pid_slew = 0.0f;

static uint8 ctrl_time_before_ms(uint32 now_ms, uint32 deadline_ms)
{
    return (((int32)(deadline_ms - now_ms)) > 0) ? 1U : 0U;
}

static uint8 ctrl_safety_pause_active_at(uint32 now_ms)
{
    if (g_ctrl_safety_pause_until_ms == 0U)
    {
        return 0U;
    }

    if (0U != ctrl_time_before_ms(now_ms, g_ctrl_safety_pause_until_ms))
    {
        return 1U;
    }

    g_ctrl_safety_pause_until_ms = 0U;
    return 0U;
}

static uint32 ctrl_main_loop_age_at(uint32 now_ms)
{
    uint32 heartbeat_ms = g_ctrl_main_loop_heartbeat_ms;

    if (heartbeat_ms == 0U)
    {
        return 0U;
    }

    return now_ms - heartbeat_ms;
}

static uint8 ctrl_main_loop_stalled_at(uint32 now_ms)
{
    return (ctrl_main_loop_age_at(now_ms) > CTRL_MAIN_LOOP_STALL_MS) ? 1U : 0U;
}

static uint32 ctrl_cpu0_age_at(uint32 now_ms)
{
    uint32 heartbeat_ms = g_ctrl_cpu0_heartbeat_ms;

    if (heartbeat_ms == 0U)
    {
        return 0U;
    }

    return now_ms - heartbeat_ms;
}

static uint32 ctrl_cmd_age_at(uint32 now_ms)
{
    uint32 publish_ms = g_ctrl_cmd_publish_ms;

    if (publish_ms == 0U)
    {
        return 0U;
    }

    if (now_ms < publish_ms)
    {
        return 0U;
    }

    return now_ms - publish_ms;
}

static uint8 ctrl_cmd_stale_at(uint32 now_ms)
{
    uint32 age_ms = ctrl_cmd_age_at(now_ms);
    uint32 stale_ms = (g_pid_target_src == TARGET_SRC_TASK) ?
                      CTRL_TASK_CMD_STALE_MS :
                      CTRL_CMD_STALE_MS;

    return ((age_ms != 0U) && (age_ms > stale_ms)) ? 1U : 0U;
}

static float ctrl_slew_f32(float current, float target, float max_step)
{
    if (max_step <= 0.0f)
    {
        return target;
    }

    if (target > (current + max_step))
    {
        return current + max_step;
    }
    if (target < (current - max_step))
    {
        return current - max_step;
    }

    return target;
}

static void ctrl_drive_target_slew_reset(void)
{
    g_drive_target_r_pid_slew = 0.0f;
    g_drive_target_l_pid_slew = 0.0f;
}

static float ctrl_diag_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void ctrl_pid_diag_mark_reset(uint8 reason, uint32 now_ms)
{
    g_ctrl_pid_reset_reason = reason;
    g_ctrl_pid_reset_time_ms = now_ms;
}

static void ctrl_pid_diag_mark_low(uint8 reason,
                                   uint32 now_ms,
                                   float target_r,
                                   float feedback_r,
                                   float output_r,
                                   float target_l,
                                   float feedback_l,
                                   float output_l)
{
    g_ctrl_pid_low_reason = reason;
    g_ctrl_pid_low_time_ms = now_ms;
    g_ctrl_pid_low_target_r = target_r;
    g_ctrl_pid_low_feedback_r = feedback_r;
    g_ctrl_pid_low_output_r = output_r;
    g_ctrl_pid_low_target_l = target_l;
    g_ctrl_pid_low_feedback_l = feedback_l;
    g_ctrl_pid_low_output_l = output_l;
}

static void ctrl_stop_all_outputs(void)
{
    g_pid_used_target_speed_mps = 0.0f;
    g_steer_target_angle_deg = 0.0f;
    ctrl_drive_target_slew_reset();
    pid_reset_all();
    dianji_set_motor_duty(DIANJI_MOTOR_M, 0);
    dianji_set_motor_duty(DIANJI_MOTOR_R, 0);
    dianji_set_motor_duty(DIANJI_MOTOR_L, 0);
}

void ctrl_main_loop_heartbeat(void)
{
    g_ctrl_main_loop_heartbeat_ms = system_getval_ms();
}

void ctrl_cpu0_progress(uint8 stage)
{
    g_ctrl_cpu0_stage = stage;
    g_ctrl_cpu0_heartbeat_ms = system_getval_ms();
}

void ctrl_safety_pause_ms(uint32 pause_ms)
{
    uint32 now_ms;

    if (pause_ms == 0U)
    {
        g_ctrl_safety_pause_until_ms = 0U;
        return;
    }

    now_ms = system_getval_ms();
    g_ctrl_safety_pause_until_ms = now_ms + pause_ms;
    g_ctrl_stop_reason = CTRL_STOP_REASON_SAFETY_PAUSE;
    g_ctrl_safety_pause_active = 1U;
    g_ctrl_main_loop_age_ms = ctrl_main_loop_age_at(now_ms);
    ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_SAFETY_PAUSE, now_ms);
    ctrl_stop_all_outputs();
}

uint8 ctrl_safety_pause_is_active(void)
{
    return ctrl_safety_pause_active_at(system_getval_ms());
}

void ctrl_cmd_publish(const ctrl_cmd_t *cmd)
{
    uint8 next_idx;

    if (NULL == cmd)
    {
        return;
    }

    g_ctrl_cmd_seq++;
    next_idx = (uint8)(g_ctrl_cmd_active_idx ^ 1U);
    g_ctrl_cmd_buf[next_idx] = *cmd;
    g_ctrl_cmd_active_idx = next_idx;
    g_ctrl_cmd = *cmd;
    g_ctrl_cmd_publish_ms = system_getval_ms();
    g_ctrl_cmd_seq++;
}

uint8 ctrl_cmd_get_snapshot(ctrl_cmd_t *cmd)
{
    uint32 seq_1;
    uint32 seq_2;
    uint8 idx;

    if (NULL == cmd)
    {
        return 0U;
    }

    seq_1 = g_ctrl_cmd_seq;
    idx = g_ctrl_cmd_active_idx;
    *cmd = g_ctrl_cmd_buf[idx];
    seq_2 = g_ctrl_cmd_seq;

    if ((seq_1 != seq_2) || ((seq_1 & 1U) != 0U))
    {
        return 0U;
    }

    return 1U;
}

IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    static uint8 speed_tick = 0U;
    static ctrl_cmd_t last_cmd = {1U, 1U, 0U, 0.0f, 0.0f, 0, 0U, 0U};
    static uint16 steer_stall_ticks = 0U;
    static uint16 steer_sensor_lost_ticks = 0U;
    static uint32 drive_stall_start_ms = 0U;
    static uint8 main_loop_stall_latched = 0U;
    float target_speed_mps;
    float target_speed_pid;
    float target_speed_r_pid;
    float target_speed_l_pid;
    float feedback_r_pid;
    float feedback_l_pid;
    float drive_pid_output_r;
    float drive_pid_output_l;
    float steer_target_angle_deg = 0.0f;
    float steer_feedback_angle_deg = 0.0f;
    float steer_pid_output = 0.0f;
    float steer_feedback_speed_counts_s = 0.0f;
    ctrl_cmd_t cmd;
    uint8 steer_closed_loop = 0U;
    uint32 now_ms;
    uint8 steer_feedback_fresh = 0U;
    uint8 safety_pause_active = 0U;
    uint8 main_loop_stalled = 0U;
    uint8 cmd_stale = 0U;
    uint32 steer_feedback_timeout_ms = PID_STEER_FEEDBACK_TIMEOUT_MS;
    uint16 steer_sensor_fault_limit_ticks = PID_STEER_SENSOR_FAULT_TIMEOUT_MS;

    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    now_ms = system_getval_ms();
    g_imu_pit_count++;

    if (imu_get_values())
    {
        g_imu_update_count++;
        g_imu_update_ms = now_ms;
        EKF_UpData();
    }
    else
    {
        g_imu_read_fail_count++;
    }
    if (0U != ctrl_cmd_get_snapshot(&cmd))
    {
        last_cmd = cmd;
    }
    else
    {
        cmd = last_cmd;
    }

#if PID_STEER_SENSOR_PROTECT_ENABLE
    if (ctrl_diag_absf(cmd.target_speed_mps) <= PID_STEER_FEEDBACK_TIMEOUT_LOW_SPEED_MPS)
    {
        steer_feedback_timeout_ms = PID_STEER_FEEDBACK_TIMEOUT_LOW_SPEED_MS;
        steer_sensor_fault_limit_ticks = (uint16)PID_STEER_FEEDBACK_TIMEOUT_LOW_SPEED_MS;
    }
#endif

    steer_feedback_fresh = ((0U != g_steer_feedback_valid) &&
                            ((now_ms - g_steer_feedback_update_ms) <= steer_feedback_timeout_ms)) ? 1U : 0U;
    if ((0U != g_steer_sensor_fault) && (0U != steer_feedback_fresh))
    {
        g_steer_sensor_fault = 0U;
        steer_sensor_lost_ticks = 0U;
    }

    g_ctrl_main_loop_age_ms = ctrl_main_loop_age_at(now_ms);
    g_ctrl_cmd_age_ms = ctrl_cmd_age_at(now_ms);
    safety_pause_active = ctrl_safety_pause_active_at(now_ms);
    main_loop_stalled = ctrl_main_loop_stalled_at(now_ms);
    cmd_stale = ctrl_cmd_stale_at(now_ms);
    g_ctrl_safety_pause_active = safety_pause_active;
    g_ctrl_cpu0_age_ms = ctrl_cpu0_age_at(now_ms);

    if (g_ctrl_cpu0_age_ms > CTRL_MAIN_LOOP_STALL_MS)
    {
        if (main_loop_stall_latched == 0U)
        {
            main_loop_stall_latched = 1U;
            g_ctrl_cpu0_stall_stage = g_ctrl_cpu0_stage;
            g_ctrl_cpu0_stall_count++;
        }
        if (g_ctrl_cpu0_age_ms > g_ctrl_cpu0_stall_max_age_ms)
        {
            g_ctrl_cpu0_stall_max_age_ms = g_ctrl_cpu0_age_ms;
        }
    }
    else
    {
        main_loop_stall_latched = 0U;
    }

    if (0U != safety_pause_active)
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_SAFETY_PAUSE;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        steer_sensor_lost_ticks = 0U;
        steer_stall_ticks = 0U;
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_SAFETY_PAUSE, now_ms);
        ctrl_stop_all_outputs();
        return;
    }

    if (0U != main_loop_stalled)
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_MAIN_LOOP_STALL;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        steer_sensor_lost_ticks = 0U;
        steer_stall_ticks = 0U;
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_MAIN_LOOP_STALL, now_ms);
        ctrl_stop_all_outputs();
        return;
    }

    if (0U != cmd_stale)
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_CMD_STALE;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        steer_sensor_lost_ticks = 0U;
        steer_stall_ticks = 0U;
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_CMD_STALE, now_ms);
        ctrl_stop_all_outputs();
        return;
    }

    if ((0U != g_steer_sensor_fault) && (0U == cmd.direct_pwm))
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_STEER_SENSOR;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_STEER_SENSOR, now_ms);
        ctrl_stop_all_outputs();
        return;
    }

    if ((0U != g_steer_stall_fault) && (0U == cmd.direct_pwm))
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_STEER_STALL;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_STEER_STALL, now_ms);
        ctrl_stop_all_outputs();
        return;
    }

#if YAOKONG_REMOTE_ENABLE
    if (0U == yaokong_is_motor_run_enabled())
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_REMOTE_MOTOR_OFF;
        g_pid_used_target_speed_mps = 0.0f;
        g_steer_target_angle_deg = 0.0f;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        ctrl_drive_target_slew_reset();
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_REMOTE_MOTOR_OFF, now_ms);
        pid_reset_all();
        dianji_set_motor_duty(DIANJI_MOTOR_M, 0);
        dianji_set_motor_duty(DIANJI_MOTOR_R, 0);
        dianji_set_motor_duty(DIANJI_MOTOR_L, 0);
        return;
    }
#endif

    if (cmd.direct_pwm)
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_NONE;
        g_pid_used_target_speed_mps = 0.0f;
        g_steer_target_angle_deg = 0.0f;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        ctrl_drive_target_slew_reset();
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_DIRECT_PWM, now_ms);
        pid_reset_all();
        if (0U != cmd.steer_pwm_only)
        {
            dianji_set_motor_duty(DIANJI_MOTOR_M, cmd.pwm_duty);
            dianji_set_motor_duty(DIANJI_MOTOR_R, 0);
            dianji_set_motor_duty(DIANJI_MOTOR_L, 0);
        }
        else
        {
            dianji_set_motor_duty(DIANJI_MOTOR_M, cmd.pwm_duty);
            dianji_set_motor_duty(DIANJI_MOTOR_R, cmd.pwm_duty);
            dianji_set_motor_duty(DIANJI_MOTOR_L, cmd.pwm_duty);
        }
        return;
    }

    if (0U == cmd.use_pid)
    {
        g_ctrl_stop_reason = CTRL_STOP_REASON_NONE;
        g_pid_used_target_speed_mps = 0.0f;
        g_steer_target_angle_deg = 0.0f;
        speed_tick = 0U;
        drive_stall_start_ms = 0U;
        ctrl_drive_target_slew_reset();
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_USE_PID_OFF, now_ms);
        pid_reset_all();
        pid_apply_to_dianji();
        return;
    }

    g_ctrl_stop_reason = CTRL_STOP_REASON_NONE;
    target_speed_mps = cmd.target_speed_mps;
    steer_target_angle_deg = cmd.target_angle_deg;
    steer_closed_loop = cmd.steer_closed_loop;

    if (steer_target_angle_deg > PID_STEER_TARGET_LIMIT_DEG)
    {
        steer_target_angle_deg = PID_STEER_TARGET_LIMIT_DEG;
    }
    else if (steer_target_angle_deg < -PID_STEER_TARGET_LIMIT_DEG)
    {
        steer_target_angle_deg = -PID_STEER_TARGET_LIMIT_DEG;
    }

    g_pid_used_target_speed_mps = target_speed_mps;
    g_steer_target_angle_deg = steer_target_angle_deg;

#if PID_STEER_SENSOR_PROTECT_ENABLE
    if ((0U != steer_closed_loop) && (0U == steer_feedback_fresh))
    {
        if (steer_sensor_lost_ticks < 65535U)
        {
            steer_sensor_lost_ticks++;
        }
        if (steer_sensor_lost_ticks >= steer_sensor_fault_limit_ticks)
        {
            g_steer_sensor_fault = 1U;
            g_ctrl_stop_reason = CTRL_STOP_REASON_STEER_SENSOR;
            speed_tick = 0U;
            drive_stall_start_ms = 0U;
            ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_STEER_SENSOR, now_ms);
            ctrl_stop_all_outputs();
            return;
        }
    }
    else
    {
        steer_sensor_lost_ticks = 0U;
    }

    if ((0U != steer_closed_loop) && (0U != steer_feedback_fresh))
    {
        steer_feedback_angle_deg = g_steer_feedback_angle_deg;
    }
    else
    {
        steer_target_angle_deg = 0.0f;
        steer_feedback_angle_deg = 0.0f;
        g_steer_target_angle_deg = 0.0f;
        ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_STEER_FEEDBACK_STALE, now_ms);
        pid_reset(PID_CTRL_FRONT_STEER_ANGLE);
    }
#else
    steer_feedback_angle_deg = g_steer_feedback_angle_deg;
#endif

    (void)pid_calc(PID_CTRL_FRONT_STEER_ANGLE,
                   steer_target_angle_deg,
                   steer_feedback_angle_deg,
                   PID_STEER_DT_S);
    steer_pid_output = pid_get_output(PID_CTRL_FRONT_STEER_ANGLE);

#if PID_STEER_STALL_PROTECT_ENABLE
    if ((0U != steer_closed_loop) && (0U != steer_feedback_fresh) &&
        (steer_pid_output >= PID_STEER_STALL_DUTY_THRESHOLD ||
         steer_pid_output <= -PID_STEER_STALL_DUTY_THRESHOLD))
    {
        steer_feedback_speed_counts_s = g_steer_feedback_speed_counts_s;
        if ((steer_feedback_speed_counts_s <= PID_STEER_STALL_SPEED_COUNTS_S) &&
            (steer_feedback_speed_counts_s >= -PID_STEER_STALL_SPEED_COUNTS_S))
        {
            if (steer_stall_ticks < 65535U)
            {
                steer_stall_ticks++;
            }
        }
        else
        {
            steer_stall_ticks = 0U;
        }

        if (steer_stall_ticks >= PID_STEER_STALL_TIME_MS)
        {
            g_steer_stall_fault = 1U;
            g_steer_stall_fault_time_ms = now_ms;
            g_ctrl_stop_reason = CTRL_STOP_REASON_STEER_STALL;
            speed_tick = 0U;
            drive_stall_start_ms = 0U;
            ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_STEER_STALL, now_ms);
            ctrl_stop_all_outputs();
            return;
        }
    }
    else
    {
        steer_stall_ticks = 0U;
    }

#endif
    speed_tick++;
    if (speed_tick >= PID_SPEED_PERIOD_TICKS)
    {
        speed_tick = 0U;
        target_speed_pid = target_speed_mps * PID_MPS_TO_ECOD_SPEED;
        target_speed_r_pid = target_speed_pid;
        target_speed_l_pid = target_speed_pid;

#if PID_ACKERMANN_ENABLE
        if ((steer_closed_loop != 0U) &&
            (target_speed_pid > 0.0f || target_speed_pid < 0.0f) &&
            (steer_feedback_angle_deg > 0.1f || steer_feedback_angle_deg < -0.1f))
        {
            float steer_rad = steer_feedback_angle_deg * PID_ACKERMANN_STEER_SIGN * 0.01745329252f;
            float tan_steer = tanf(steer_rad);

            if (tan_steer > 0.0001f || tan_steer < -0.0001f)
            {
                float turn_radius_m = PID_ACKERMANN_WHEELBASE_M / tan_steer;
                float abs_radius_m = (turn_radius_m >= 0.0f) ? turn_radius_m : -turn_radius_m;

                if (abs_radius_m > (PID_ACKERMANN_REAR_TRACK_M * 0.5f))
                {
                    float ratio_delta = (PID_ACKERMANN_REAR_TRACK_M * 0.5f) / abs_radius_m;
                    float steer_abs_deg = ctrl_diag_absf(steer_feedback_angle_deg);

                    if (steer_abs_deg > CTRL_PARALLEL_STEER_DIFF_ASSIST_START_DEG)
                    {
                        float assist_blend =
                            (steer_abs_deg - CTRL_PARALLEL_STEER_DIFF_ASSIST_START_DEG) /
                            (CTRL_PARALLEL_STEER_DIFF_ASSIST_FULL_DEG -
                             CTRL_PARALLEL_STEER_DIFF_ASSIST_START_DEG);
                        float assist_gain;

                        if (assist_blend > 1.0f)
                        {
                            assist_blend = 1.0f;
                        }
                        else if (assist_blend < 0.0f)
                        {
                            assist_blend = 0.0f;
                        }

                        assist_gain = 1.0f +
                                      (CTRL_PARALLEL_STEER_DIFF_ASSIST_MAX_GAIN - 1.0f) *
                                      assist_blend;
                        ratio_delta *= assist_gain;
                    }

                    if (ratio_delta > PID_ACKERMANN_MAX_RATIO_DELTA)
                    {
                        ratio_delta = PID_ACKERMANN_MAX_RATIO_DELTA;
                    }

                    if (turn_radius_m > 0.0f)
                    {
                        target_speed_l_pid = target_speed_pid * (1.0f - ratio_delta);
                        target_speed_r_pid = target_speed_pid * (1.0f + ratio_delta);
                    }
                    else
                    {
                        target_speed_l_pid = target_speed_pid * (1.0f + ratio_delta);
                        target_speed_r_pid = target_speed_pid * (1.0f - ratio_delta);
                    }
                }
            }
        }
#endif
        if ((target_speed_pid > -1e-6f) && (target_speed_pid < 1e-6f))
        {
            ctrl_drive_target_slew_reset();
            ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_ZERO_TARGET, now_ms);
            target_speed_r_pid = 0.0f;
            target_speed_l_pid = 0.0f;
        }
        else
        {
            g_drive_target_r_pid_slew = ctrl_slew_f32(g_drive_target_r_pid_slew,
                                                      target_speed_r_pid,
                                                      PID_DRIVE_TARGET_SLEW_ECOD);
            g_drive_target_l_pid_slew = ctrl_slew_f32(g_drive_target_l_pid_slew,
                                                      target_speed_l_pid,
                                                      PID_DRIVE_TARGET_SLEW_ECOD);
            target_speed_r_pid = g_drive_target_r_pid_slew;
            target_speed_l_pid = g_drive_target_l_pid_slew;
        }
#if PID_DRIVE_SWAP_REAR_ENCODERS
        feedback_r_pid = ((float)g_ecod2_speed_filt) * PID_DRIVE_R_ENCODER_SIGN;
        feedback_l_pid = ((float)g_ecod1_speed_filt) * PID_DRIVE_L_ENCODER_SIGN;
#else
        feedback_r_pid = ((float)g_ecod1_speed_filt) * PID_DRIVE_R_ENCODER_SIGN;
        feedback_l_pid = ((float)g_ecod2_speed_filt) * PID_DRIVE_L_ENCODER_SIGN;
#endif

        (void)pid_calc_drive(PID_CTRL_REAR_RIGHT_SPEED,
                             target_speed_r_pid,
                             feedback_r_pid,
                             PID_SPEED_DT_S,
                             cmd.brake_to_stop);
        (void)pid_calc_drive(PID_CTRL_REAR_LEFT_SPEED,
                             target_speed_l_pid,
                             feedback_l_pid,
                             PID_SPEED_DT_S,
                             cmd.brake_to_stop);

        drive_pid_output_r = pid_get_output(PID_CTRL_REAR_RIGHT_SPEED);
        drive_pid_output_l = pid_get_output(PID_CTRL_REAR_LEFT_SPEED);
        if ((ctrl_diag_absf(target_speed_r_pid) >= CTRL_PID_LOW_TARGET_ECOD) &&
            (ctrl_diag_absf(drive_pid_output_r) <= CTRL_PID_LOW_OUTPUT_DUTY))
        {
            if ((ctrl_diag_absf(target_speed_l_pid) >= CTRL_PID_LOW_TARGET_ECOD) &&
                (ctrl_diag_absf(drive_pid_output_l) <= CTRL_PID_LOW_OUTPUT_DUTY))
            {
                ctrl_pid_diag_mark_low(CTRL_PID_LOW_REASON_BOTH,
                                       now_ms,
                                       target_speed_r_pid,
                                       feedback_r_pid,
                                       drive_pid_output_r,
                                       target_speed_l_pid,
                                       feedback_l_pid,
                                       drive_pid_output_l);
            }
            else
            {
                ctrl_pid_diag_mark_low(CTRL_PID_LOW_REASON_RIGHT,
                                       now_ms,
                                       target_speed_r_pid,
                                       feedback_r_pid,
                                       drive_pid_output_r,
                                       target_speed_l_pid,
                                       feedback_l_pid,
                                       drive_pid_output_l);
            }
        }
        else if ((ctrl_diag_absf(target_speed_l_pid) >= CTRL_PID_LOW_TARGET_ECOD) &&
                 (ctrl_diag_absf(drive_pid_output_l) <= CTRL_PID_LOW_OUTPUT_DUTY))
        {
            ctrl_pid_diag_mark_low(CTRL_PID_LOW_REASON_LEFT,
                                   now_ms,
                                   target_speed_r_pid,
                                   feedback_r_pid,
                                   drive_pid_output_r,
                                   target_speed_l_pid,
                                   feedback_l_pid,
                                   drive_pid_output_l);
        }

#if PID_DRIVE_STALL_PROTECT_ENABLE
        if ((((target_speed_r_pid >= PID_DRIVE_STALL_TARGET_ECOD) ||
              (target_speed_r_pid <= -PID_DRIVE_STALL_TARGET_ECOD)) &&
             (feedback_r_pid <= PID_DRIVE_STALL_FEEDBACK_ECOD) &&
             (feedback_r_pid >= -PID_DRIVE_STALL_FEEDBACK_ECOD) &&
             ((drive_pid_output_r >= PID_DRIVE_STALL_OUTPUT_DUTY) ||
              (drive_pid_output_r <= -PID_DRIVE_STALL_OUTPUT_DUTY))) ||
            (((target_speed_l_pid >= PID_DRIVE_STALL_TARGET_ECOD) ||
              (target_speed_l_pid <= -PID_DRIVE_STALL_TARGET_ECOD)) &&
             (feedback_l_pid <= PID_DRIVE_STALL_FEEDBACK_ECOD) &&
             (feedback_l_pid >= -PID_DRIVE_STALL_FEEDBACK_ECOD) &&
             ((drive_pid_output_l >= PID_DRIVE_STALL_OUTPUT_DUTY) ||
              (drive_pid_output_l <= -PID_DRIVE_STALL_OUTPUT_DUTY))))
        {
            if (drive_stall_start_ms == 0U)
            {
                drive_stall_start_ms = now_ms;
            }
            else if ((now_ms - drive_stall_start_ms) >= PID_DRIVE_STALL_TIME_MS)
            {
                g_ctrl_stop_reason = CTRL_STOP_REASON_DRIVE_STALL_LOCAL;
                speed_tick = 0U;
                drive_stall_start_ms = 0U;
#if (PID_DRIVE_STALL_PAUSE_MS > 0U)
                g_ctrl_safety_pause_until_ms = now_ms + PID_DRIVE_STALL_PAUSE_MS;
                g_ctrl_safety_pause_active = 1U;
#endif
                ctrl_drive_target_slew_reset();
                ctrl_pid_diag_mark_reset(CTRL_PID_RESET_REASON_DRIVE_STALL, now_ms);
                pid_reset_all();
                ctrl_stop_all_outputs();
                return;
            }
        }
        else
        {
            drive_stall_start_ms = 0U;
        }
#else
        drive_stall_start_ms = 0U;
#endif
    }
    pid_apply_to_dianji();
}

IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
}

IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);

    if (exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        imu660rc_callback();
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, EXTI_CH1_CH5_INT_VECTAB_NUM, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);

    if (exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        tof_module_exti_handler();
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);

    if (exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();
    }

#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
    if (exti_flag_get(DOT_MATRIX_SCREEN_SYNC_PIN))
    {
        exti_flag_clear(DOT_MATRIX_SCREEN_SYNC_PIN);
        dot_matrix_screen_scan();
    }
#else
    if (exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
#endif
}

IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}

IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);
#if DEBUG_UART_USE_INTERRUPT
    debug_interrupr_handler();
#endif
}

IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO) { interrupt_global_enable(0); }
#if TLD7002_PROJECT_ENABLE
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    tld7002_callback();
}
#else
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO) { interrupt_global_enable(0); camera_uart_handler(); }
#endif
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO) { interrupt_global_enable(0); wireless_module_uart_handler(); }
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO) { interrupt_global_enable(0); gnss_uart_callback(); }
IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);
#if YAOKONG_REMOTE_ENABLE
    yaokong_handler();
#endif
}
IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO) { interrupt_global_enable(0); lora_uart_rx_isr(); }
IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart19_tx_isr, UART19_INT_VECTAB_NUM, UART19_TX_INT_PRIO) { interrupt_global_enable(0); }
IFX_INTERRUPT(uart19_rx_isr, UART19_INT_VECTAB_NUM, UART19_RX_INT_PRIO) { interrupt_global_enable(0); gnss_uart_callback(); }

IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart0_handle); }
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart1_handle); }
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart2_handle); }
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart3_handle); }
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart4_handle); }
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart5_handle); }
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart6_handle); }
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart8_handle); }
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart9_handle); }
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart10_handle); }
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart11_handle); }
IFX_INTERRUPT(uart19_er_isr, UART19_INT_VECTAB_NUM, UART19_ER_INT_PRIO) { interrupt_global_enable(0); IfxAsclin_Asc_isrError(&uart19_handle); }
//dy ahstu zhugeliang ltl
