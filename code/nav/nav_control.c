#include "nav_control.h"
#include "nav_gps.h"
#include "nav_ins.h"
#include "nav_fusion.h"
#include "nav_path.h"
#include "dici.h"
#include "zf_device_gnss.h"
#include <math.h>

#define NAV_IMU_G_TO_MPS2            (9.80665f)
#define NAV_IMU_FORWARD_ACCEL_MPS2() (imu_data.acc_y * NAV_IMU_G_TO_MPS2)
#define NAV_CONTROL_DEFAULT_GPS_CORRECTION_GAIN       (0.35f)
#define NAV_CONTROL_DEFAULT_GPS_CORRECTION_SNAP_M     (3.0f)

static uint8 g_nav_control_gps_correct_ins = 1U;
static float g_nav_control_gps_correction_gain = NAV_CONTROL_DEFAULT_GPS_CORRECTION_GAIN;
static float g_nav_control_gps_correction_snap_m = NAV_CONTROL_DEFAULT_GPS_CORRECTION_SNAP_M;
static float g_nav_control_last_gps_ins_error_m = 0.0f;

static void NavControl_CorrectInsFromGps(const nav_gps_sample_t *gps_sample)
{
    nav_ins_state_t ins_state;
    uint8 ins_was_valid;
    float dx;
    float dy;

    if ((0U == g_nav_control_gps_correct_ins) ||
        (gps_sample == NULL) ||
        (0U == gps_sample->valid))
    {
        return;
    }

    nav_ins_get_state(&ins_state);
    ins_was_valid = ins_state.valid;
    if (0U != ins_state.valid)
    {
        dx = gps_sample->x_m - ins_state.x_m;
        dy = gps_sample->y_m - ins_state.y_m;
        g_nav_control_last_gps_ins_error_m = sqrtf(dx * dx + dy * dy);
    }
    else
    {
        g_nav_control_last_gps_ins_error_m = 0.0f;
    }

    nav_ins_correct_position(gps_sample->x_m,
                             gps_sample->y_m,
                             g_nav_control_gps_correction_gain,
                             g_nav_control_gps_correction_snap_m);
    if (0U != ins_was_valid)
    {
        nav_ins_get_state(&ins_state);
        nav_fusion_update_from_ins(&ins_state);
    }
}

void nav_control_init(void)
{
    nav_gps_init();
    nav_ins_init();
    nav_path_init();
    nav_fusion_init();
    g_nav_control_gps_correct_ins = 1U;
    g_nav_control_gps_correction_gain = NAV_CONTROL_DEFAULT_GPS_CORRECTION_GAIN;
    g_nav_control_gps_correction_snap_m = NAV_CONTROL_DEFAULT_GPS_CORRECTION_SNAP_M;
    g_nav_control_last_gps_ins_error_m = 0.0f;
}

void nav_control_set_encoder_count_to_meter(float count_to_meter)
{
    nav_ins_set_count_to_meter(count_to_meter);
}

void nav_control_set_gps_timeout_ms(uint32 timeout_ms)
{
    nav_fusion_set_gps_timeout_ms(timeout_ms);
}

void nav_control_set_gps_ins_correction(uint8 enable, float blend_gain, float snap_threshold_m)
{
    if (blend_gain < 0.0f)
    {
        blend_gain = 0.0f;
    }
    else if (blend_gain > 1.0f)
    {
        blend_gain = 1.0f;
    }

    g_nav_control_gps_correct_ins = (enable != 0U) ? 1U : 0U;
    g_nav_control_gps_correction_gain = blend_gain;
    g_nav_control_gps_correction_snap_m = snap_threshold_m;
}

void nav_control_set_gps_origin(double latitude_deg, double longitude_deg)
{
    nav_gps_set_origin(latitude_deg, longitude_deg);
}

uint8 nav_control_set_gps_origin_from_current(void)
{
    return nav_gps_set_origin_from_current();
}

uint8 nav_control_update_gps_sample(const nav_gps_sample_t *gps_sample)
{
    nav_gps_sample_t checked_sample;

    if (0U == nav_gps_update_sample(gps_sample, &checked_sample))
    {
        return 0U;
    }

    nav_fusion_update_from_gps(&checked_sample);
    NavControl_CorrectInsFromGps(&checked_sample);
    return 1U;
}

uint8 nav_control_poll_gps(uint32 timestamp_ms)
{
    nav_gps_sample_t gps_sample;

    if (0U == gnss_flag)
    {
        return 0U;
    }

    gnss_flag = 0U;
    if (0U != gnss_data_parse())
    {
        return 0U;
    }

    if (0U == nav_gps_update_from_gnss(timestamp_ms, &gps_sample))
    {
        return 0U;
    }

    nav_fusion_update_from_gps(&gps_sample);
    NavControl_CorrectInsFromGps(&gps_sample);
    return 1U;
}

void nav_control_update_ins(uint32 timestamp_ms,
                            int16 encoder_r_count,
                            int16 encoder_l_count,
                            float yaw_deg,
                            float yaw_rate_dps)
{
    nav_control_update_ins_with_accel(timestamp_ms,
                                      encoder_r_count,
                                      encoder_l_count,
                                      yaw_deg,
                                      yaw_rate_dps,
                                      0.0f);
}

void nav_control_update_ins_with_accel(uint32 timestamp_ms,
                                       int16 encoder_r_count,
                                       int16 encoder_l_count,
                                       float yaw_deg,
                                       float yaw_rate_dps,
                                       float forward_accel_mps2)
{
    nav_ins_state_t ins_state;

    nav_ins_update_with_accel(timestamp_ms,
                              encoder_r_count,
                              encoder_l_count,
                              yaw_deg,
                              yaw_rate_dps,
                              forward_accel_mps2);
    nav_ins_get_state(&ins_state);
    nav_path_update(ins_state.distance_m,
                    ins_state.x_m,
                    ins_state.y_m,
                    ins_state.yaw_deg,
                    ins_state.speed_mps);
    nav_fusion_update_from_ins(&ins_state);
}

uint8 nav_control_update_ins_from_imu(uint32 timestamp_ms,
                                      int16 encoder_r_count,
                                      int16 encoder_l_count)
{
    if (IMU_AHRS_IsReferenceReady() == 0U)
    {
        return 0U;
    }

    nav_control_update_ins_with_accel(timestamp_ms,
                                      encoder_r_count,
                                      encoder_l_count,
                                      IMU_RelYaw,
                                      IMU_GYRO_Z,
                                      NAV_IMU_FORWARD_ACCEL_MPS2());
    return 1U;
}

void nav_control_get_state(nav_state_t *out_state)
{
    nav_fusion_get_state(out_state);
    if (out_state != NULL)
    {
        out_state->gps_ins_correction_enabled = g_nav_control_gps_correct_ins;
        out_state->gps_ins_error_m = g_nav_control_last_gps_ins_error_m;
        out_state->gps_ins_correction_gain = g_nav_control_gps_correction_gain;
        out_state->gps_ins_snap_threshold_m = g_nav_control_gps_correction_snap_m;
    }
}

uint8 nav_control_is_valid(void)
{
    return nav_fusion_is_valid();
}

uint8 nav_control_path_start_record(float sample_interval_m)
{
    return nav_path_start_record(sample_interval_m);
}

uint8 nav_control_path_start_replay(void)
{
    return nav_path_start_replay();
}

uint8 nav_control_path_start_replay_reverse(void)
{
    return nav_path_start_replay_reverse();
}

uint8 nav_control_path_start_replay_reverse_fixed_yaw(void)
{
    return nav_path_start_replay_reverse_fixed_yaw();
}

uint8 nav_control_path_start_replay_backward(void)
{
    return nav_path_start_replay_backward();
}

uint8 nav_control_path_start_replay_backward_fixed_yaw(void)
{
    return nav_path_start_replay_backward_fixed_yaw();
}

uint8 nav_control_path_start_replay_reverse_fixed_yaw_from_step(uint16 start_step)
{
    return nav_path_start_replay_reverse_fixed_yaw_from_step(start_step);
}

uint8 nav_control_path_start_replay_reverse_fixed_ins_from_step(uint16 start_step)
{
    return nav_path_start_replay_reverse_fixed_ins_from_step(start_step);
}

void nav_control_path_set_replay_progress_gate(uint8 enable)
{
    nav_path_set_replay_progress_gate(enable);
}

void nav_control_path_set_replay_rescue_project(uint8 enable)
{
    nav_path_set_replay_rescue_project(enable);
}

uint8 nav_control_path_find_reverse_fixed_ins_match(float x_m,
                                                    float y_m,
                                                    float yaw_deg,
                                                    uint16 max_step,
                                                    nav_path_replay_match_t *out_match)
{
    return nav_path_find_reverse_fixed_ins_match(x_m,
                                                 y_m,
                                                 yaw_deg,
                                                 max_step,
                                                 out_match);
}

void nav_control_path_stop(void)
{
    nav_path_stop();
}

void nav_control_path_clear(void)
{
    nav_path_clear();
}

uint8 nav_control_path_get_replay_local_position(float x_m,
                                                 float y_m,
                                                 float *local_x_m,
                                                 float *local_y_m)
{
    return nav_path_get_replay_local_position(x_m,
                                              y_m,
                                              local_x_m,
                                              local_y_m);
}

uint8 nav_control_path_get_replay_point(uint16 index,
                                        float *x_m,
                                        float *y_m)
{
    return nav_path_get_replay_point(index, x_m, y_m);
}

uint8 nav_control_path_save_to_sd(const char *filename)
{
    return nav_path_save_to_sd(filename);
}

uint8 nav_control_path_request_save_to_sd(uint8 task_id,
                                          const char *filename,
                                          uint8 flash_result,
                                          uint8 flash_load_result,
                                          uint8 ref_ready,
                                          uint8 ins_valid,
                                          const nav_ins_state_t *ins_state)
{
    return nav_path_request_save_to_sd(task_id,
                                       filename,
                                       flash_result,
                                       flash_load_result,
                                       ref_ready,
                                       ins_valid,
                                       ins_state);
}

uint8 nav_control_path_sd_save_is_busy(void)
{
    return nav_path_sd_save_is_busy();
}

uint8 nav_control_path_force_last_sample_yaw(float yaw_deg)
{
    return nav_path_force_last_sample_yaw(yaw_deg);
}

uint8 nav_control_path_append_status_to_sd(uint8 task_id,
                                           const char *event,
                                           uint8 sd_result,
                                           uint8 flash_result,
                                           uint8 flash_load_result,
                                           uint8 ref_ready,
                                           uint8 ins_valid,
                                           uint8 logger_state,
                                           uint8 logger_error,
                                           uint8 logger_task,
                                           uint8 logger_profile,
                                           uint8 logger_target_src)
{
    return nav_path_append_status_to_sd(task_id,
                                        event,
                                        sd_result,
                                        flash_result,
                                        flash_load_result,
                                        ref_ready,
                                        ins_valid,
                                        logger_state,
                                        logger_error,
                                        logger_task,
                                        logger_profile,
                                        logger_target_src);
}

uint8 nav_control_path_append_record_diag_to_sd(uint8 task_id,
                                                const char *event,
                                                const nav_ins_state_t *ins_state)
{
    return nav_path_append_record_diag_to_sd(task_id, event, ins_state);
}

uint8 nav_control_path_load_from_sd(const char *filename)
{
    return nav_path_load_from_sd(filename);
}

uint8 nav_control_path_save_to_flash(void)
{
    return nav_path_save_to_flash();
}

uint8 nav_control_path_load_from_flash(void)
{
    return nav_path_load_from_flash();
}

uint8 nav_control_path_save_to_flash_task(uint8 task_id)
{
    return nav_path_save_to_flash_task(task_id);
}

uint8 nav_control_path_load_from_flash_task(uint8 task_id)
{
    return nav_path_load_from_flash_task(task_id);
}

void nav_control_path_get_state(nav_path_state_t *out_state)
{
    nav_path_get_state(out_state);
}
