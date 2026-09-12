#include "nav_fusion.h"
#include "nav_ins.h"
#include "nav_path.h"

#define NAV_FUSION_DEFAULT_GPS_TIMEOUT_MS  (600U)

static nav_state_t g_nav_state = {0};
static uint32 g_nav_fusion_gps_timeout_ms = NAV_FUSION_DEFAULT_GPS_TIMEOUT_MS;

void nav_fusion_init(void)
{
    g_nav_state.valid = 0U;
    g_nav_state.gps_valid = 0U;
    g_nav_state.ins_valid = 0U;
    g_nav_state.gps_ins_correction_enabled = 0U;
    g_nav_state.timestamp_ms = 0U;
    g_nav_state.last_gps_ms = 0U;
    g_nav_state.x_m = 0.0f;
    g_nav_state.y_m = 0.0f;
    g_nav_state.yaw_deg = 0.0f;
    g_nav_state.yaw_rate_dps = 0.0f;
    g_nav_state.forward_accel_mps2 = 0.0f;
    g_nav_state.speed_mps = 0.0f;
    g_nav_state.encoder_speed_mps = 0.0f;
    g_nav_state.distance_m = 0.0f;
    g_nav_state.gps_ins_error_m = 0.0f;
    g_nav_state.gps_ins_correction_gain = 0.0f;
    g_nav_state.gps_ins_snap_threshold_m = 0.0f;
    g_nav_state.path_valid = 0U;
    g_nav_state.path_active = 0U;
    g_nav_state.path_finished = 0U;
    g_nav_state.path_last_sd_result = 255U;
    g_nav_state.path_last_flash_result = 255U;
    g_nav_state.path_last_flash_load_result = 255U;
    g_nav_state.path_mode = NAV_PATH_MODE_IDLE;
    g_nav_state.path_sample_count = 0U;
    g_nav_state.path_replay_index = 0U;
    g_nav_state.path_sample_interval_m = 0.0f;
    g_nav_state.path_progress_m = 0.0f;
    g_nav_state.path_ref_x_m = 0.0f;
    g_nav_state.path_ref_y_m = 0.0f;
    g_nav_state.path_ref_yaw_deg = 0.0f;
    g_nav_state.path_ref_kappa_1pm = 0.0f;
    g_nav_state.path_ref_kappa_ahead_1pm = 0.0f;
    g_nav_state.path_ref_kappa_yaw_1pm = 0.0f;
    g_nav_state.path_ref_speed_mps = 0.0f;
    g_nav_state.path_ref_speed_ahead_mps = 0.0f;
    g_nav_state.path_ref_motion_dir = 1;
    g_nav_state.path_ref_motion_dir_ahead = 1;
    g_nav_state.path_yaw_error_deg = 0.0f;
    g_nav_state.path_lateral_error_m = 0.0f;
    g_nav_fusion_gps_timeout_ms = NAV_FUSION_DEFAULT_GPS_TIMEOUT_MS;
}

void nav_fusion_set_gps_timeout_ms(uint32 timeout_ms)
{
    if (timeout_ms > 0U)
    {
        g_nav_fusion_gps_timeout_ms = timeout_ms;
    }
}

void nav_fusion_update_from_gps(const nav_gps_sample_t *gps_sample)
{
    if ((gps_sample == NULL) || (0U == gps_sample->valid))
    {
        return;
    }

    g_nav_state.x_m = gps_sample->x_m;
    g_nav_state.y_m = gps_sample->y_m;
    g_nav_state.speed_mps = gps_sample->speed_mps;
    g_nav_state.timestamp_ms = gps_sample->timestamp_ms;
    g_nav_state.last_gps_ms = gps_sample->timestamp_ms;
    g_nav_state.gps_valid = 1U;
    g_nav_state.valid = 1U;
}

void nav_fusion_update_from_ins(const nav_ins_state_t *ins_state)
{
    nav_path_state_t path_state;

    if ((ins_state == NULL) || (0U == ins_state->valid))
    {
        g_nav_state.ins_valid = 0U;
        return;
    }

    g_nav_state.x_m = ins_state->x_m;
    g_nav_state.y_m = ins_state->y_m;
    g_nav_state.yaw_deg = ins_state->yaw_deg;
    g_nav_state.yaw_rate_dps = ins_state->yaw_rate_dps;
    g_nav_state.forward_accel_mps2 = ins_state->forward_accel_mps2;
    g_nav_state.speed_mps = ins_state->speed_mps;
    g_nav_state.encoder_speed_mps = ins_state->encoder_speed_mps;
    g_nav_state.distance_m = ins_state->distance_m;
    g_nav_state.timestamp_ms = ins_state->timestamp_ms;

    nav_path_get_state(&path_state);
    g_nav_state.path_valid = path_state.valid;
    g_nav_state.path_active = path_state.active;
    g_nav_state.path_finished = path_state.finished;
    g_nav_state.path_last_sd_result = path_state.last_sd_result;
    g_nav_state.path_last_flash_result = path_state.last_flash_result;
    g_nav_state.path_last_flash_load_result = path_state.last_flash_load_result;
    g_nav_state.path_mode = path_state.mode;
    g_nav_state.path_sample_count = path_state.sample_count;
    g_nav_state.path_replay_index = path_state.replay_index;
    g_nav_state.path_sample_interval_m = path_state.sample_interval_m;
    g_nav_state.path_progress_m = path_state.progress_m;
    g_nav_state.path_ref_x_m = path_state.ref_x_m;
    g_nav_state.path_ref_y_m = path_state.ref_y_m;
    g_nav_state.path_ref_yaw_deg = path_state.ref_yaw_deg;
    g_nav_state.path_ref_kappa_1pm = path_state.ref_kappa_1pm;
    g_nav_state.path_ref_kappa_ahead_1pm = path_state.ref_kappa_ahead_1pm;
    g_nav_state.path_ref_kappa_yaw_1pm = path_state.ref_kappa_yaw_1pm;
    g_nav_state.path_ref_speed_mps = path_state.ref_speed_mps;
    g_nav_state.path_ref_speed_ahead_mps = path_state.ref_speed_ahead_mps;
    g_nav_state.path_ref_motion_dir = path_state.ref_motion_dir;
    g_nav_state.path_ref_motion_dir_ahead = path_state.ref_motion_dir_ahead;
    g_nav_state.path_yaw_error_deg = path_state.yaw_error_deg;
    g_nav_state.path_lateral_error_m = path_state.lateral_error_m;

    g_nav_state.ins_valid = 1U;
    g_nav_state.valid = 1U;

    if ((0U == g_nav_state.last_gps_ms) ||
        ((ins_state->timestamp_ms - g_nav_state.last_gps_ms) > g_nav_fusion_gps_timeout_ms))
    {
        g_nav_state.gps_valid = 0U;
    }
}

void nav_fusion_get_state(nav_state_t *out_state)
{
    if (out_state != NULL)
    {
        *out_state = g_nav_state;
    }
}

uint8 nav_fusion_is_valid(void)
{
    return g_nav_state.valid;
}
