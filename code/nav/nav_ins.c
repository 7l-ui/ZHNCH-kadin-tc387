#include "nav_ins.h"
#include <math.h>

#define NAV_INS_DEFAULT_COUNT_TO_METER  (0.200f / 360.0f)
#define NAV_INS_ACCEL_EPS_MPS2          (0.03f)
#define NAV_INS_SPEED_FILTER_ALPHA      (0.35f)
#define NAV_INS_ACCEL_SPEED_GAIN        (0.15f)
#define NAV_INS_MAX_DT_MS               (200U)
#define NAV_INS_ENCODER_COUNT_WRAP      (16384)
#define NAV_INS_ENCODER_COUNT_HALF_WRAP (NAV_INS_ENCODER_COUNT_WRAP / 2)

static nav_ins_state_t g_nav_ins_state = {0};
static uint8 g_nav_ins_encoder_ready = 0U;
static float g_nav_ins_count_to_meter = NAV_INS_DEFAULT_COUNT_TO_METER;
static float g_nav_ins_speed_filter_alpha = NAV_INS_SPEED_FILTER_ALPHA;
static float g_nav_ins_accel_speed_gain = NAV_INS_ACCEL_SPEED_GAIN;

static float NavIns_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static int32 NavIns_EncoderDelta16(int16 current_count, int16 last_count)
{
    int32 delta = (int32)current_count - (int32)last_count;

    if (delta > NAV_INS_ENCODER_COUNT_HALF_WRAP)
    {
        delta -= NAV_INS_ENCODER_COUNT_WRAP;
    }
    else if (delta < -NAV_INS_ENCODER_COUNT_HALF_WRAP)
    {
        delta += NAV_INS_ENCODER_COUNT_WRAP;
    }

    return delta;
}

static float NavIns_WrapDeltaDeg(float angle_deg)
{
    return NavIns_Wrap180(angle_deg);
}

static float NavIns_MidYawDeg(float last_yaw_deg, float current_yaw_deg)
{
    return NavIns_Wrap180(last_yaw_deg + 0.5f * NavIns_WrapDeltaDeg(current_yaw_deg - last_yaw_deg));
}

static void NavIns_UpdateInternal(uint32 timestamp_ms,
                                  int16 encoder_r_count,
                                  int16 encoder_l_count,
                                  float yaw_deg,
                                  float yaw_rate_dps,
                                  float forward_accel_mps2,
                                  uint8 use_accel)
{
    uint32 dt_ms;
    int32 delta_r;
    int32 delta_l;
    float delta_count;
    float encoder_distance_m;
    float dt_s;
    float encoder_speed_mps;
    float measured_speed_mps;
    float accel_speed_mps;
    float filtered_speed_mps;
    float mid_yaw_deg;
    float yaw_rad;
    float delta_x_m;
    float delta_y_m;

    if (0U == g_nav_ins_encoder_ready)
    {
        nav_ins_reset_encoder(encoder_r_count, encoder_l_count, timestamp_ms);
        g_nav_ins_state.yaw_deg = NavIns_Wrap180(yaw_deg);
        g_nav_ins_state.yaw_rate_dps = yaw_rate_dps;
        g_nav_ins_state.forward_accel_mps2 = forward_accel_mps2;
        g_nav_ins_state.encoder_speed_mps = 0.0f;
        g_nav_ins_state.update_dt_ms = 0U;
        g_nav_ins_state.encoder_delta_r = 0;
        g_nav_ins_state.encoder_delta_l = 0;
        g_nav_ins_state.encoder_delta_avg = 0.0f;
        g_nav_ins_state.delta_distance_m = 0.0f;
        g_nav_ins_state.delta_x_m = 0.0f;
        g_nav_ins_state.delta_y_m = 0.0f;
        g_nav_ins_state.yaw_used_deg = g_nav_ins_state.yaw_deg;
        g_nav_ins_state.count_to_meter = g_nav_ins_count_to_meter;
        g_nav_ins_state.valid = 1U;
        return;
    }

    dt_ms = timestamp_ms - g_nav_ins_state.timestamp_ms;
    if (0U == dt_ms)
    {
        g_nav_ins_state.yaw_deg = NavIns_Wrap180(yaw_deg);
        g_nav_ins_state.yaw_rate_dps = yaw_rate_dps;
        g_nav_ins_state.forward_accel_mps2 = forward_accel_mps2;
        g_nav_ins_state.update_dt_ms = 0U;
        g_nav_ins_state.encoder_delta_r = 0;
        g_nav_ins_state.encoder_delta_l = 0;
        g_nav_ins_state.encoder_delta_avg = 0.0f;
        g_nav_ins_state.delta_distance_m = 0.0f;
        g_nav_ins_state.delta_x_m = 0.0f;
        g_nav_ins_state.delta_y_m = 0.0f;
        g_nav_ins_state.yaw_used_deg = g_nav_ins_state.yaw_deg;
        g_nav_ins_state.count_to_meter = g_nav_ins_count_to_meter;
        return;
    }

    if (dt_ms > NAV_INS_MAX_DT_MS)
    {
        nav_ins_reset_encoder(encoder_r_count, encoder_l_count, timestamp_ms);
        g_nav_ins_state.yaw_deg = NavIns_Wrap180(yaw_deg);
        g_nav_ins_state.yaw_rate_dps = yaw_rate_dps;
        g_nav_ins_state.forward_accel_mps2 = forward_accel_mps2;
        g_nav_ins_state.encoder_speed_mps = 0.0f;
        g_nav_ins_state.update_dt_ms = dt_ms;
        g_nav_ins_state.encoder_delta_r = 0;
        g_nav_ins_state.encoder_delta_l = 0;
        g_nav_ins_state.encoder_delta_avg = 0.0f;
        g_nav_ins_state.delta_distance_m = 0.0f;
        g_nav_ins_state.delta_x_m = 0.0f;
        g_nav_ins_state.delta_y_m = 0.0f;
        g_nav_ins_state.yaw_used_deg = g_nav_ins_state.yaw_deg;
        g_nav_ins_state.count_to_meter = g_nav_ins_count_to_meter;
        return;
    }

    delta_r = NavIns_EncoderDelta16(encoder_r_count, g_nav_ins_state.encoder_r_count);
    delta_l = NavIns_EncoderDelta16(encoder_l_count, g_nav_ins_state.encoder_l_count);
    delta_count = ((float)delta_r + (float)delta_l) * 0.5f;
    encoder_distance_m = delta_count * g_nav_ins_count_to_meter;
    dt_s = (float)dt_ms * 0.001f;
    encoder_speed_mps = encoder_distance_m / dt_s;
    measured_speed_mps = encoder_speed_mps;

    if (0U != use_accel)
    {
        if ((forward_accel_mps2 < NAV_INS_ACCEL_EPS_MPS2) &&
            (forward_accel_mps2 > -NAV_INS_ACCEL_EPS_MPS2))
        {
            forward_accel_mps2 = 0.0f;
        }

        accel_speed_mps = g_nav_ins_state.speed_mps + forward_accel_mps2 * dt_s;
        measured_speed_mps = encoder_speed_mps * (1.0f - g_nav_ins_accel_speed_gain) +
                             accel_speed_mps * g_nav_ins_accel_speed_gain;
    }

    filtered_speed_mps = g_nav_ins_state.speed_mps * (1.0f - g_nav_ins_speed_filter_alpha) +
                         measured_speed_mps * g_nav_ins_speed_filter_alpha;

    yaw_deg = NavIns_Wrap180(yaw_deg);
    mid_yaw_deg = NavIns_MidYawDeg(g_nav_ins_state.yaw_deg, yaw_deg);
    yaw_rad = mid_yaw_deg * NAV_PI / 180.0f;

    delta_x_m = sinf(yaw_rad) * encoder_distance_m;
    delta_y_m = cosf(yaw_rad) * encoder_distance_m;

    g_nav_ins_state.x_m += delta_x_m;
    g_nav_ins_state.y_m += delta_y_m;
    g_nav_ins_state.yaw_deg = yaw_deg;
    g_nav_ins_state.yaw_rate_dps = yaw_rate_dps;
    g_nav_ins_state.forward_accel_mps2 = forward_accel_mps2;
    g_nav_ins_state.speed_mps = filtered_speed_mps;
    g_nav_ins_state.encoder_speed_mps = encoder_speed_mps;
    g_nav_ins_state.distance_m += encoder_distance_m;
    g_nav_ins_state.encoder_r_count = encoder_r_count;
    g_nav_ins_state.encoder_l_count = encoder_l_count;
    g_nav_ins_state.update_dt_ms = dt_ms;
    g_nav_ins_state.encoder_delta_r = delta_r;
    g_nav_ins_state.encoder_delta_l = delta_l;
    g_nav_ins_state.encoder_delta_avg = delta_count;
    g_nav_ins_state.delta_distance_m = encoder_distance_m;
    g_nav_ins_state.delta_x_m = delta_x_m;
    g_nav_ins_state.delta_y_m = delta_y_m;
    g_nav_ins_state.yaw_used_deg = mid_yaw_deg;
    g_nav_ins_state.count_to_meter = g_nav_ins_count_to_meter;
    g_nav_ins_state.timestamp_ms = timestamp_ms;
    g_nav_ins_state.valid = 1U;
}

void nav_ins_init(void)
{
    g_nav_ins_state.valid = 0U;
    g_nav_ins_state.x_m = 0.0f;
    g_nav_ins_state.y_m = 0.0f;
    g_nav_ins_state.yaw_deg = 0.0f;
    g_nav_ins_state.yaw_rate_dps = 0.0f;
    g_nav_ins_state.forward_accel_mps2 = 0.0f;
    g_nav_ins_state.speed_mps = 0.0f;
    g_nav_ins_state.encoder_speed_mps = 0.0f;
    g_nav_ins_state.distance_m = 0.0f;
    g_nav_ins_state.encoder_r_count = 0;
    g_nav_ins_state.encoder_l_count = 0;
    g_nav_ins_state.update_dt_ms = 0U;
    g_nav_ins_state.encoder_delta_r = 0;
    g_nav_ins_state.encoder_delta_l = 0;
    g_nav_ins_state.encoder_delta_avg = 0.0f;
    g_nav_ins_state.delta_distance_m = 0.0f;
    g_nav_ins_state.delta_x_m = 0.0f;
    g_nav_ins_state.delta_y_m = 0.0f;
    g_nav_ins_state.yaw_used_deg = 0.0f;
    g_nav_ins_state.count_to_meter = NAV_INS_DEFAULT_COUNT_TO_METER;
    g_nav_ins_encoder_ready = 0U;
    g_nav_ins_count_to_meter = NAV_INS_DEFAULT_COUNT_TO_METER;
    g_nav_ins_speed_filter_alpha = NAV_INS_SPEED_FILTER_ALPHA;
    g_nav_ins_accel_speed_gain = NAV_INS_ACCEL_SPEED_GAIN;
}

void nav_ins_set_count_to_meter(float count_to_meter)
{
    if (count_to_meter > 0.0f)
    {
        g_nav_ins_count_to_meter = count_to_meter;
        g_nav_ins_state.count_to_meter = count_to_meter;
    }
}

void nav_ins_set_speed_filter_alpha(float alpha)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    g_nav_ins_speed_filter_alpha = alpha;
}

void nav_ins_set_accel_speed_gain(float gain)
{
    if (gain < 0.0f)
    {
        gain = 0.0f;
    }
    else if (gain > 1.0f)
    {
        gain = 1.0f;
    }

    g_nav_ins_accel_speed_gain = gain;
}

void nav_ins_reset_position(float x_m, float y_m)
{
    g_nav_ins_state.x_m = x_m;
    g_nav_ins_state.y_m = y_m;
    g_nav_ins_state.valid = 1U;
}

void nav_ins_correct_position(float x_m, float y_m, float blend_gain, float snap_threshold_m)
{
    float dx;
    float dy;
    float error_m;

    if (blend_gain < 0.0f)
    {
        blend_gain = 0.0f;
    }
    else if (blend_gain > 1.0f)
    {
        blend_gain = 1.0f;
    }

    if ((0U == g_nav_ins_state.valid) || (blend_gain >= 1.0f))
    {
        nav_ins_reset_position(x_m, y_m);
        return;
    }

    dx = x_m - g_nav_ins_state.x_m;
    dy = y_m - g_nav_ins_state.y_m;
    error_m = sqrtf(dx * dx + dy * dy);

    if ((snap_threshold_m > 0.0f) && (error_m > snap_threshold_m))
    {
        nav_ins_reset_position(x_m, y_m);
        return;
    }

    g_nav_ins_state.x_m += dx * blend_gain;
    g_nav_ins_state.y_m += dy * blend_gain;
    g_nav_ins_state.valid = 1U;
}

void nav_ins_reset_encoder(int16 encoder_r_count, int16 encoder_l_count, uint32 timestamp_ms)
{
    g_nav_ins_state.encoder_r_count = encoder_r_count;
    g_nav_ins_state.encoder_l_count = encoder_l_count;
    g_nav_ins_state.timestamp_ms = timestamp_ms;
    g_nav_ins_state.update_dt_ms = 0U;
    g_nav_ins_state.encoder_delta_r = 0;
    g_nav_ins_state.encoder_delta_l = 0;
    g_nav_ins_state.encoder_delta_avg = 0.0f;
    g_nav_ins_state.delta_distance_m = 0.0f;
    g_nav_ins_state.delta_x_m = 0.0f;
    g_nav_ins_state.delta_y_m = 0.0f;
    g_nav_ins_state.count_to_meter = g_nav_ins_count_to_meter;
    g_nav_ins_encoder_ready = 1U;
}

void nav_ins_update(uint32 timestamp_ms,
                    int16 encoder_r_count,
                    int16 encoder_l_count,
                    float yaw_deg,
                    float yaw_rate_dps)
{
    NavIns_UpdateInternal(timestamp_ms,
                          encoder_r_count,
                          encoder_l_count,
                          yaw_deg,
                          yaw_rate_dps,
                          0.0f,
                          0U);
}

void nav_ins_update_with_accel(uint32 timestamp_ms,
                               int16 encoder_r_count,
                               int16 encoder_l_count,
                               float yaw_deg,
                               float yaw_rate_dps,
                               float forward_accel_mps2)
{
    NavIns_UpdateInternal(timestamp_ms,
                          encoder_r_count,
                          encoder_l_count,
                          yaw_deg,
                          yaw_rate_dps,
                          forward_accel_mps2,
                          1U);
}

void nav_ins_get_state(nav_ins_state_t *out_state)
{
    if (out_state != NULL)
    {
        *out_state = g_nav_ins_state;
    }
}
