#include "taban.h"
#include "taban_config.h"

static taban_status_t g_taban_status = {0};
static const adc_channel_enum g_taban_adc_channel[TABAN_ID_MAX] =
{
    TABAN0_ADC_CHANNEL,
    TABAN1_ADC_CHANNEL
};
static const uint16 g_taban_adc_min[TABAN_ID_MAX] =
{
    TABAN0_ADC_MIN,
    TABAN1_ADC_MIN
};
static const uint16 g_taban_adc_max[TABAN_ID_MAX] =
{
    TABAN0_ADC_MAX,
    TABAN1_ADC_MAX
};
static const uint16 g_taban_deadzone_low[TABAN_ID_MAX] =
{
    TABAN0_ADC_DEADZONE_LOW,
    TABAN1_ADC_DEADZONE_LOW
};
static const uint16 g_taban_deadzone_high[TABAN_ID_MAX] =
{
    TABAN0_ADC_DEADZONE_HIGH,
    TABAN1_ADC_DEADZONE_HIGH
};
static const uint16 g_taban_release_adc[TABAN_ID_MAX] =
{
    TABAN0_STARTUP_RELEASE_ADC,
    TABAN1_STARTUP_RELEASE_ADC
};
static const uint16 g_taban_fault_low[TABAN_ID_MAX] =
{
    TABAN0_ADC_FAULT_LOW,
    TABAN1_ADC_FAULT_LOW
};
static const uint16 g_taban_fault_high[TABAN_ID_MAX] =
{
    TABAN0_ADC_FAULT_HIGH,
    TABAN1_ADC_FAULT_HIGH
};

static uint8 taban_id_valid(taban_id_t pedal_id)
{
    return ((uint8)pedal_id < (uint8)TABAN_ID_MAX);
}

static uint16 taban_clip_u16(uint16 value, uint16 low, uint16 high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

static uint16 taban_calc_permille(taban_id_t pedal_id, uint16 filt_adc)
{
    uint16 pedal_adc;
    uint32 numerator;
    uint16 range_adc;
    uint16 low_edge;
    uint16 high_edge;
    uint16 adc_min;
    uint16 adc_max;

    if (0U == taban_id_valid(pedal_id))
    {
        return 0U;
    }

    adc_min = g_taban_adc_min[pedal_id];
    adc_max = g_taban_adc_max[pedal_id];

    if (adc_max <= adc_min)
    {
        return 0U;
    }

    low_edge = (uint16)(adc_min + g_taban_deadzone_low[pedal_id]);
    if (adc_max > g_taban_deadzone_high[pedal_id])
    {
        high_edge = (uint16)(adc_max - g_taban_deadzone_high[pedal_id]);
    }
    else
    {
        high_edge = adc_max;
    }

    if (high_edge <= low_edge)
    {
        low_edge = adc_min;
        high_edge = adc_max;
    }

    if (filt_adc <= low_edge)
    {
        return 0U;
    }

    if (filt_adc >= high_edge)
    {
        return 1000U;
    }

    pedal_adc = taban_clip_u16(filt_adc, low_edge, high_edge);
    range_adc = (uint16)(high_edge - low_edge);
    numerator = (uint32)(pedal_adc - low_edge) * 1000U;

    return (uint16)(numerator / range_adc);
}

static float taban_clip_f32(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}

static uint16 taban_blend_permille(uint16 linear_permille, uint16 shaped_permille, uint16 blend_permille)
{
    uint32 mix;

    if (blend_permille > 1000U)
    {
        blend_permille = 1000U;
    }

    mix = (uint32)linear_permille * (uint32)(1000U - blend_permille);
    mix += (uint32)shaped_permille * (uint32)blend_permille;

    return (uint16)(mix / 1000U);
}

static uint16 taban_shape_accel_permille(uint16 in_permille)
{
    uint32 square_permille;
    uint16 shaped_permille;

    if (in_permille >= 1000U)
    {
        return 1000U;
    }

    square_permille = (uint32)in_permille * (uint32)in_permille;
    shaped_permille = (uint16)(square_permille / 1000U);

    return taban_blend_permille(in_permille, shaped_permille, TABAN_ACCEL_CURVE_SOFTEN_PERMILLE);
}

static uint16 taban_shape_brake_permille(uint16 in_permille)
{
    uint16 eff_in;
    uint16 full_in;
    uint16 freeplay;
    uint32 enhanced;
    uint16 shaped_permille;

    freeplay = TABAN_BRAKE_FREEPLAY_PERMILLE;
    full_in = TABAN_BRAKE_FULL_INPUT_PERMILLE;

    if (full_in > 1000U)
    {
        full_in = 1000U;
    }

    if (full_in <= freeplay)
    {
        full_in = (uint16)(freeplay + 1U);
        if (full_in > 1000U)
        {
            full_in = 1000U;
        }
    }

    if (in_permille <= freeplay)
    {
        return 0U;
    }

    if (in_permille >= full_in)
    {
        eff_in = 1000U;
    }
    else if (freeplay >= 999U)
    {
        eff_in = 1000U;
    }
    else
    {
        eff_in = (uint16)(((uint32)(in_permille - freeplay) * 1000U) /
                          (uint32)(full_in - freeplay));
    }

    if (eff_in >= 1000U)
    {
        return 1000U;
    }

    // x*(2-x) in permille domain, stronger response in early stroke.
    enhanced = (uint32)eff_in * (uint32)(2000U - eff_in);
    shaped_permille = (uint16)(enhanced / 1000U);

    return taban_blend_permille(eff_in, shaped_permille, TABAN_BRAKE_CURVE_ENHANCE_PERMILLE);
}

void taban_init(void)
{
    uint8 i;
    uint16 init_sample;

    for (i = 0U; i < (uint8)TABAN_ID_MAX; i++)
    {
        adc_init(g_taban_adc_channel[i], TABAN_ADC_RESOLUTION);

        init_sample = adc_mean_filter_convert(g_taban_adc_channel[i], TABAN_SAMPLE_COUNT);
        g_taban_status.raw_adc[i] = init_sample;
        g_taban_status.filt_adc[i] = init_sample;
        g_taban_status.throttle_permille[i] = 0U;
        g_taban_status.released_once[i] = 0U;
        g_taban_status.fault_active[i] = 0U;

        if (init_sample <= g_taban_release_adc[i])
        {
            g_taban_status.released_once[i] = 1U;
        }
    }

    g_taban_status.target_speed_mps = 0.0f;
}

void taban_task(void)
{
    uint8 i;
    uint16 sample_adc;
    uint32 filt_acc;
    uint16 accel_permille;
    uint16 brake_permille;
    uint16 accel_cmd_permille;
    uint16 brake_cmd_permille;
    uint32 speed_cmd_permille;
    float target_speed;

    for (i = 0U; i < (uint8)TABAN_ID_MAX; i++)
    {
        sample_adc = adc_mean_filter_convert(g_taban_adc_channel[i], TABAN_SAMPLE_COUNT);
        g_taban_status.raw_adc[i] = sample_adc;

        if ((sample_adc <= g_taban_fault_low[i]) || (sample_adc >= g_taban_fault_high[i]))
        {
            g_taban_status.fault_active[i] = 1U;
        }
        else
        {
            g_taban_status.fault_active[i] = 0U;
        }

        filt_acc = (uint32)g_taban_status.filt_adc[i] * (uint32)(TABAN_FILTER_DIV - 1U);
        filt_acc += sample_adc;
        g_taban_status.filt_adc[i] = (uint16)(filt_acc / TABAN_FILTER_DIV);

        if (g_taban_status.filt_adc[i] <= g_taban_release_adc[i])
        {
            g_taban_status.released_once[i] = 1U;
        }

        if ((0U != g_taban_status.fault_active[i]) || (0U == g_taban_status.released_once[i]))
        {
            g_taban_status.throttle_permille[i] = 0U;
        }
        else
        {
            g_taban_status.throttle_permille[i] = taban_calc_permille((taban_id_t)i, g_taban_status.filt_adc[i]);
        }
    }

    // TABAN_ID_0: throttle pedal, TABAN_ID_1: brake pedal.
    accel_permille = g_taban_status.throttle_permille[TABAN_ID_0];
    brake_permille = g_taban_status.throttle_permille[TABAN_ID_1];

    accel_cmd_permille = taban_shape_accel_permille(accel_permille);
    brake_cmd_permille = taban_shape_brake_permille(brake_permille);

    if (brake_cmd_permille >= TABAN_BRAKE_OVERRIDE_PERMILLE)
    {
        speed_cmd_permille = 0U;
    }
    else
    {
        speed_cmd_permille = (uint32)accel_cmd_permille * (uint32)(1000U - brake_cmd_permille);
        speed_cmd_permille /= 1000U;
    }

    target_speed = (float)speed_cmd_permille * (TABAN_TARGET_SPEED_MAX_MPS / 1000.0f);
    g_taban_status.target_speed_mps = taban_clip_f32(target_speed, 0.0f, TABAN_TARGET_SPEED_MAX_MPS);
}

uint16 taban_get_raw_adc(taban_id_t pedal_id)
{
    if (0U == taban_id_valid(pedal_id))
    {
        return 0U;
    }

    return g_taban_status.raw_adc[pedal_id];
}

uint16 taban_get_filt_adc(taban_id_t pedal_id)
{
    if (0U == taban_id_valid(pedal_id))
    {
        return 0U;
    }

    return g_taban_status.filt_adc[pedal_id];
}

uint16 taban_get_throttle_permille(taban_id_t pedal_id)
{
    if (0U == taban_id_valid(pedal_id))
    {
        return 0U;
    }

    return g_taban_status.throttle_permille[pedal_id];
}

uint8 taban_is_released_once(taban_id_t pedal_id)
{
    if (0U == taban_id_valid(pedal_id))
    {
        return 0U;
    }

    return g_taban_status.released_once[pedal_id];
}

uint8 taban_is_fault_active(taban_id_t pedal_id)
{
    if (0U == taban_id_valid(pedal_id))
    {
        return 0U;
    }

    return g_taban_status.fault_active[pedal_id];
}

float taban_get_target_speed_mps(void)
{
    return g_taban_status.target_speed_mps;
}

void taban_get_status(taban_status_t *status)
{
    if (NULL == status)
    {
        return;
    }

    *status = g_taban_status;
}
