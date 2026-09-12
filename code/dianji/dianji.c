#include "dianji.h"
#include "dianji_config.h"
#include "xl9555.h"
#include "xl9555_app.h"

#if ((DIANJI_DRIVER_MODE != DIANJI_DRIVER_PWM_PWM) && (DIANJI_DRIVER_MODE != DIANJI_DRIVER_PH_EN_8701))
#error "Unsupported DIANJI_DRIVER_MODE"
#endif

#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PWM_PWM)
#if ((DIANJI_PWM_OUTPUT_SIGN_M != 1) && (DIANJI_PWM_OUTPUT_SIGN_M != -1))
#error "DIANJI_PWM_OUTPUT_SIGN_M must be 1 or -1"
#endif
#if ((DIANJI_PWM_OUTPUT_SIGN_R != 1) && (DIANJI_PWM_OUTPUT_SIGN_R != -1))
#error "DIANJI_PWM_OUTPUT_SIGN_R must be 1 or -1"
#endif
#if ((DIANJI_PWM_OUTPUT_SIGN_L != 1) && (DIANJI_PWM_OUTPUT_SIGN_L != -1))
#error "DIANJI_PWM_OUTPUT_SIGN_L must be 1 or -1"
#endif
#endif

#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PH_EN_8701)
typedef struct
{
    gpio_pin_enum ph;
    pwm_channel_enum en;
    uint8 forward_level;
    uint8 reverse_level;
} dianji_ph_en_pair_t;

static const dianji_ph_en_pair_t g_dianji_ph_en_pair[DIANJI_MOTOR_MAX] =
{
    {DIANJI_PH_M_PIN, DIANJI_PWM_BPWM_M_CH, DIANJI_PH_FORWARD_LEVEL_M, DIANJI_PH_REVERSE_LEVEL_M},
    {DIANJI_PH_R_PIN, DIANJI_PWM_BPWM_R_CH, DIANJI_PH_FORWARD_LEVEL_R, DIANJI_PH_REVERSE_LEVEL_R},
    {DIANJI_PH_L_PIN, DIANJI_PWM_BPWM_L_CH, DIANJI_PH_FORWARD_LEVEL_L, DIANJI_PH_REVERSE_LEVEL_L}
};
#else
typedef struct
{
    pwm_channel_enum apwm;
    pwm_channel_enum bpwm;
} dianji_pwm_pair_t;

static const dianji_pwm_pair_t g_dianji_pwm_pair[DIANJI_MOTOR_MAX] =
{
    {DIANJI_PWM_APWM_M_CH, DIANJI_PWM_BPWM_M_CH},
    {DIANJI_PWM_APWM_R_CH, DIANJI_PWM_BPWM_R_CH},
    {DIANJI_PWM_APWM_L_CH, DIANJI_PWM_BPWM_L_CH}
};

static const int8 g_dianji_pwm_output_sign[DIANJI_MOTOR_MAX] =
{
    DIANJI_PWM_OUTPUT_SIGN_M,
    DIANJI_PWM_OUTPUT_SIGN_R,
    DIANJI_PWM_OUTPUT_SIGN_L
};
#endif

static dianji_status_t g_dianji_status = {0};
static uint32 g_enable_request_ms = 0U;
static int8 g_dianji_output_dir[DIANJI_MOTOR_MAX] = {0};
static int8 g_dianji_last_active_dir[DIANJI_MOTOR_MAX] = {0};
static uint32 g_dianji_off_since_ms[DIANJI_MOTOR_MAX] = {0};

static uint16 dianji_abs_int16(int16 v)
{
    if (v >= 0)
    {
        return (uint16)v;
    }

    // Protect INT16_MIN edge case.
    if (v == (int16)-32768)
    {
        return (uint16)32767;
    }

    return (uint16)(-v);
}

static uint16 dianji_limit_u16(uint16 value, uint16 limit)
{
    if (value > limit)
    {
        return limit;
    }
    return value;
}

static int8 dianji_get_duty_dir(int16 duty_signed)
{
    if (duty_signed > 0)
    {
        return 1;
    }
    if (duty_signed < 0)
    {
        return -1;
    }
    return 0;
}

static uint8 dianji_elapsed_ms_reached(uint32 now_ms, uint32 start_ms, uint32 interval_ms)
{
    return (((uint32)(now_ms - start_ms)) >= interval_ms) ? 1U : 0U;
}

static uint16 dianji_get_pwm_deadtime_limit(void)
{
#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PH_EN_8701)
    return (uint16)PWM_DUTY_MAX;
#else
    uint32 reserved_duty = (uint32)((((uint64)PWM_DUTY_MAX *
                                      (uint64)DIANJI_PWM_SOFT_DEADTIME_US *
                                      (uint64)DIANJI_PWM_FREQ_HZ) + 999999ULL) /
                                    1000000ULL);

    if (reserved_duty >= PWM_DUTY_MAX)
    {
        return 0U;
    }

    return (uint16)((uint32)PWM_DUTY_MAX - reserved_duty);
#endif
}

static uint16 dianji_get_config_limit(dianji_motor_id_t motor)
{
    switch (motor)
    {
        case DIANJI_MOTOR_M:
            return (uint16)DIANJI_DUTY_LIMIT_M;
        case DIANJI_MOTOR_R:
            return (uint16)DIANJI_DUTY_LIMIT_R;
        case DIANJI_MOTOR_L:
            return (uint16)DIANJI_DUTY_LIMIT_L;
        default:
            return (uint16)DIANJI_DUTY_LIMIT;
    }
}

static uint16 dianji_get_safe_limit(dianji_motor_id_t motor)
{
    uint16 limit = dianji_get_config_limit(motor);
    uint16 deadtime_limit = dianji_get_pwm_deadtime_limit();

    if (limit > PWM_DUTY_MAX)
    {
        limit = PWM_DUTY_MAX;
    }
    if (limit > deadtime_limit)
    {
        limit = deadtime_limit;
    }
    return limit;
}

static xl9555_dev_t *dianji_get_xl9555_dev(void)
{
    return (xl9555_dev_t *)xl9555_app_get_device();
}

static void dianji_set_xl9555_pin_level(uint8 pin, uint8 level)
{
    xl9555_dev_t *dev = dianji_get_xl9555_dev();
    if ((NULL != dev) && (NULL != dev->soft_iic_obj))
    {
        xl9555_set_gpio_level(dev, pin, level);
    }
}

static void dianji_set_enable_hw(uint8 enable_on)
{
    if (enable_on)
    {
        dianji_set_xl9555_pin_level(DIANJI_XL9555_ENABLE_PIN, DIANJI_ENABLE_ACTIVE_LEVEL);
        dianji_set_xl9555_pin_level(DIANJI_XL9555_RUNLED_PIN, DIANJI_RUNLED_ON_LEVEL);
    }
    else
    {
        dianji_set_xl9555_pin_level(DIANJI_XL9555_ENABLE_PIN, DIANJI_ENABLE_INACTIVE_LEVEL);
        dianji_set_xl9555_pin_level(DIANJI_XL9555_RUNLED_PIN, DIANJI_RUNLED_OFF_LEVEL);
    }
}

static void dianji_set_motor_pwm_off(dianji_motor_id_t motor)
{
#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PH_EN_8701)
    pwm_set_duty(g_dianji_ph_en_pair[motor].en, 0U);
#else
    pwm_set_duty(g_dianji_pwm_pair[motor].apwm, 0U);
    pwm_set_duty(g_dianji_pwm_pair[motor].bpwm, 0U);
#endif
}

static void dianji_mark_motor_off(dianji_motor_id_t motor, uint32 now_ms)
{
    if (0 != g_dianji_output_dir[motor])
    {
        g_dianji_off_since_ms[motor] = now_ms;
    }

    g_dianji_output_dir[motor] = 0;
    dianji_set_motor_pwm_off(motor);
}

static void dianji_reset_output_state(uint32 now_ms)
{
    uint8 i;

    for (i = 0U; i < (uint8)DIANJI_MOTOR_MAX; i++)
    {
        g_dianji_output_dir[i] = 0;
        g_dianji_last_active_dir[i] = 0;
        g_dianji_off_since_ms[i] = now_ms;
        dianji_set_motor_pwm_off((dianji_motor_id_t)i);
    }
}

#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PH_EN_8701)
static uint8 dianji_get_ph_level(dianji_motor_id_t motor, int8 target_dir)
{
    if (target_dir > 0)
    {
        return g_dianji_ph_en_pair[motor].forward_level;
    }
    return g_dianji_ph_en_pair[motor].reverse_level;
}
#endif

static void dianji_apply_one_motor(dianji_motor_id_t motor, int16 duty_signed, uint32 now_ms)
{
    uint16 duty_abs;
    uint16 duty_limit;
#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PWM_PWM)
    uint16 duty_apwm = 0U;
    uint16 duty_bpwm = 0U;
#endif
    int8 target_dir = dianji_get_duty_dir(duty_signed);

    if ((uint8)motor >= (uint8)DIANJI_MOTOR_MAX)
    {
        return;
    }

    duty_limit = dianji_get_safe_limit(motor);
    duty_abs = dianji_abs_int16(duty_signed);
    duty_abs = dianji_limit_u16(duty_abs, duty_limit);

    if ((0 == target_dir) || (0U == duty_abs))
    {
        dianji_mark_motor_off(motor, now_ms);
        return;
    }

    if ((0 != g_dianji_output_dir[motor]) && (g_dianji_output_dir[motor] != target_dir))
    {
        dianji_mark_motor_off(motor, now_ms);
        return;
    }

    if ((0 == g_dianji_output_dir[motor]) &&
        (0 != g_dianji_last_active_dir[motor]) &&
        (g_dianji_last_active_dir[motor] != target_dir) &&
        (0U == dianji_elapsed_ms_reached(now_ms,
                                         g_dianji_off_since_ms[motor],
                                         DIANJI_DIR_CHANGE_BLANK_MS)))
    {
        dianji_set_motor_pwm_off(motor);
        return;
    }

#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PWM_PWM)
    if ((0 == g_dianji_output_dir[motor]) && (0U != DIANJI_PWM_SOFT_DEADTIME_US))
    {
        dianji_set_motor_pwm_off(motor);
        system_delay_us(DIANJI_PWM_SOFT_DEADTIME_US);
    }

    if ((target_dir * g_dianji_pwm_output_sign[motor]) > 0)
    {
        duty_apwm = duty_abs;
    }
    else
    {
        duty_bpwm = duty_abs;
    }

    pwm_set_duty(g_dianji_pwm_pair[motor].apwm, duty_apwm);
    pwm_set_duty(g_dianji_pwm_pair[motor].bpwm, duty_bpwm);
#else
    if (0 == g_dianji_output_dir[motor])
    {
        dianji_set_motor_pwm_off(motor);
        gpio_set_level(g_dianji_ph_en_pair[motor].ph, dianji_get_ph_level(motor, target_dir));
        if (0U != DIANJI_PH_EN_DIR_SETUP_US)
        {
            system_delay_us(DIANJI_PH_EN_DIR_SETUP_US);
        }
    }
    else
    {
        gpio_set_level(g_dianji_ph_en_pair[motor].ph, dianji_get_ph_level(motor, target_dir));
    }

    pwm_set_duty(g_dianji_ph_en_pair[motor].en, duty_abs);
#endif
    g_dianji_output_dir[motor] = target_dir;
    g_dianji_last_active_dir[motor] = target_dir;
}

static void dianji_apply_all_output(void)
{
    uint8 i;
    uint32 now_ms = system_getval_ms();

    if ((0U == g_dianji_status.enable_active) || (0U != g_dianji_status.fault_latched))
    {
        dianji_reset_output_state(now_ms);
        return;
    }

    for (i = 0U; i < (uint8)DIANJI_MOTOR_MAX; i++)
    {
        dianji_apply_one_motor((dianji_motor_id_t)i, g_dianji_status.target_duty[i], now_ms);
    }
}

static void dianji_force_disable_now(void)
{
    g_dianji_status.enable_active = 0U;
    dianji_set_enable_hw(0U);
    dianji_apply_all_output();
}

void dianji_init(void)
{
    uint8 i;
    uint32 now_ms;

    // Ensure XL9555 is ready before touching ENABLE/RUNLED.
    xl9555_app_init();

    g_dianji_status.initialized = 1U;
    g_dianji_status.enable_requested = 0U;
    g_dianji_status.enable_active = 0U;
    g_dianji_status.fault_latched = 0U;
    g_enable_request_ms = 0U;
    now_ms = system_getval_ms();

    for (i = 0U; i < (uint8)DIANJI_MOTOR_MAX; i++)
    {
        g_dianji_status.target_duty[i] = 0;
        g_dianji_output_dir[i] = 0;
        g_dianji_last_active_dir[i] = 0;
        g_dianji_off_since_ms[i] = now_ms;
    }

    // Stage-1: disable driver first.
    dianji_set_enable_hw(0U);

    // Stage-2: initialize motor control outputs to safe off state.
#if (DIANJI_DRIVER_MODE == DIANJI_DRIVER_PH_EN_8701)
    gpio_init(DIANJI_PH_M_PIN, GPO, DIANJI_PH_FORWARD_LEVEL_M, GPO_PUSH_PULL);
    gpio_init(DIANJI_PH_R_PIN, GPO, DIANJI_PH_FORWARD_LEVEL_R, GPO_PUSH_PULL);
    gpio_init(DIANJI_PH_L_PIN, GPO, DIANJI_PH_FORWARD_LEVEL_L, GPO_PUSH_PULL);
    pwm_init(DIANJI_PWM_BPWM_M_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_BPWM_R_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_BPWM_L_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
#else
    pwm_init(DIANJI_PWM_APWM_M_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_BPWM_M_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_APWM_R_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_BPWM_R_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_APWM_L_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
    pwm_init(DIANJI_PWM_BPWM_L_CH, DIANJI_PWM_FREQ_HZ, DIANJI_PWM_INIT_DUTY);
#endif

    dianji_apply_all_output();

    // Stage-3: hold safe delay after power-up.
    system_delay_ms(DIANJI_BOOT_SAFE_DELAY_MS);
}

void dianji_task(void)
{
    uint32 now_ms;

    if (0U == g_dianji_status.initialized)
    {
        return;
    }

    if ((0U != g_dianji_status.fault_latched) || (0U == g_dianji_status.enable_requested))
    {
        if (0U != g_dianji_status.enable_active)
        {
            dianji_force_disable_now();
        }
        else
        {
            dianji_apply_all_output();
        }
        return;
    }

    if (0U == g_dianji_status.enable_active)
    {
        now_ms = system_getval_ms();
        if ((now_ms - g_enable_request_ms) >= DIANJI_ENABLE_GUARD_DELAY_MS)
        {
            g_dianji_status.enable_active = 1U;
            dianji_set_enable_hw(1U);
        }
        else
        {
            dianji_apply_all_output();
            return;
        }
    }

    dianji_apply_all_output();
}

void dianji_request_enable(uint8 enable_req)
{
    if (0U == g_dianji_status.initialized)
    {
        return;
    }

    if (0U == enable_req)
    {
        g_dianji_status.enable_requested = 0U;
        dianji_force_disable_now();
        return;
    }

    if (0U != g_dianji_status.fault_latched)
    {
        return;
    }

    g_dianji_status.enable_requested = 1U;
    g_enable_request_ms = system_getval_ms();
}

void dianji_emergency_stop(void)
{
    if (0U == g_dianji_status.initialized)
    {
        return;
    }

    g_dianji_status.fault_latched = 1U;
    g_dianji_status.enable_requested = 0U;
    dianji_force_disable_now();
}

void dianji_clear_fault(void)
{
    if (0U == g_dianji_status.initialized)
    {
        return;
    }

    g_dianji_status.fault_latched = 0U;
}

void dianji_set_motor_duty(dianji_motor_id_t motor, int16 duty_signed)
{
    int16 duty_tmp = duty_signed;
    uint16 duty_limit;

    if (0U == g_dianji_status.initialized)
    {
        return;
    }

    if ((uint8)motor >= (uint8)DIANJI_MOTOR_MAX)
    {
        return;
    }

    duty_limit = dianji_get_safe_limit(motor);
    if (duty_tmp > (int16)duty_limit)
    {
        duty_tmp = (int16)duty_limit;
    }
    else if (duty_tmp < -(int16)duty_limit)
    {
        duty_tmp = -(int16)duty_limit;
    }

    g_dianji_status.target_duty[motor] = duty_tmp;
    dianji_apply_all_output();
}

void dianji_set_all_duty(int16 duty_m, int16 duty_r, int16 duty_l)
{
    if (0U == g_dianji_status.initialized)
    {
        return;
    }

    dianji_set_motor_duty(DIANJI_MOTOR_M, duty_m);
    dianji_set_motor_duty(DIANJI_MOTOR_R, duty_r);
    dianji_set_motor_duty(DIANJI_MOTOR_L, duty_l);
}

void dianji_get_status(dianji_status_t *status)
{
    if (NULL == status)
    {
        return;
    }

    *status = g_dianji_status;
}
