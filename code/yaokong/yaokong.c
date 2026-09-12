#include "yaokong.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
#include <string.h>

#define YAOKONG_FRAME_GAP_US      (3000UL)
#define YAOKONG_SIGNAL_TIMEOUT_US (3000000UL)
#define YAOKONG_DEADZONE_ENTER    (35)
#define YAOKONG_DEADZONE_EXIT     (35)
#define YAOKONG_SPEED_FILTER_ALPHA (0.35f)
#define YAOKONG_STEER_FILTER_ALPHA (0.35f)
#define YAOKONG_SPEED_ZERO_EPS    (0.015f)
#define YAOKONG_STEER_ZERO_EPS_DEG (0.20f)
#define YAOKONG_CH3_PRESS_THRESHOLD (1360U)
#define YAOKONG_CH5_PRESS_THRESHOLD (1360U)
#define YAOKONG_CH6_PRESS_THRESHOLD (1360U)
#define YAOKONG_CH6_DEBUG_PRINT_ENABLE (1U)
#define YAOKONG_CH6_DEBUG_RAW_DELTA    (40U)

volatile yaokong_status_t g_yaokong = {0};

static uint8  yaokong_rx_buf[YAOKONG_FRAME_LEN];
static uint8  yaokong_rx_len = 0;
static uint32 yaokong_last_byte_us = 0;
static volatile uint32 yaokong_last_frame_us = 0;
static uint8  yaokong_ch3_level_valid = 0U;
static uint8  yaokong_ch3_last_level = 0U;
static uint8  yaokong_ch5_level_valid = 0U;
static uint8  yaokong_ch5_last_level = 0U;
static uint8  yaokong_ch6_level_valid = 0U;
static uint8  yaokong_ch6_last_level = 0U;
static uint8  yaokong_ch6_press_pending = 0U;
#if YAOKONG_CH6_DEBUG_PRINT_ENABLE
static uint8  yaokong_ch6_debug_raw_valid = 0U;
static uint16 yaokong_ch6_debug_last_raw = 0U;
static uint8  yaokong_ch6_debug_last_pressed = 0U;
#endif
static uint8  yaokong_remote_armed = 0U;
static uint8  yaokong_speed_axis_active = 0U;
static uint8  yaokong_steer_axis_active = 0U;
static float  yaokong_speed_target_filt = 0.0f;
static float  yaokong_steer_target_filt = 0.0f;
static uint32 yaokong_remote_neutral_start_us = 0U;
static volatile uint32 yaokong_valid_frame_count = 0U;
static volatile uint32 yaokong_bad_frame_count = 0U;

static float yaokong_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static uint32 yaokong_now_us(void)
{
    return system_getval_us();
}

static uint32 yaokong_elapsed_us(uint32 now_us, uint32 start_us)
{
    return now_us - start_us;
}

#if YAOKONG_CH6_DEBUG_PRINT_ENABLE
static uint16 yaokong_absdiff_u16(uint16 a, uint16 b)
{
    return (a >= b) ? (uint16)(a - b) : (uint16)(b - a);
}

static void yaokong_ch6_debug_reset(void)
{
    yaokong_ch6_debug_raw_valid = 0U;
    yaokong_ch6_debug_last_raw = 0U;
    yaokong_ch6_debug_last_pressed = 0U;
}

static void yaokong_ch6_debug_update(uint16 raw_ch6, uint8 pressed)
{
    if ((yaokong_ch6_debug_raw_valid == 0U) ||
        (pressed != yaokong_ch6_debug_last_pressed) ||
        (yaokong_absdiff_u16(raw_ch6, yaokong_ch6_debug_last_raw) >=
         YAOKONG_CH6_DEBUG_RAW_DELTA))
    {
        yaokong_ch6_debug_raw_valid = 1U;
        yaokong_ch6_debug_last_raw = raw_ch6;
        yaokong_ch6_debug_last_pressed = pressed;
        printf("[REMOTE_TEST] CH6 raw=%u threshold=%u pressed=%u\r\n",
               (unsigned)raw_ch6,
               (unsigned)YAOKONG_CH6_PRESS_THRESHOLD,
               (unsigned)pressed);
    }
}
#else
static void yaokong_ch6_debug_reset(void)
{
}

static void yaokong_ch6_debug_update(uint16 raw_ch6, uint8 pressed)
{
    (void)raw_ch6;
    (void)pressed;
}
#endif

static void yaokong_rx_reset(void)
{
    yaokong_rx_len = 0;
}

static void yaokong_target_filter_reset(void)
{
    yaokong_speed_axis_active = 0U;
    yaokong_steer_axis_active = 0U;
    yaokong_speed_target_filt = 0.0f;
    yaokong_steer_target_filt = 0.0f;
    g_yaokong.target_speed_mps = 0.0f;
    g_yaokong.target_steer_angle_deg = 0.0f;
}

static uint8 yaokong_axis_active_update(int32 diff, uint8 active)
{
    int32 abs_diff = (diff >= 0) ? diff : -diff;

    if (0U != active)
    {
        return (abs_diff > YAOKONG_DEADZONE_ENTER) ? 1U : 0U;
    }

    return (abs_diff >= YAOKONG_DEADZONE_EXIT) ? 1U : 0U;
}

static float yaokong_filter_target(float current, float target, float alpha, float zero_eps)
{
    current += alpha * (target - current);
    if ((yaokong_absf(target) <= 1e-6f) &&
        (current <= zero_eps) && (current >= -zero_eps))
    {
        current = 0.0f;
    }

    return current;
}

static void yaokong_parse_frame(void)
{
    uint8 *buf = yaokong_rx_buf;

    g_yaokong.channel[0] = (uint16)((buf[1]       | (buf[2] << 8)) & 0x07FF);
    g_yaokong.channel[1] = (uint16)(((buf[2] >> 3) | (buf[3] << 5)) & 0x07FF);
    g_yaokong.channel[2] = (uint16)(((buf[3] >> 6) | (buf[4] << 2)  | (buf[5] << 10)) & 0x07FF);
    g_yaokong.channel[3] = (uint16)(((buf[5] >> 1) | (buf[6] << 7)) & 0x07FF);
    g_yaokong.channel[4] = (uint16)(((buf[6] >> 4) | (buf[7] << 4)) & 0x07FF);
    g_yaokong.channel[5] = (uint16)(((buf[7] >> 7) | (buf[8] << 1)  | (buf[9] << 9)) & 0x07FF);

    g_yaokong.state = ((buf[23] & YAOKONG_ABNORMAL_MASK) != 0U) ? 0U : 1U;
    g_yaokong.finsh_flag = 1U;
    yaokong_last_frame_us = yaokong_now_us();
    yaokong_valid_frame_count++;
}

void yaokong_handler(void)
{
    uint8 byte;
    uint32 now_us;

    while (uart_query_byte(YAOKONG_UART_INDEX, &byte))
    {
        now_us = yaokong_now_us();
        if (yaokong_elapsed_us(now_us, yaokong_last_byte_us) > YAOKONG_FRAME_GAP_US)
        {
            yaokong_rx_reset();
        }
        yaokong_last_byte_us = now_us;

        if (yaokong_rx_len == 0U)
        {
            if (byte != YAOKONG_FRAME_HEADER)
            {
                continue;
            }
            yaokong_rx_buf[yaokong_rx_len++] = byte;
            continue;
        }

        if (yaokong_rx_len >= YAOKONG_FRAME_LEN)
        {
            yaokong_rx_reset();
            if (byte != YAOKONG_FRAME_HEADER)
            {
                continue;
            }
        }

        yaokong_rx_buf[yaokong_rx_len++] = byte;

        if (yaokong_rx_len >= YAOKONG_FRAME_LEN)
        {
            if ((yaokong_rx_buf[0] == YAOKONG_FRAME_HEADER) &&
                (yaokong_rx_buf[YAOKONG_FRAME_LEN - 1U] == YAOKONG_FRAME_FOOTER))
            {
                yaokong_parse_frame();
            }
            else
            {
                yaokong_bad_frame_count++;
            }
            yaokong_rx_reset();
        }
    }
}

void yaokong_init(void)
{
    memset((void *)&g_yaokong, 0, sizeof(g_yaokong));
    memset(yaokong_rx_buf, 0, sizeof(yaokong_rx_buf));
    yaokong_rx_reset();
    yaokong_last_byte_us = yaokong_now_us();
    yaokong_last_frame_us = 0U;
    g_yaokong.guard_enable = YAOKONG_GUARD_ENABLE_DEFAULT;
    g_yaokong.motor_run_latch = 1U;
    g_yaokong.connected = 0U;
    g_yaokong.ch3_pressed = 0U;
    g_yaokong.ch5_pressed = 0U;
    g_yaokong.ch6_pressed = 0U;
    yaokong_ch3_level_valid = 0U;
    yaokong_ch3_last_level = 0U;
    yaokong_ch5_level_valid = 0U;
    yaokong_ch5_last_level = 0U;
    yaokong_ch6_level_valid = 0U;
    yaokong_ch6_last_level = 0U;
    yaokong_ch6_press_pending = 0U;
    yaokong_ch6_debug_reset();
    yaokong_remote_armed = 0U;
    yaokong_target_filter_reset();
    yaokong_remote_neutral_start_us = 0U;
    yaokong_valid_frame_count = 0U;
    yaokong_bad_frame_count = 0U;

    uart_sbus_init(YAOKONG_UART_INDEX, YAOKONG_UART_BAUDRATE, YAOKONG_UART_TX_PIN, YAOKONG_UART_RX_PIN);
}

void yaokong_update_speed(void)
{
    uint16 raw_speed;
    uint16 raw_steer;
    uint16 raw_ch3;
    uint16 raw_ch5;
    uint16 raw_ch6;
    int32 speed_diff;
    int32 steer_diff;
    float frac;
    float target_speed_raw = 0.0f;
    float target_steer_raw = 0.0f;
    uint32 now_us;
    uint32 last_frame_us;
    uint8 connected_now;
    uint8 ch3_pressed_now;
    uint8 ch5_pressed_now;
    uint8 ch6_pressed_now;

    last_frame_us = yaokong_last_frame_us;
    now_us = yaokong_now_us();
    connected_now = ((last_frame_us != 0U) &&
                     (yaokong_elapsed_us(now_us, last_frame_us) <= YAOKONG_SIGNAL_TIMEOUT_US)) ? 1U : 0U;
    g_yaokong.connected = connected_now;
    if (0U == connected_now)
    {
        g_yaokong.state = 0U;
        yaokong_target_filter_reset();
        g_yaokong.ch3_pressed = 0U;
        g_yaokong.ch5_pressed = 0U;
        g_yaokong.ch6_pressed = 0U;
        yaokong_ch3_level_valid = 0U;
        yaokong_ch5_level_valid = 0U;
        yaokong_ch6_level_valid = 0U;
        yaokong_ch6_press_pending = 0U;
        yaokong_ch6_debug_reset();
        yaokong_remote_armed = 0U;
        yaokong_remote_neutral_start_us = 0U;
        return;
    }

    if (g_yaokong.finsh_flag == 0U)
    {
        return;
    }
    g_yaokong.finsh_flag = 0U;

    if (g_yaokong.state == 0U)
    {
        yaokong_target_filter_reset();
        g_yaokong.ch3_pressed = 0U;
        g_yaokong.ch5_pressed = 0U;
        g_yaokong.ch6_pressed = 0U;
        yaokong_ch3_level_valid = 0U;
        yaokong_ch5_level_valid = 0U;
        yaokong_ch6_level_valid = 0U;
        yaokong_ch6_press_pending = 0U;
        yaokong_ch6_debug_reset();
        yaokong_remote_armed = 0U;
        yaokong_remote_neutral_start_us = 0U;
        return;
    }

    raw_ch3 = g_yaokong.channel[YAOKONG_CHANNEL_CH3_UNLOCK];
    raw_ch5 = g_yaokong.channel[YAOKONG_CHANNEL_CH5_BUTTON];
    raw_ch6 = g_yaokong.channel[YAOKONG_CHANNEL_CH6_BUTTON];
    ch3_pressed_now = (raw_ch3 > YAOKONG_CH3_PRESS_THRESHOLD) ? 1U : 0U;
    ch5_pressed_now = (raw_ch5 > YAOKONG_CH5_PRESS_THRESHOLD) ? 1U : 0U;
    ch6_pressed_now = (raw_ch6 > YAOKONG_CH6_PRESS_THRESHOLD) ? 1U : 0U;
    yaokong_ch6_debug_update(raw_ch6, ch6_pressed_now);
    if (0U == yaokong_ch5_level_valid)
    {
        yaokong_ch5_last_level = ch5_pressed_now;
        yaokong_ch5_level_valid = 1U;
        if (0U != ch5_pressed_now)
        {
            g_yaokong.motor_run_latch = 0U;
            yaokong_target_filter_reset();
            yaokong_remote_armed = 0U;
            yaokong_remote_neutral_start_us = 0U;
            printf("[REMOTE] CH5 motor stop latched\r\n");
        }
    }
    else if (ch5_pressed_now != yaokong_ch5_last_level)
    {
        g_yaokong.motor_run_latch = 0U;
        yaokong_target_filter_reset();
        yaokong_remote_armed = 0U;
        yaokong_remote_neutral_start_us = 0U;
        printf("[REMOTE] CH5 motor stop latched raw=%u pressed=%u\r\n",
               (unsigned)raw_ch5,
               (unsigned)ch5_pressed_now);
        yaokong_ch5_last_level = ch5_pressed_now;
    }
    g_yaokong.ch5_pressed = ch5_pressed_now;

    if (0U == yaokong_ch3_level_valid)
    {
        yaokong_ch3_last_level = ch3_pressed_now;
        yaokong_ch3_level_valid = 1U;
    }
    else if (ch3_pressed_now != yaokong_ch3_last_level)
    {
        if (0U == g_yaokong.motor_run_latch)
        {
            g_yaokong.motor_run_latch = 1U;
            yaokong_target_filter_reset();
            yaokong_remote_armed = 0U;
            yaokong_remote_neutral_start_us = 0U;
            printf("[REMOTE] CH3 unlock motor_run_latch raw=%u pressed=%u ch5=%u\r\n",
                   (unsigned)raw_ch3,
                   (unsigned)ch3_pressed_now,
                   (unsigned)ch5_pressed_now);
        }
        yaokong_ch3_last_level = ch3_pressed_now;
    }
    g_yaokong.ch3_pressed = ch3_pressed_now;

    if (0U == yaokong_ch6_level_valid)
    {
        yaokong_ch6_last_level = ch6_pressed_now;
        yaokong_ch6_level_valid = 1U;
    }
    else if (ch6_pressed_now != yaokong_ch6_last_level)
    {
        yaokong_ch6_press_pending = 1U;
        printf("[REMOTE] CH6 task3 action raw=%u threshold=%u level=%u\r\n",
               (unsigned)raw_ch6,
               (unsigned)YAOKONG_CH6_PRESS_THRESHOLD,
               (unsigned)ch6_pressed_now);
        yaokong_ch6_last_level = ch6_pressed_now;
    }
    g_yaokong.ch6_pressed = ch6_pressed_now;

    if (0U == g_yaokong.motor_run_latch)
    {
        yaokong_target_filter_reset();
        yaokong_remote_armed = 0U;
        yaokong_remote_neutral_start_us = 0U;
        return;
    }

    raw_speed = g_yaokong.channel[YAOKONG_CHANNEL_THROTTLE];
    speed_diff = (int32)raw_speed - (int32)YAOKONG_THROTTLE_MID;
    yaokong_speed_axis_active = yaokong_axis_active_update(speed_diff, yaokong_speed_axis_active);

    if (0U == yaokong_speed_axis_active)
    {
        target_speed_raw = 0.0f;
    }
    else if (speed_diff > 0)
    {
        frac = (float)speed_diff / (float)(YAOKONG_THROTTLE_POS_RAW - YAOKONG_THROTTLE_MID);
        if (frac > 1.0f)
        {
            frac = 1.0f;
        }
        target_speed_raw = frac * YAOKONG_TARGET_SPEED_MAX;
    }
    else
    {
        frac = (float)(-speed_diff) / (float)(YAOKONG_THROTTLE_MID - YAOKONG_THROTTLE_NEG_RAW);
        if (frac > 1.0f)
        {
            frac = 1.0f;
        }
        target_speed_raw = -frac * YAOKONG_TARGET_SPEED_MAX;
    }
    yaokong_speed_target_filt = yaokong_filter_target(yaokong_speed_target_filt,
                                                      target_speed_raw,
                                                      YAOKONG_SPEED_FILTER_ALPHA,
                                                      YAOKONG_SPEED_ZERO_EPS);
    g_yaokong.target_speed_mps = yaokong_speed_target_filt;

    raw_steer = g_yaokong.channel[YAOKONG_CHANNEL_STEER];
    steer_diff = (int32)raw_steer - (int32)YAOKONG_STEER_MID;
    yaokong_steer_axis_active = yaokong_axis_active_update(steer_diff, yaokong_steer_axis_active);

    if (0U == yaokong_steer_axis_active)
    {
        target_steer_raw = 0.0f;
    }
    else if (steer_diff > 0)
    {
        frac = (float)steer_diff / (float)(YAOKONG_STEER_POS_RAW - YAOKONG_STEER_MID);
        if (frac > 1.0f)
        {
            frac = 1.0f;
        }
        target_steer_raw = frac * YAOKONG_TARGET_STEER_MAX_DEG;
    }
    else
    {
        frac = (float)(-steer_diff) / (float)(YAOKONG_STEER_MID - YAOKONG_STEER_NEG_RAW);
        if (frac > 1.0f)
        {
            frac = 1.0f;
        }
        target_steer_raw = -frac * YAOKONG_TARGET_STEER_MAX_DEG;
    }
    target_steer_raw *= YAOKONG_STEER_DIR;
    yaokong_steer_target_filt = yaokong_filter_target(yaokong_steer_target_filt,
                                                      target_steer_raw,
                                                      YAOKONG_STEER_FILTER_ALPHA,
                                                      YAOKONG_STEER_ZERO_EPS_DEG);
    g_yaokong.target_steer_angle_deg = yaokong_steer_target_filt;

    if ((yaokong_absf(g_yaokong.target_speed_mps) <= YAOKONG_REMOTE_ARM_SPEED_EPS) &&
        (yaokong_absf(g_yaokong.target_steer_angle_deg) <= YAOKONG_REMOTE_ARM_STEER_EPS_DEG))
    {
        if (yaokong_remote_neutral_start_us == 0U)
        {
            yaokong_remote_neutral_start_us = now_us;
        }
        else if (yaokong_elapsed_us(now_us, yaokong_remote_neutral_start_us) >= YAOKONG_REMOTE_ARM_NEUTRAL_US)
        {
            yaokong_remote_armed = 1U;
        }
    }
    else
    {
        yaokong_remote_neutral_start_us = 0U;
    }
}

float yaokong_get_target_speed(void)
{
    return g_yaokong.target_speed_mps;
}

float yaokong_get_target_steer_angle(void)
{
    return g_yaokong.target_steer_angle_deg;
}

uint8 yaokong_is_connected(void)
{
    return g_yaokong.connected;
}

uint8 yaokong_is_motor_run_enabled(void)
{
    if (0U == g_yaokong.motor_run_latch)
    {
        return 0U;
    }

    if (0U == g_yaokong.guard_enable)
    {
        return 1U;
    }

    if (0U == g_yaokong.connected)
    {
        return 0U;
    }

    return 1U;
}

void yaokong_remote_entry_reset(void)
{
    yaokong_remote_armed = 0U;
    yaokong_remote_neutral_start_us = 0U;
}

uint8 yaokong_remote_is_armed(void)
{
    return yaokong_remote_armed;
}

uint8 yaokong_ch6_take_press_event(void)
{
    if (yaokong_ch6_press_pending != 0U)
    {
        yaokong_ch6_press_pending = 0U;
        return 1U;
    }

    return 0U;
}

uint32 yaokong_get_last_frame_age_ms(void)
{
    uint32 age_us;
    uint32 last_frame_us;
    uint32 now_us;

    last_frame_us = yaokong_last_frame_us;
    if (last_frame_us == 0U)
    {
        return 0xFFFFFFFFUL;
    }

    now_us = yaokong_now_us();
    age_us = yaokong_elapsed_us(now_us, last_frame_us);
    return age_us / 1000UL;
}

uint32 yaokong_get_valid_frame_count(void)
{
    return yaokong_valid_frame_count;
}

uint32 yaokong_get_bad_frame_count(void)
{
    return yaokong_bad_frame_count;
}

void yaokong_guard_set_enable(uint8 enable)
{
    g_yaokong.guard_enable = (enable != 0U) ? 1U : 0U;
}

uint8 yaokong_guard_get_enable(void)
{
    return g_yaokong.guard_enable;
}
