#include "zf_common_headfile.h"
#include "Key.h"
#include "UI.h"
#include "camera_debug_link.h"
#include "mag_cal_logger.h"
#include "sd_simple.h"
#include "data_logger.h"
#include "taban.h"
#include "xl9555_app.h"
#include "zf_device_absolute_encoder.h"
#include "zf_device_imu963ra.h"
#include "zf_device_gnss.h"
#include "dici.h"
#include "pid.h"
#include "pid_config.h"
#include "dianji.h"
#include "isr.h"
#include "yaokong.h"
#include "task_ctrl.h"
#include "nav_control.h"
#include "lora/lora_atk_mw1278d.h"
#include "guimai/guimai_board.h"
#include "tld7002_project_config.h"
#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
#include "zf_device_dot_matrix_screen.h"
#include "zf_device_tld7002.h"
#endif

#pragma section all "cpu0_dsram"

volatile int16 g_ecod1_count = 0;
volatile int16 g_ecod2_count = 0;
volatile int16 g_ecod1_speed = 0;
volatile int16 g_ecod2_speed = 0;
volatile int16 g_ecod1_speed_filt = 0;
volatile int16 g_ecod2_speed_filt = 0;

#define PID_FIXED_TARGET_DEFAULT_MPS     (0.35f)
#define STEER_FIXED_TARGET_DEFAULT_DEG   (0.0f)
#define ABS_ENC_COUNTS_PER_REV           (4096)
#define STEER_CENTER_RAW_DEFAULT         (3821)
#define ENABLE_WIFI_SPI                  (0U)
#define ENABLE_MT6701_I2C                (0U)
#define ENABLE_STARTUP_I2C_DEBUG         (0U)
#define ENABLE_PERIODIC_SENSOR_LOG       (0U)
#define ENABLE_LOGGER_DIAG_LOG           (0U)
#define ENABLE_MAG_CAL_SD_LOG            (0U)
#define MAG_CAL_AUTO_STOP_MS             (30000U)
#define SD_APP_MODE_DEFAULT              ((ENABLE_MAG_CAL_SD_LOG != 0U) ? SD_APP_MODE_MAG_CAL : SD_APP_MODE_LOGGER)

static const float g_mag_cal_offset[3] =
{
    0.022333500f,
    -0.020666500f,
    0.016266900f
};

static const float g_mag_cal_softiron[3][3] =
{
    {1.012404658f, 0.0f,         0.0f},
    {0.0f,         1.017641236f, 0.0f},
    {0.0f,         0.0f,         0.971262210f}
};

volatile uint8 g_abs_encoder_ready = 0U;
volatile uint8 g_imu963ra_init_ret = 0xFFU;
static uint8 g_sd_ready = 0U;
static sd_app_mode_t g_sd_app_mode = SD_APP_MODE_DEFAULT;
volatile int16 g_abs_encoder_raw = 0;
volatile int16 g_steer_center_raw = STEER_CENTER_RAW_DEFAULT;
volatile int16 g_steer_delta_counts = 0;

void Ctrl_SetSteerCenterRaw(int16 center_raw)
{
    while (center_raw < 0)
    {
        center_raw += ABS_ENC_COUNTS_PER_REV;
    }
    while (center_raw >= ABS_ENC_COUNTS_PER_REV)
    {
        center_raw -= ABS_ENC_COUNTS_PER_REV;
    }

    g_steer_center_raw = center_raw;
}

void Ctrl_CaptureSteerCenterNow(void)
{
    Ctrl_SetSteerCenterRaw(g_abs_encoder_raw);
}

int16 Ctrl_GetSteerCenterRaw(void)
{
    return g_steer_center_raw;
}

int16 Ctrl_GetSteerDeltaCounts(void)
{
    return g_steer_delta_counts;
}

static const char *Ctrl_TargetSrcName(uint8 target_src)
{
    switch (target_src)
    {
        case TARGET_SRC_TASK:
            return "TASK";
        case TARGET_SRC_PEDAL:
            return "PEDAL";
        case TARGET_SRC_REMOTE:
            return "REMOTE";
        case TARGET_SRC_PWM_TEST:
            return "PWMTEST";
        case TARGET_SRC_STEER_SCAN:
            return "SCAN";
        default:
            return "UNKNOWN";
    }
}

#define VOICE_CMD_REVERSE        (2U)
#define VOICE_CMD_DOUBLE_FLASH   (3U)
#define VOICE_CMD_LEFT_TURN      (4U)
#define VOICE_CMD_RIGHT_TURN     (5U)
#define VOICE_CMD_LOW_BEAM       (6U)
#define VOICE_CMD_HIGH_BEAM      (7U)
#define VOICE_CMD_FOG_LIGHT      (8U)
#define VOICE_CMD_INTERIOR_LIGHT (30U)
#define VOICE_CMD_WIPER          (31U)

static uint8 Guimai_VoiceCommandNeedsOutputGap(uint8 cmd)
{
    switch (cmd)
    {
        case VOICE_CMD_DOUBLE_FLASH:
        case VOICE_CMD_LEFT_TURN:
        case VOICE_CMD_RIGHT_TURN:
        case VOICE_CMD_LOW_BEAM:
        case VOICE_CMD_HIGH_BEAM:
        case VOICE_CMD_FOG_LIGHT:
        case VOICE_CMD_INTERIOR_LIGHT:
        case VOICE_CMD_WIPER:
        case GUIMAI_CMD_HORN_1S:
        case GUIMAI_CMD_HORN_2S:
        case GUIMAI_CMD_HORN_3S:
        case GUIMAI_CMD_HORN_2_BEEP:
        case GUIMAI_CMD_HORN_3_BEEP:
        case GUIMAI_CMD_HORN_4_BEEP:
        case GUIMAI_CMD_HORN_SHORT_LONG:
        case GUIMAI_CMD_HORN_RAPID:
        case GUIMAI_CMD_HORN_ALARM:
            return 1U;
        default:
            return 0U;
    }
}

#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
static uint8 Guimai_DotMatrixHandleCommand(uint8 cmd)
{
    uint8 handled = 1U;
    dot_matrix_screen_pattern_enum pattern = DOT_MATRIX_SCREEN_PATTERN_BLANK;

    switch (cmd)
    {
        case VOICE_CMD_REVERSE:
            pattern = DOT_MATRIX_SCREEN_PATTERN_REVERSE;
            break;
        case VOICE_CMD_DOUBLE_FLASH:
            pattern = DOT_MATRIX_SCREEN_PATTERN_DOUBLE_FLASH;
            break;
        case VOICE_CMD_LEFT_TURN:
            pattern = DOT_MATRIX_SCREEN_PATTERN_LEFT_TURN;
            break;
        case VOICE_CMD_RIGHT_TURN:
            pattern = DOT_MATRIX_SCREEN_PATTERN_RIGHT_TURN;
            break;
        case VOICE_CMD_LOW_BEAM:
            pattern = DOT_MATRIX_SCREEN_PATTERN_LOW_BEAM;
            break;
        case VOICE_CMD_HIGH_BEAM:
            pattern = DOT_MATRIX_SCREEN_PATTERN_HIGH_BEAM;
            break;
        case VOICE_CMD_FOG_LIGHT:
            pattern = DOT_MATRIX_SCREEN_PATTERN_FOG_LIGHT;
            break;
        case VOICE_CMD_INTERIOR_LIGHT:
            pattern = DOT_MATRIX_SCREEN_PATTERN_INTERIOR_LIGHT;
            break;
        case VOICE_CMD_WIPER:
            pattern = DOT_MATRIX_SCREEN_PATTERN_WIPER;
            break;
        case GUIMAI_CMD_HORN_1S:
        case GUIMAI_CMD_HORN_2S:
        case GUIMAI_CMD_HORN_3S:
        case GUIMAI_CMD_HORN_2_BEEP:
        case GUIMAI_CMD_HORN_3_BEEP:
        case GUIMAI_CMD_HORN_4_BEEP:
        case GUIMAI_CMD_HORN_SHORT_LONG:
        case GUIMAI_CMD_HORN_RAPID:
        case GUIMAI_CMD_HORN_ALARM:
            handled = 0U;
            break;
        default:
            handled = 0U;
            break;
    }

    if (handled)
    {
        dot_matrix_screen_show_pattern(pattern);
        printf("[GUIMAI_LED] cmd=%u pattern=%u\r\n", (unsigned)cmd, (unsigned)pattern);
    }
    else
    {
        printf("[GUIMAI_LED] cmd=%u no dot pattern\r\n", (unsigned)cmd);
    }

    return handled;
}

#endif

static uint8 Guimai_VoiceOutputHandleCommand(uint8 cmd)
{
    uint8 handled = 0U;

#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
    if (0U != Guimai_DotMatrixHandleCommand(cmd))
    {
        handled = 1U;
    }
#endif

    if (0U != guimai_voice_handle_horn_command(cmd))
    {
        handled = 1U;
    }

    return handled;
}

static uint8 Guimai_VoiceOutputTryRun(uint8 cmd, uint32 *last_output_ms)
{
    uint8 output_gap_cmd = Guimai_VoiceCommandNeedsOutputGap(cmd);
    uint8 output_handled = 0U;
    uint32 now_ms = 0U;

    if ((0U != output_gap_cmd) &&
        ((uint32)GUIMAI_VOICE_LIGHT_HORN_GAP_MS > 0U) &&
        (last_output_ms != NULL))
    {
        now_ms = system_getval_ms();
        if ((*last_output_ms != 0U) &&
            ((uint32)(now_ms - *last_output_ms) < (uint32)GUIMAI_VOICE_LIGHT_HORN_GAP_MS))
        {
            return 0U;
        }
    }

    output_handled = Guimai_VoiceOutputHandleCommand(cmd);
    if ((0U != output_gap_cmd) &&
        (0U != output_handled) &&
        (last_output_ms != NULL))
    {
        *last_output_ms = system_getval_ms();
    }

    return 1U;
}

static void Guimai_VoiceCommandTask(void)
{
    static uint8 pending_valid = 0U;
    static uint8 pending_cmd = 0U;
    static uint8 pending_task2_output_valid = 0U;
    static uint8 pending_task2_output_cmd = 0U;
    static uint32 last_output_ms = 0U;
    uint8 cmd = 0U;

    while (TRUE)
    {
        if (pending_task2_output_valid == 0U)
        {
            if (0U == ctrl_task2_output_pop(&pending_task2_output_cmd))
            {
                break;
            }
            pending_task2_output_valid = 1U;
        }

        if (0U == Guimai_VoiceOutputTryRun(pending_task2_output_cmd, &last_output_ms))
        {
            break;
        }

        pending_task2_output_valid = 0U;
    }

    while (TRUE)
    {
        if (pending_valid)
        {
            cmd = pending_cmd;
        }
        else if (0U == guimai_voice_get_command(&cmd))
        {
            break;
        }

        pending_valid = 0U;
        if (0U == ctrl_task2_handle_voice_command(cmd))
        {
            if (0U == Guimai_VoiceOutputTryRun(cmd, &last_output_ms))
            {
                pending_cmd = cmd;
                pending_valid = 1U;
                break;
            }
        }
    }
}

int core0_main(void)
{
    disable_Watchdog();         // disable watchdog
    interrupt_global_enable(0); // enable global interrupt
    clock_init();
    debug_init();

    system_delay_ms(50);   // wait power-up stable

    Key_Init();
    printf("[INIT] XL9555: %s (addr=0x%02X)\r\n",
           xl9555_app_is_ready() ? "OK" : "FAIL",
           xl9555_app_get_addr());
    (void)lora_init();

#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
    dot_matrix_screen_init();
    {
        const tld7002_diag_result_struct *tld_diag = tld7002_diag_get_result();
        printf("[TLD7002_DIAG] code=0x%08lx %s otp=%u init=0x%04x duty=0x%04x vled_err=0x%04x vled=%umV dts_err=0x%04x dts=%uC\r\n",
               (unsigned long)tld_diag->code,
               tld7002_diag_code_string(tld_diag->code),
               tld_diag->otp_emu_ok,
               tld_diag->init_err,
               tld_diag->duty_err,
               tld_diag->vled_err,
               tld_diag->vled_mv,
               tld_diag->dts_err,
               tld_diag->dts_c);
        if(TLD7002_DIAG_OK == tld_diag->code)
        {
            printf("[INIT] TLD7002 diag passed, dot matrix normal mode.\r\n");
        }
        else
        {
            printf("[INIT] TLD7002 diag failed, keep dot matrix normal mode.\r\n");
        }
    }
#endif

    #if ENABLE_WIFI_SPI
    {
        uint8_t wifi_ready = 0U;

        system_delay_ms(300);

        for (uint8_t attempt = 1U; attempt <= 2U; attempt++)
        {
            uint8_t wifi_hw_ret = wifi_spi_init(NULL, NULL);
            uint8_t int_level = gpio_get_level(WIFI_SPI_INT_PIN);

            if ((wifi_hw_ret == 0U) && (wifi_spi_version[0] != '\0') && (wifi_spi_mac_addr[0] != '\0'))
            {
                printf("[WIFI_SPI] hw init success(attempt=%u), INT=%u, ver=%s, mac=%s\r\n",
                       attempt, int_level, wifi_spi_version, wifi_spi_mac_addr);
                wifi_ready = 1U;
                break;
            }
            else
            {
                printf("[WIFI_SPI] hw init invalid(attempt=%u), ret=%u, INT=%u, ver=%s, mac=%s\r\n",
                       attempt, wifi_hw_ret, int_level, wifi_spi_version, wifi_spi_mac_addr);
                system_delay_ms(300);
            }
        }

        if (wifi_ready)
        {
            uint8_t wifi_join_ret = wifi_spi_wifi_connect("LTL", "3170851279");
            if (wifi_join_ret == 0U)
            {
                printf("[WIFI_SPI] wifi join success, ip=%s\r\n", wifi_spi_ip_addr_port);

                uint8_t wifi_conn_ret = wifi_spi_socket_connect("TCP", "192.168.87.163", "8086", "6666");
                if (wifi_conn_ret == 0U)
                {
                    printf("[WIFI_SPI] tcp connect success, ip=%s\r\n", wifi_spi_ip_addr_port);
                }
                else
                {
                    printf("[WIFI_SPI] tcp connect failed, ret=%u, ip=%s\r\n", wifi_conn_ret, wifi_spi_ip_addr_port);
                }
            }
            else
            {
                printf("[WIFI_SPI] wifi join failed, ret=%u\r\n", wifi_join_ret);
            }
        }
        else
        {
            printf("[WIFI_SPI] skip join/connect because module response is invalid.\r\n");
        }
    }
    #endif


    {
        g_imu963ra_init_ret = IMU_Sensor_Init();
        printf("[INIT] IMU963RA: %s (ret=%u)\r\n", (g_imu963ra_init_ret == 0U) ? "OK" : "FAIL", g_imu963ra_init_ret);

        if (g_imu963ra_init_ret == 0U)
        {
            EKF_Init();
#if IMU_AHRS_USE_MAG
            IMU_AHRS_SetMagCalibrationMatrix(g_mag_cal_offset, g_mag_cal_softiron);
            IMU_AHRS_SetMagEnable(1U);
            printf("[INIT] AHRS mode=9AXIS, INS uses relative yaw, mag cal MAG010\r\n");
#else
            IMU_AHRS_SetMagEnable(0U);
            printf("[INIT] AHRS mode=6AXIS, INS uses relative yaw, mag disabled\r\n");
#endif
        }
    }

    gnss_init(TAU1201);
    nav_control_init();
    printf("[INIT] GPS: %s\r\n", gnss_is_ready() ? "OK" : "FAIL");

    UI_Init();
    taban_init();
#if YAOKONG_REMOTE_ENABLE
    yaokong_init();
#endif

    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
    encoder_clear_count(TIM2_ENCODER);
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P33_5);
    encoder_clear_count(TIM4_ENCODER);
    printf("[ENC] init done: ECOD1(T2 P33_7/P33_6), ECOD2(T4 P02_8/P33_5)\r\n");

    // ABS encoder: SPI4 P22_3/P22_0/P22_1, CS=P23_1 GPIO.
    {
        uint8_t abs_ret = absolute_encoder_init();
        uint8_t abs_err_code = absolute_encoder_get_last_error_code();
        uint8_t abs_err_reg = absolute_encoder_get_last_error_reg();
        uint8_t abs_err_expect = absolute_encoder_get_last_expect();
        uint8_t abs_err_actual = absolute_encoder_get_last_actual();
        uint8_t abs_attempt = absolute_encoder_get_last_init_attempt();
        g_abs_encoder_ready = (abs_ret == 0U) ? 1U : 0U;
        if(abs_ret == 0U)
        {
            printf("[INIT] ABS ENCODER: OK (ret=%u attempt=%u, SPI4 SCK=P22_3 MOSI=P22_0 MISO=P22_1 CS=P23_1 GPIO, speed=10MHz)\r\n", abs_ret, abs_attempt);
        }
        else
        {
            printf("[INIT] ABS ENCODER: FAIL (ret=%u attempt=%u code=%u reg=%u expect=%u/0x%02X actual=%u/0x%02X, SPI4 SCK=P22_3 MOSI=P22_0 MISO=P22_1 CS=P23_1 GPIO, speed=10MHz)\r\n",
                   abs_ret,
                   abs_attempt,
                   abs_err_code,
                   abs_err_reg,
                   abs_err_expect,
                   abs_err_expect,
                   abs_err_actual,
                   abs_err_actual);
        }
    }

    dianji_init();
    pid_init();
    pid_reset_all();
    g_pid_fixed_target_speed_mps = PID_FIXED_TARGET_DEFAULT_MPS;
    g_steer_fixed_target_angle_deg = STEER_FIXED_TARGET_DEFAULT_DEG;
    g_pid_target_src = TARGET_SRC_TASK;
    g_pid_target_use_pedal = 0U;
    dianji_request_enable(1U);
    printf("[CTRL] PID ready, task0_target=%.2f m/s, mode=%s\r\n",
           g_pid_fixed_target_speed_mps,
           Ctrl_TargetSrcName(g_pid_target_src));

    guimai_board_init();

    if ((g_sd_app_mode == SD_APP_MODE_LOGGER) || (g_sd_app_mode == SD_APP_MODE_MAG_CAL))
    {
        g_sd_ready = 1U;
        if (g_sd_app_mode == SD_APP_MODE_MAG_CAL)
        {
            printf("[SD] app mode=%u (0=none 2=logger 3=mag_cal), CPU1 MAG worker will check SD and retry until ready.\r\n",
                   (unsigned)g_sd_app_mode);
        }
        else
        {
            printf("[SD] app mode=%u (0=none 2=logger 3=mag_cal), probe/init deferred to CPU1 logger.\r\n",
                   (unsigned)g_sd_app_mode);
        }
    }
    else
    {
        uint8_t sd_ok = sd_simple_probe();
        g_sd_ready = (sd_ok == 0U) ? 1U : 0U;
        if (sd_ok == 0U)
        {
            printf("[SD] card available.\r\n");
            printf("[SD] app mode=%u (0=none 2=logger 3=mag_cal)\r\n", (unsigned)g_sd_app_mode);
        }
        else
        {
            printf("[SD] card unavailable, probe code=%u\r\n", sd_ok);
        }
    }

#if ENABLE_STARTUP_I2C_DEBUG
    printf("[DEBUG] Testing P13_3/P13_2 GPIO pins...\r\n");
    {
        gpio_init(P13_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
        gpio_init(P13_2, GPI, 0, GPI_PULL_UP);

        system_delay_ms(10);
        uint8_t sda_high = gpio_get_level(P13_2);
        gpio_set_level(P13_3, GPIO_LOW);
        system_delay_ms(10);
        uint8_t scl_low = gpio_get_level(P13_3);

        printf("[DEBUG] P13_3(SCL) can drive low: %s\r\n", scl_low == 0 ? "YES" : "NO");
        printf("[DEBUG] P13_2(SDA) pull-up works: %s\r\n", sda_high == 1 ? "YES" : "NO");
    }

    printf("[DEBUG] Scanning I2C bus on P13_3/P13_2...\r\n");
    {
        soft_iic_info_struct scan_iic;
        soft_iic_init(&scan_iic, 0x00, 100, P13_3, P13_2);

        uint8_t found_count = 0;
        for (uint8_t addr = 0x01; addr <= 0x7F; addr++)
        {
            if (soft_iic_probe_addr(&scan_iic, addr))
            {
                printf("[DEBUG] Found I2C device at address 0x%02X\r\n", addr);
                found_count++;
            }
        }

        if (found_count == 0U)
        {
            printf("[DEBUG] No I2C devices found on P13_3/P13_2 bus\r\n");
        }
        else
        {
            printf("[DEBUG] Total %u I2C device(s) found\r\n", found_count);
        }
    }
#endif

    if ((g_sd_ready != 0U) && (g_sd_app_mode == SD_APP_MODE_LOGGER))
    {
        printf("[INIT] DATA LOGGER ready, waits for task/manual start.\r\n");
    }
    else if ((g_sd_ready != 0U) && (g_sd_app_mode == SD_APP_MODE_MAG_CAL))
    {
        mag_cal_logger_set_auto_stop_ms(MAG_CAL_AUTO_STOP_MS);
        uint8_t mag_cal_ret = mag_cal_logger_start();
        printf("[INIT] MAG CAL logger start ret=%u, auto_stop=%lu ms, rotate device in all directions.\r\n",
               (unsigned)mag_cal_ret,
               (unsigned long)MAG_CAL_AUTO_STOP_MS);
    }
    else
    {
        printf("[INIT] SD app disabled or SD unavailable.\r\n");
    }

    ctrl_cpu0_progress(CTRL_CPU0_STAGE_BOOT);
    pit_ms_init(CCU60_CH0, 1);
    printf("[INIT] PIT timer started (1ms for IMU)\r\n");
    cpu_wait_event_ready();
    while (TRUE)
    {
        static uint32 last_enc_print_ms = 0;
        static uint32 logger_diag_ms = 0;
        static uint8 logger_diag_done = 0U;
        static uint8 last_logger_state = 0U;
        static uint8 last_target_src = 0xFFU;
        static uint32 remote_logger_retry_ms = 0U;
        uint32 now_ms = system_getval_ms();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_CAMERA_LINK);
        camera_debug_link_task();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_KEYS);
        Key_scan();
        Key_Active();

        ctrl_cpu0_progress(CTRL_CPU0_STAGE_GPS);
        (void)nav_control_poll_gps(now_ms);
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_VOICE_BOARD);
        guimai_board_task();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_VOICE_COMMAND);
        Guimai_VoiceCommandTask();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_LORA);
        lora_task();
        lora_tx_demo_task(now_ms);
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_PEDAL);
        taban_task();
#if YAOKONG_REMOTE_ENABLE
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_REMOTE);
        yaokong_update_speed();
        if (yaokong_ch6_take_press_event() != 0U)
        {
            if ((ui_task_get() == 0U) &&
                (g_pid_target_src == TARGET_SRC_REMOTE))
            {
                printf("[REMOTE_TEST] CH6 logged; task3 action suppressed\r\n");
            }
            else
            {
                ctrl_task3_remote_ch6_request();
            }
        }
#endif
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_INS);
        (void)nav_control_update_ins_from_imu(now_ms,
                                              g_ecod1_count,
                                              g_ecod2_count);

        g_pid_pedal_target_speed_mps = taban_get_target_speed_mps();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_CONTROL);
        ctrl_mode_update();
        if (last_target_src != g_pid_target_src)
        {
            last_target_src = g_pid_target_src;
            printf("[CTRL] target source -> %s\r\n", Ctrl_TargetSrcName(g_pid_target_src));
        }
        if ((ui_task_get() == 0U) &&
            (g_pid_target_src == TARGET_SRC_REMOTE) &&
            ((data_logger_get_task_id() != 0U) ||
             (data_logger_get_profile() != (uint8)DATA_LOGGER_PROFILE_DRIVE_DEBUG) ||
             (data_logger_get_target_src() != TARGET_SRC_REMOTE)) &&
            ((remote_logger_retry_ms == 0U) ||
             ((uint32)(now_ms - remote_logger_retry_ms) >= 200U)))
        {
            remote_logger_retry_ms = now_ms;
            (void)data_logger_restart_task_src(0U,
                                                DATA_LOGGER_PROFILE_DRIVE_DEBUG,
                                                TARGET_SRC_REMOTE);
        }
        else if (g_pid_target_src != TARGET_SRC_REMOTE)
        {
            remote_logger_retry_ms = 0U;
        }
#if ENABLE_PERIODIC_SENSOR_LOG
        if ((now_ms - last_enc_print_ms) >= 1000)
        {
            last_enc_print_ms = now_ms;
            if (g_abs_encoder_ready != 0U)
            {
                printf("[ABS_ENC] raw=%d center=%d delta=%d steer=%.2f deg\r\n",
                       (int)g_abs_encoder_raw,
                       (int)g_steer_center_raw,
                       (int)g_steer_delta_counts,
                       (double)g_steer_feedback_angle_deg);
            }
            else
            {
                printf("[ABS_ENC] not ready\r\n");
            }
            printf("[IMU] Pitch=%.2f Roll=%.2f Yaw=%.2f Gyro_Z=%.2f\r\n",
                   IMU_Pitch, IMU_Roll, IMU_Yaw, IMU_GYRO_Z);
        }
#else
        (void)last_enc_print_ms;
#endif

        if (g_sd_app_mode == SD_APP_MODE_LOGGER)
        {
            uint8 logger_state = data_logger_get_state();

            ctrl_cpu0_progress(CTRL_CPU0_STAGE_LOGGER);
            data_logger_task(now_ms);
            if (logger_state != last_logger_state)
            {
                last_logger_state = logger_state;
                logger_diag_ms = 0U;
                logger_diag_done = 0U;
            }
#if ENABLE_LOGGER_DIAG_LOG
            if ((logger_state != 0U) && (logger_diag_ms == 0U))
            {
                logger_diag_ms = now_ms;
            }
            else if ((logger_state != 0U) &&
                     (logger_diag_done == 0U) &&
                     ((now_ms - logger_diag_ms) >= 2000U))
            {
                logger_diag_done = 1U;
                printf("[LOGGER] diag: state=%u err=%u dropped=%u file=%s\r\n",
                       (unsigned)data_logger_get_state(),
                       (unsigned)data_logger_get_last_error(),
                       (unsigned)data_logger_get_dropped_count(),
                       data_logger_get_filename());
            }
#else
            (void)logger_diag_ms;
            (void)logger_diag_done;
#endif
        }
        else if (g_sd_app_mode == SD_APP_MODE_MAG_CAL)
        {
            static uint8 last_mag_cal_state = 0xFFU;
            uint8 mag_cal_state = mag_cal_logger_get_state();

            if (mag_cal_state != last_mag_cal_state)
            {
                last_mag_cal_state = mag_cal_state;
                printf("[MAG_CAL] state=%u err=%u file=%s\r\n",
                       (unsigned)mag_cal_state,
                       (unsigned)mag_cal_logger_get_last_error(),
                       mag_cal_logger_get_filename());
            }
        }

        ctrl_cpu0_progress(CTRL_CPU0_STAGE_UI);
        UI();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_BEEP);
        Beep_Deal();
        ctrl_cpu0_progress(CTRL_CPU0_STAGE_DELAY);
        system_delay_ms(10);
    }
}

#pragma section all restore
