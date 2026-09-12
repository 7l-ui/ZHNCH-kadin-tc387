#include "lora_atk_mw1278d.h"
#include "nav_control.h"
#include "isr.h"
#include "xl9555_app.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static volatile uint8 g_lora_ready = 0U;
static volatile uint8 g_lora_rx_len = 0U;
static volatile uint8 g_lora_rx_finished = 0U;
static volatile uint32 g_lora_last_rx_ms = 0U;
static uint8 g_lora_rx_buf[LORA_RX_BUF_SIZE];
static uint8 g_lora_frame_buf[LORA_RX_BUF_SIZE];
static char g_lora_tx_buf[LORA_TX_BUF_SIZE];
static char g_lora_last_at_rsp[LORA_RX_BUF_SIZE];

static uint8 lora_config_step(const char *name, uint8 ret)
{
#if LORA_CONFIG_VERBOSE
    printf("[LORA CFG] %s ret=%u, AUX=%u\r\n",
           name,
           (unsigned)ret,
           (unsigned)xl9555_app_lora_get_aux());
#else
    (void)name;
#endif
    return ret;
}

static uint8 lora_query_step(const char *cmd)
{
    uint8 ret;

    ret = lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS);
#if LORA_CONFIG_VERBOSE
    printf("[LORA QUERY] %s ret=%u rsp=%s\r\n", cmd, (unsigned)ret, g_lora_last_at_rsp);
#endif
    return ret;
}

static void lora_save_at_rsp(const uint8 *frame)
{
    uint32 used;
    uint32 left;

    if (frame == NULL)
    {
        return;
    }

    used = (uint32)strlen(g_lora_last_at_rsp);
    if (used >= (sizeof(g_lora_last_at_rsp) - 1U))
    {
        return;
    }

    left = (uint32)sizeof(g_lora_last_at_rsp) - used - 1U;
    strncat(g_lora_last_at_rsp, (const char *)frame, left);
}

static void lora_rx_restart(void)
{
    boolean interrupt_state = disableInterrupts();

    g_lora_rx_len = 0U;
    g_lora_rx_finished = 0U;
    g_lora_last_rx_ms = 0U;

    restoreInterrupts(interrupt_state);
}

static uint8 *lora_rx_get_frame(void)
{
    uint8 len;
    boolean interrupt_state;

    if ((g_lora_rx_finished == 0U) && (g_lora_rx_len > 0U))
    {
        uint32 now_ms = system_getval_ms();

        if ((now_ms - g_lora_last_rx_ms) >= LORA_FRAME_GAP_MS)
        {
            g_lora_rx_finished = 1U;
        }
    }

    if (g_lora_rx_finished == 0U)
    {
        return NULL;
    }

    interrupt_state = disableInterrupts();
    len = g_lora_rx_len;
    if (len >= LORA_RX_BUF_SIZE)
    {
        len = LORA_RX_BUF_SIZE - 1U;
    }
    memcpy(g_lora_frame_buf, g_lora_rx_buf, len);
    g_lora_frame_buf[len] = '\0';
    g_lora_rx_len = 0U;
    g_lora_rx_finished = 0U;
    g_lora_last_rx_ms = 0U;
    restoreInterrupts(interrupt_state);

    return g_lora_frame_buf;
}

static void lora_uart_printf(const char *fmt, ...)
{
    va_list ap;
    int32 len;

    va_start(ap, fmt);
    len = vsnprintf(g_lora_tx_buf, sizeof(g_lora_tx_buf), fmt, ap);
    va_end(ap);

    if (len <= 0)
    {
        return;
    }

    if ((uint32)len >= sizeof(g_lora_tx_buf))
    {
        len = (int32)(sizeof(g_lora_tx_buf) - 1U);
    }

    uart_write_buffer(LORA_UART_INDEX, (const uint8 *)g_lora_tx_buf, (uint32)len);
}

void lora_uart_rx_isr(void)
{
    uint8 dat;

    while (uart_query_byte(LORA_UART_INDEX, &dat))
    {
        if (g_lora_rx_len < (LORA_RX_BUF_SIZE - 1U))
        {
            g_lora_rx_buf[g_lora_rx_len++] = dat;
        }
        else
        {
            g_lora_rx_len = 0U;
            g_lora_rx_buf[g_lora_rx_len++] = dat;
        }

        g_lora_rx_finished = 0U;
        g_lora_last_rx_ms = system_getval_ms();
    }
}

uint8 lora_enter_config(void)
{
    xl9555_app_lora_set_mode(1U);
    system_delay_ms(50U);
    lora_rx_restart();
    return LORA_OK;
}

uint8 lora_exit_config(void)
{
    xl9555_app_lora_set_mode(0U);
    system_delay_ms(50U);
    lora_rx_restart();
    return LORA_OK;
}

uint8 lora_free(void)
{
    return (xl9555_app_lora_get_aux() == 0U) ? LORA_OK : LORA_BUSY;
}

uint8 lora_send_at_cmd(const char *cmd, const char *ack, uint32 timeout_ms)
{
    uint32 start_ms;
    uint8 *frame;

    if (cmd == NULL)
    {
        return LORA_PARAM_ERROR;
    }

    g_lora_last_at_rsp[0] = '\0';
    lora_rx_restart();
    lora_uart_printf("%s\r\n", cmd);

    if ((ack == NULL) || (timeout_ms == 0U))
    {
        return LORA_OK;
    }

    start_ms = system_getval_ms();
    while ((system_getval_ms() - start_ms) < timeout_ms)
    {
        frame = lora_rx_get_frame();
        if (frame != NULL)
        {
            lora_save_at_rsp(frame);
            if (strstr((const char *)frame, ack) != NULL)
            {
                return LORA_OK;
            }
            if (strstr(g_lora_last_at_rsp, ack) != NULL)
            {
                return LORA_OK;
            }
        }
        system_delay_ms(1U);
    }

    return LORA_TIMEOUT;
}

static uint8 lora_at_test(void)
{
    uint8 i;

    for (i = 0U; i < 10U; i++)
    {
        if (lora_send_at_cmd("AT", "OK", LORA_AT_TIMEOUT_MS) == LORA_OK)
        {
            return LORA_OK;
        }
        system_delay_ms(20U);
    }

    return LORA_ERROR;
}

uint8 lora_addr_config(uint16 addr)
{
    char cmd[20];

    snprintf(cmd, sizeof(cmd), "AT+ADDR=%02X,%02X",
             (uint8)((addr >> 8U) & 0xFFU),
             (uint8)(addr & 0xFFU));

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_wlrate_channel_config(lora_wlrate_t wlrate, uint8 channel)
{
    char cmd[20];

    if ((wlrate > LORA_WLRATE_19K2) || (channel > 83U))
    {
        return LORA_PARAM_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+WLRATE=%u,%u", (unsigned)channel, (unsigned)wlrate);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_tpower_config(lora_tpower_t tpower)
{
    char cmd[16];

    if (tpower > LORA_TPOWER_20DBM)
    {
        return LORA_PARAM_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+TPOWER=%u", (unsigned)tpower);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_workmode_config(lora_workmode_t workmode)
{
    char cmd[16];

    if (workmode > LORA_WORKMODE_SIGNAL)
    {
        return LORA_PARAM_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+CWMODE=%u", (unsigned)workmode);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_tmode_config(lora_tmode_t tmode)
{
    char cmd[16];

    if (tmode > LORA_TMODE_DIRECTED)
    {
        return LORA_PARAM_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+TMODE=%u", (unsigned)tmode);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_wltime_config(lora_wltime_t wltime)
{
    char cmd[16];

    if (wltime > LORA_WLTIME_2S)
    {
        return LORA_PARAM_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+WLTIME=%u", (unsigned)wltime);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_uart_config(lora_uartrate_t baudrate, lora_uartpari_t parity)
{
    char cmd[16];

    if ((baudrate > LORA_UARTRATE_115200BPS) || (parity > LORA_UARTPARI_ODD))
    {
        return LORA_PARAM_ERROR;
    }

    snprintf(cmd, sizeof(cmd), "AT+UART=%u,%u", (unsigned)baudrate, (unsigned)parity);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

static void lora_data_uart_init(void)
{
    uart_init(LORA_UART_INDEX, LORA_UART_DATA_BAUDRATE, LORA_UART_TX_PIN, LORA_UART_RX_PIN);
    uart_rx_interrupt(LORA_UART_INDEX, 1U);
}

uint8 lora_flash_config(uint8 enable)
{
    char cmd[16];

    snprintf(cmd, sizeof(cmd), "AT+FLASH=%u", (enable != 0U) ? 1U : 0U);

    return (lora_send_at_cmd(cmd, "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_restore_default(void)
{
    return (lora_send_at_cmd("AT+DEFAULT", "OK", LORA_AT_TIMEOUT_MS) == LORA_OK) ? LORA_OK : LORA_ERROR;
}

uint8 lora_config_default(void)
{
    uint8 ret = 0U;

#if LORA_CONFIG_VERBOSE
    printf("[LORA CFG] target ADDR=00,00 WLRATE=%u,%u TPOWER=3 CWMODE=0 TMODE=0 WLTIME=0 UART=7,0 FLASH=%u\r\n",
           (unsigned)LORA_DEFAULT_CHANNEL,
           (unsigned)LORA_WLRATE_19K2,
           (unsigned)LORA_CONFIG_SAVE_TO_FLASH);
#endif

    ret += lora_config_step("ADDR", lora_addr_config(LORA_DEFAULT_ADDR));
    ret += lora_config_step("WLRATE", lora_wlrate_channel_config(LORA_WLRATE_19K2, LORA_DEFAULT_CHANNEL));
    ret += lora_config_step("TPOWER", lora_tpower_config(LORA_TPOWER_20DBM));
    ret += lora_config_step("CWMODE", lora_workmode_config(LORA_WORKMODE_NORMAL));
    ret += lora_config_step("TMODE", lora_tmode_config(LORA_TMODE_TRANSPARENT));
    ret += lora_config_step("WLTIME", lora_wltime_config(LORA_WLTIME_1S));
    ret += lora_config_step("UART", lora_uart_config(LORA_UARTRATE_115200BPS, LORA_UARTPARI_NONE));
#if LORA_CONFIG_SAVE_TO_FLASH
    ret += lora_config_step("FLASH=1", lora_flash_config(1U));
#else
    ret += lora_config_step("FLASH=0", lora_flash_config(0U));
#endif

#if LORA_CONFIG_QUERY_AFTER
    ret += lora_query_step("AT+MODEL?");
    ret += lora_query_step("AT+ADDR?");
    ret += lora_query_step("AT+WLRATE?");
    ret += lora_query_step("AT+TPOWER?");
    ret += lora_query_step("AT+CWMODE?");
    ret += lora_query_step("AT+TMODE?");
    ret += lora_query_step("AT+WLTIME?");
    ret += lora_query_step("AT+UART?");
#endif

    return (ret == 0U) ? LORA_OK : LORA_ERROR;
}

uint8 lora_init(void)
{
    uint8 ret;

    g_lora_ready = 0U;
    lora_rx_restart();
    uart_init(LORA_UART_INDEX, LORA_UART_AT_BAUDRATE, LORA_UART_TX_PIN, LORA_UART_RX_PIN);
    uart_rx_interrupt(LORA_UART_INDEX, 1U);

    lora_enter_config();
    ret = lora_at_test();
    if (ret == LORA_OK)
    {
        ret = lora_config_default();
    }
    lora_exit_config();

    if (ret == LORA_OK)
    {
        lora_data_uart_init();
        g_lora_ready = 1U;
        printf("[LORA] ATK-MW1278D init OK (UART5 TX=P22_2 RX=P33_4 DATA=115200)\r\n");
    }
    else
    {
        printf("[LORA] ATK-MW1278D init FAIL (ret=%u, AUX=%u)\r\n",
               (unsigned)ret,
               (unsigned)xl9555_app_lora_get_aux());
    }

    return ret;
}

uint8 lora_is_ready(void)
{
    return g_lora_ready;
}

uint8 lora_send_buffer(const uint8 *buf, uint16 len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return LORA_PARAM_ERROR;
    }

    if (g_lora_ready == 0U)
    {
        return LORA_ERROR;
    }

    if (lora_free() == LORA_BUSY)
    {
        return LORA_BUSY;
    }

    lora_data_uart_init();
    uart_write_buffer(LORA_UART_INDEX, buf, len);
    return LORA_OK;
}

uint8 lora_send_string(const char *str)
{
    if (str == NULL)
    {
        return LORA_PARAM_ERROR;
    }

    return lora_send_buffer((const uint8 *)str, (uint16)strlen(str));
}

void lora_task(void)
{
    uint8 *frame;

    if (g_lora_ready == 0U)
    {
        return;
    }

    frame = lora_rx_get_frame();
    if (frame != NULL)
    {
        printf("[LORA RX] %s", frame);
        if (strchr((const char *)frame, '\n') == NULL)
        {
            printf("\r\n");
        }
    }
}

void lora_tx_demo_task(uint32 now_ms)
{
#if LORA_TX_DEMO_ENABLE
    static uint32 last_tx_ms = 0U;
    char msg[64];
    int32 len;
    uint8 ret;
    nav_state_t nav_state;
    float target_speed_mps;
    float real_speed_mps;
    float steer_target_deg;
    float steer_feedback_deg;

    if (g_lora_ready == 0U)
    {
        return;
    }

    if ((last_tx_ms != 0U) && ((now_ms - last_tx_ms) < LORA_TX_DEMO_PERIOD_MS))
    {
        return;
    }

    last_tx_ms = now_ms;
    nav_control_get_state(&nav_state);
    target_speed_mps = g_pid_used_target_speed_mps;
    real_speed_mps = nav_state.speed_mps;
    steer_target_deg = g_steer_target_angle_deg;
    steer_feedback_deg = g_steer_feedback_angle_deg;
    len = snprintf(msg, sizeof(msg), "samples:%.3f,%.3f,%.2f,%.2f\r\n",
                   (double)target_speed_mps,
                   (double)real_speed_mps,
                   (double)steer_target_deg,
                   (double)steer_feedback_deg);
    if (len > 0)
    {
        ret = lora_send_string(msg);
#if LORA_TX_DEMO_LOG_ENABLE
        printf("[LORA VOFA] ret=%u, AUX=%u, target=%.3f real=%.3f steer_t=%.2f steer_f=%.2f\r\n",
               (unsigned)ret,
               (unsigned)xl9555_app_lora_get_aux(),
               (double)target_speed_mps,
               (double)real_speed_mps,
               (double)steer_target_deg,
               (double)steer_feedback_deg);
#else
        (void)ret;
#endif
    }
#else
    (void)now_ms;
#endif
}
