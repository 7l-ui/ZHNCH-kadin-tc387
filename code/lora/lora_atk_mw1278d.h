#ifndef _LORA_ATK_MW1278D_H_
#define _LORA_ATK_MW1278D_H_

#include "zf_common_headfile.h"

#define LORA_UART_INDEX             UART_5
#define LORA_UART_TX_PIN            UART5_TX_P22_2
#define LORA_UART_RX_PIN            UART5_RX_P33_4
#define LORA_UART_AT_BAUDRATE       (115200U)
#define LORA_UART_DATA_BAUDRATE     (115200U)

#define LORA_DEFAULT_ADDR           (0U)
#define LORA_DEFAULT_CHANNEL        (23U)
#define LORA_RX_BUF_SIZE            (128U)
#define LORA_TX_BUF_SIZE            (128U)
#define LORA_FRAME_GAP_MS           (10U)
#define LORA_AT_TIMEOUT_MS          (500U)
#define LORA_TX_DEMO_ENABLE         (1U)
#define LORA_TX_DEMO_LOG_ENABLE     (0U)
#define LORA_TX_DEMO_PERIOD_MS      (50U)
#define LORA_CONFIG_VERBOSE         (0U)
#define LORA_CONFIG_QUERY_AFTER     (0U)
#define LORA_CONFIG_SAVE_TO_FLASH   (1U)

typedef enum {
    LORA_OK = 0,
    LORA_ERROR = 1,
    LORA_TIMEOUT = 2,
    LORA_PARAM_ERROR = 3,
    LORA_BUSY = 4,
} lora_status_t;

typedef enum {
    LORA_TPOWER_11DBM = 0,
    LORA_TPOWER_14DBM = 1,
    LORA_TPOWER_17DBM = 2,
    LORA_TPOWER_20DBM = 3,
} lora_tpower_t;

typedef enum {
    LORA_WORKMODE_NORMAL = 0,
    LORA_WORKMODE_WAKEUP = 1,
    LORA_WORKMODE_LOWPOWER = 2,
    LORA_WORKMODE_SIGNAL = 3,
} lora_workmode_t;

typedef enum {
    LORA_TMODE_TRANSPARENT = 0,
    LORA_TMODE_DIRECTED = 1,
} lora_tmode_t;

typedef enum {
    LORA_WLRATE_0K3 = 0,
    LORA_WLRATE_1K2 = 1,
    LORA_WLRATE_2K4 = 2,
    LORA_WLRATE_4K8 = 3,
    LORA_WLRATE_9K6 = 4,
    LORA_WLRATE_19K2 = 5,
} lora_wlrate_t;

typedef enum {
    LORA_WLTIME_1S = 0,
    LORA_WLTIME_2S = 1,
} lora_wltime_t;

typedef enum {
    LORA_UARTRATE_1200BPS = 0,
    LORA_UARTRATE_2400BPS = 1,
    LORA_UARTRATE_4800BPS = 2,
    LORA_UARTRATE_9600BPS = 3,
    LORA_UARTRATE_19200BPS = 4,
    LORA_UARTRATE_38400BPS = 5,
    LORA_UARTRATE_57600BPS = 6,
    LORA_UARTRATE_115200BPS = 7,
} lora_uartrate_t;

typedef enum {
    LORA_UARTPARI_NONE = 0,
    LORA_UARTPARI_EVEN = 1,
    LORA_UARTPARI_ODD = 2,
} lora_uartpari_t;

uint8 lora_init(void);
uint8 lora_is_ready(void);
uint8 lora_free(void);
uint8 lora_send_string(const char *str);
uint8 lora_send_buffer(const uint8 *buf, uint16 len);
void lora_uart_rx_isr(void);
void lora_task(void);
void lora_tx_demo_task(uint32 now_ms);

uint8 lora_enter_config(void);
uint8 lora_exit_config(void);
uint8 lora_send_at_cmd(const char *cmd, const char *ack, uint32 timeout_ms);
uint8 lora_config_default(void);
uint8 lora_flash_config(uint8 enable);
uint8 lora_restore_default(void);
uint8 lora_addr_config(uint16 addr);
uint8 lora_wlrate_channel_config(lora_wlrate_t wlrate, uint8 channel);
uint8 lora_tpower_config(lora_tpower_t tpower);
uint8 lora_workmode_config(lora_workmode_t workmode);
uint8 lora_tmode_config(lora_tmode_t tmode);
uint8 lora_wltime_config(lora_wltime_t wltime);
uint8 lora_uart_config(lora_uartrate_t baudrate, lora_uartpari_t parity);

#endif
