#ifndef _XL9555_APP_H_
#define _XL9555_APP_H_

#include "xl9555.h"
#include "zf_common_headfile.h"

// IO0 group (input): keys/switches
#define XL9555_PIN_IO0_0     6   // KEY1
#define XL9555_PIN_IO0_1     5   // KEY2
#define XL9555_PIN_IO0_2     4   // KEY3
#define XL9555_PIN_IO0_3     3   // KEY4
#define XL9555_PIN_IO0_4     2   // KEY5
#define XL9555_PIN_IO0_5     1   // SWITCH2
#define XL9555_PIN_IO0_6     0   // SWITCH1
#define XL9555_PIN_IO0_7     7   // reserved

// IO1 group: LED/LoRa control/status
#define XL9555_PIN_IO1_0     8   // ENLED
#define XL9555_PIN_IO1_1     9   // RUNLED
#define XL9555_PIN_IO1_2     10  // LORA_AUX input
#define XL9555_PIN_IO1_3     11  // LORA_MOD

// Keys
#define XL9555_KEY_1         XL9555_PIN_IO0_0
#define XL9555_KEY_2         XL9555_PIN_IO0_1
#define XL9555_KEY_3         XL9555_PIN_IO0_2
#define XL9555_KEY_4         XL9555_PIN_IO0_3
#define XL9555_KEY_5         XL9555_PIN_IO0_4

// Switches
#define XL9555_SWITCH_1      XL9555_PIN_IO0_6
#define XL9555_SWITCH_2      XL9555_PIN_IO0_5

// LEDs
#define XL9555_LED_ENABLE    XL9555_PIN_IO1_0
#define XL9555_LED_RUN       XL9555_PIN_IO1_1

// LoRa control
#define XL9555_LORA_AUX      XL9555_PIN_IO1_2
#define XL9555_LORA_MOD      XL9555_PIN_IO1_3

// Logic levels
#define XL9555_KEY_PRESSED   0
#define XL9555_KEY_RELEASED  1
#define XL9555_SWITCH_ON     0
#define XL9555_SWITCH_OFF    1
#define XL9555_LED_ON        1
#define XL9555_LED_OFF       0
#define XL9555_LORA_MODE_0   0
#define XL9555_LORA_MODE_1   1

typedef struct {
    uint8 key1 : 1;
    uint8 key2 : 1;
    uint8 key3 : 1;
    uint8 key4 : 1;
    uint8 key5 : 1;
    uint8 switch1 : 1;
    uint8 switch2 : 1;
    uint8 reserved : 1;
} xl9555_input_status_t;

typedef struct {
    uint8 led_enable : 1;
    uint8 led_run : 1;
    uint8 lora_aux : 1;
    uint8 lora_mode : 1;
    uint8 reserved : 4;
} xl9555_output_control_t;

#ifdef __cplusplus
extern "C" {
#endif

void xl9555_app_diagnose(void);
void xl9555_app_init(void);
void* xl9555_app_get_device(void);
uint8 xl9555_app_is_ready(void);
uint8 xl9555_app_get_addr(void);

uint8 xl9555_app_get_key(uint8 key_num);
uint8 xl9555_app_get_switch(uint8 switch_num);
void xl9555_app_get_all_inputs(xl9555_input_status_t *status);
uint8 xl9555_app_scan_keys(uint8 *key_pressed);

void xl9555_app_led_enable(uint8 state);
void xl9555_app_led_run(uint8 state);
void xl9555_app_led_enable_toggle(void);
void xl9555_app_led_run_toggle(void);

uint8 xl9555_app_lora_get_aux(void);
void xl9555_app_lora_set_aux(uint8 state);
void xl9555_app_lora_set_mode(uint8 mode);
void xl9555_app_lora_toggle_mode(void);

void xl9555_app_set_all_outputs(xl9555_output_control_t control);
void xl9555_app_get_all_outputs(xl9555_output_control_t *control);

void xl9555_app_configure_pin_directions(void);
void xl9555_app_configure_polarity(void);
void xl9555_app_read_all_gpio(uint16 *levels);
void xl9555_app_write_all_gpio(uint16 levels);

// Optional test API (disabled by default in xl9555_app.c)
void xl9555_app_test(void);

#ifdef __cplusplus
}
#endif

#endif // _XL9555_APP_H_
