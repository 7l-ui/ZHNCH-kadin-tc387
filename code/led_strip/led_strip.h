#ifndef _led_strip_h_
#define _led_strip_h_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define LED_STRIP_PIN                  (P33_11)
#define LED_STRIP_LED_NUM              (30U)
#define LED_STRIP_SAFE_BRIGHTNESS      (22U)

// WS2812/SK6812 usually uses GRB byte order even when the strip is called RGB.
#define LED_STRIP_COLOR_ORDER_GRB      (1U)
// WS2812B RGB uses 24 bits per LED. Set to 1 only for RGBW strips such as SK6812 RGBW.
#define LED_STRIP_RGBW_MODE            (0U)
#define LED_STRIP_FLOW_INTERVAL_MS     (60U)
#define LED_STRIP_FLOW_STEP            (3U)

typedef enum
{
    LED_STRIP_MODE_RAINBOW_STATIC = 0,
    LED_STRIP_MODE_RAINBOW_FLOW
} led_strip_mode_enum;

typedef struct
{
    uint8 r;
    uint8 g;
    uint8 b;
} led_strip_rgb_struct;

void led_strip_init        (void);
void led_strip_clear       (void);
void led_strip_refresh     (void);
void led_strip_set_pixel   (uint16 index, uint8 r, uint8 g, uint8 b);
void led_strip_set_all     (uint8 r, uint8 g, uint8 b);
void led_strip_show_rainbow(uint8 brightness);
void led_strip_set_mode    (led_strip_mode_enum mode);
void led_strip_toggle_mode (void);
led_strip_mode_enum led_strip_get_mode(void);
void led_strip_task        (uint32 now_ms);
void led_strip_show        (const led_strip_rgb_struct *pixels, uint16 count);

#endif
