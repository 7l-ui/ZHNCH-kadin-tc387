#include "IfxCpu.h"
#include "IfxStm.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#include "led_strip.h"

#define LED_STRIP_T0H_TICKS            (5U)        // CPU3 GPIO/STM access adds extra high time
#define LED_STRIP_T0L_TICKS            (100U)
#define LED_STRIP_T1H_TICKS            (55U)
#define LED_STRIP_T1L_TICKS            (45U)
#define LED_STRIP_RESET_US             (80U)

static led_strip_rgb_struct led_strip_buffer[LED_STRIP_LED_NUM];
static Ifx_P *led_strip_port;
static Ifx_STM *led_strip_stm;
static uint32 led_strip_high_mask;
static uint32 led_strip_low_mask;
static led_strip_mode_enum led_strip_mode = LED_STRIP_MODE_RAINBOW_STATIC;
static volatile led_strip_mode_enum led_strip_mode_request = LED_STRIP_MODE_RAINBOW_STATIC;
static volatile uint8 led_strip_mode_update_pending;
static uint8 led_strip_rainbow_offset;
static uint32 led_strip_last_flow_ms;

#define led_strip_wait_ticks(ticks) \
    do { \
        uint32 begin = led_strip_stm->TIM0.U; \
        while ((uint32)(led_strip_stm->TIM0.U - begin) < (uint32)(ticks)) { } \
    } while (0)

#define led_strip_data_high()          (led_strip_port->OMR.U = led_strip_high_mask)
#define led_strip_data_low()           (led_strip_port->OMR.U = led_strip_low_mask)

static void led_strip_write_bit(uint8 bit)
{
    if (bit)
    {
        led_strip_data_high();
        led_strip_wait_ticks(LED_STRIP_T1H_TICKS);
        led_strip_data_low();
        led_strip_wait_ticks(LED_STRIP_T1L_TICKS);
    }
    else
    {
        led_strip_data_high();
        led_strip_wait_ticks(LED_STRIP_T0H_TICKS);
        led_strip_data_low();
        led_strip_wait_ticks(LED_STRIP_T0L_TICKS);
    }
}

static void led_strip_write_byte(uint8 data)
{
    uint8 mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        led_strip_write_bit((data & mask) ? 1U : 0U);
    }
}

static void led_strip_write_color(uint8 r, uint8 g, uint8 b)
{
#if LED_STRIP_COLOR_ORDER_GRB
    led_strip_write_byte(g);
    led_strip_write_byte(r);
    led_strip_write_byte(b);
#else
    led_strip_write_byte(r);
    led_strip_write_byte(g);
    led_strip_write_byte(b);
#endif

#if LED_STRIP_RGBW_MODE
    led_strip_write_byte(0U);
#endif
}

static uint8 led_strip_scale(uint8 value, uint8 brightness)
{
    return (uint8)(((uint16)value * (uint16)brightness) / 255U);
}

static led_strip_rgb_struct led_strip_color_wheel(uint8 pos, uint8 brightness)
{
    led_strip_rgb_struct color;

    if (pos < 85U)
    {
        color.r = led_strip_scale((uint8)(255U - pos * 3U), brightness);
        color.g = led_strip_scale((uint8)(pos * 3U), brightness);
        color.b = 0U;
    }
    else if (pos < 170U)
    {
        pos = (uint8)(pos - 85U);
        color.r = 0U;
        color.g = led_strip_scale((uint8)(255U - pos * 3U), brightness);
        color.b = led_strip_scale((uint8)(pos * 3U), brightness);
    }
    else
    {
        pos = (uint8)(pos - 170U);
        color.r = led_strip_scale((uint8)(pos * 3U), brightness);
        color.g = 0U;
        color.b = led_strip_scale((uint8)(255U - pos * 3U), brightness);
    }

    return color;
}

void led_strip_init(void)
{
    led_strip_port = get_port(LED_STRIP_PIN);
    led_strip_stm = IfxStm_getAddress((IfxStm_Index)IfxCpu_getCoreId());
    led_strip_high_mask = (uint32)(1UL << ((uint32)LED_STRIP_PIN & 0x1FU));
    led_strip_low_mask = (uint32)(0x10000UL << ((uint32)LED_STRIP_PIN & 0x1FU));

    gpio_init(LED_STRIP_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    led_strip_clear();
}

void led_strip_clear(void)
{
    led_strip_set_all(0U, 0U, 0U);
}

void led_strip_refresh(void)
{
    led_strip_show(led_strip_buffer, LED_STRIP_LED_NUM);
}

void led_strip_set_pixel(uint16 index, uint8 r, uint8 g, uint8 b)
{
    if (index >= LED_STRIP_LED_NUM)
    {
        return;
    }

    led_strip_buffer[index].r = r;
    led_strip_buffer[index].g = g;
    led_strip_buffer[index].b = b;
}

void led_strip_set_all(uint8 r, uint8 g, uint8 b)
{
    uint16 i;

    for (i = 0U; i < LED_STRIP_LED_NUM; i++)
    {
        led_strip_buffer[i].r = r;
        led_strip_buffer[i].g = g;
        led_strip_buffer[i].b = b;
    }

    led_strip_refresh();
}

static void led_strip_show_rainbow_offset(uint8 brightness, uint8 offset)
{
    uint16 i;

    for (i = 0U; i < LED_STRIP_LED_NUM; i++)
    {
        uint8 pos = (uint8)((((uint32)i * 255UL) / LED_STRIP_LED_NUM) + offset);
        led_strip_buffer[i] = led_strip_color_wheel(pos, brightness);
    }

    led_strip_refresh();
}

void led_strip_show_rainbow(uint8 brightness)
{
    led_strip_show_rainbow_offset(brightness, 0U);
}

void led_strip_set_mode(led_strip_mode_enum mode)
{
    led_strip_mode_request = mode;
    led_strip_mode_update_pending = 1U;
}

void led_strip_toggle_mode(void)
{
    if (LED_STRIP_MODE_RAINBOW_FLOW == led_strip_mode_request)
    {
        led_strip_set_mode(LED_STRIP_MODE_RAINBOW_STATIC);
    }
    else
    {
        led_strip_set_mode(LED_STRIP_MODE_RAINBOW_FLOW);
    }
}

led_strip_mode_enum led_strip_get_mode(void)
{
    return led_strip_mode;
}

void led_strip_task(uint32 now_ms)
{
    if ((led_strip_mode != led_strip_mode_request) || (0U != led_strip_mode_update_pending))
    {
        led_strip_mode_update_pending = 0U;
        led_strip_mode = led_strip_mode_request;
        led_strip_last_flow_ms = 0U;

        if (LED_STRIP_MODE_RAINBOW_FLOW == led_strip_mode)
        {
            led_strip_show_rainbow_offset(LED_STRIP_SAFE_BRIGHTNESS, led_strip_rainbow_offset);
        }
        else
        {
            led_strip_rainbow_offset = 0U;
            led_strip_show_rainbow(LED_STRIP_SAFE_BRIGHTNESS);
        }
    }

    if (LED_STRIP_MODE_RAINBOW_FLOW != led_strip_mode)
    {
        return;
    }

    if ((0U == led_strip_last_flow_ms) ||
        ((uint32)(now_ms - led_strip_last_flow_ms) >= LED_STRIP_FLOW_INTERVAL_MS))
    {
        led_strip_last_flow_ms = now_ms;
        led_strip_rainbow_offset = (uint8)(led_strip_rainbow_offset + LED_STRIP_FLOW_STEP);
        led_strip_show_rainbow_offset(LED_STRIP_SAFE_BRIGHTNESS, led_strip_rainbow_offset);
    }
}

void led_strip_show(const led_strip_rgb_struct *pixels, uint16 count)
{
    uint16 i;
    uint32 interrupt_state;

    if ((0U == count) || (0 == pixels))
    {
        return;
    }

    if (0 == led_strip_port)
    {
        led_strip_init();
    }

    led_strip_stm = IfxStm_getAddress((IfxStm_Index)IfxCpu_getCoreId());
    interrupt_state = interrupt_global_disable();

    for (i = 0U; i < count; i++)
    {
        led_strip_write_color(pixels[i].r, pixels[i].g, pixels[i].b);
    }

    led_strip_data_low();
    interrupt_global_enable(interrupt_state);
    system_delay_us(LED_STRIP_RESET_US);
}
