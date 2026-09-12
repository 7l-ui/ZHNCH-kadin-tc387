/*********************************************************************************************************************
 * XL9555 Application Layer - Implementation
 *
 * Board target: TC387 + XL9555 (I2C 16-bit GPIO expander).
 *
 * This module provides:
 *   - Device init/address scan
 *   - Key/switch input wrappers
 *   - LED/LoRa output/status wrappers
 *   - Optional self-test API (disabled by default)
 *********************************************************************************************************************/

#include "xl9555_app.h"
#include <string.h>

static xl9555_dev_t g_xl9555_dev;
static xl9555_input_status_t g_last_input_status;
static xl9555_output_control_t g_output_control;
static uint32_t g_debounce_counters[5] = {0};
static uint8_t g_xl9555_ready = 0;
static uint8_t g_xl9555_addr = XL9555_I2C_ADDRESS_BASE;

#define DEBOUNCE_THRESHOLD     3
#define DEBOUNCE_INTERVAL_MS   10

#ifndef XL9555_APP_ENABLE_SELF_TEST
#define XL9555_APP_ENABLE_SELF_TEST  (0)
#endif

static void xl9555_enhance_pullup(void)
{
    gpio_init(XL9555_SDA_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(XL9555_SCL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(10);
    
    gpio_init(XL9555_SDA_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(XL9555_SCL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(1);
}

static void xl9555_bus_recover(void)
{
    xl9555_enhance_pullup();
    
    gpio_init(XL9555_SCL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(XL9555_SDA_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);

    for (uint8_t i = 0; i < 9; i++) {
        gpio_set_level(XL9555_SCL_PIN, GPIO_HIGH);
        system_delay_us(20);
        gpio_set_level(XL9555_SCL_PIN, GPIO_LOW);
        system_delay_us(20);
    }

    gpio_set_level(XL9555_SCL_PIN, GPIO_HIGH);
    gpio_set_level(XL9555_SDA_PIN, GPIO_HIGH);
    system_delay_us(20);
}

// Probe one candidate address and verify read/write path using polarity registers.
// The probe value is always rolled back to avoid side effects.
static uint8_t xl9555_app_try_init_with_addr(uint8_t addr)
{
    uint8_t pol0_before = 0, pol1_before = 0;
    uint8_t pol0_after = 0, pol1_after = 0;

    xl9555_init_desc(&g_xl9555_dev,
                     addr,
                     XL9555_I2C_DELAY,
                     XL9555_SCL_PIN,
                     XL9555_SDA_PIN);

    pol0_before = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0);
    pol1_before = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1);

    soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0, 0x5A);
    soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1, 0xA5);
    pol0_after = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0);
    pol1_after = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1);
    soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0, pol0_before);
    soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1, pol1_before);

    // Keep serial output concise during normal startup; remove per-address probe dump.

    if ((pol0_after == 0x5A) && (pol1_after == 0xA5)) {
        xl9555_app_configure_pin_directions();
        xl9555_app_configure_polarity();
        g_xl9555_addr = addr;
        return 1;
    }

    return 0;
}

/**
 * @brief XL9555 �?�?�涓?I2C �?荤嚎璇婃�?
 * @note 鎵撳嵃�??曡剼鐢靛钩銆佹€荤嚎鐘舵�?佷笌�?歌鏁呴殰鎺掓煡淇℃�?
 */
// �?件与 I2C 总线诊断，输出排查信�?
void xl9555_app_diagnose(void)
{
    uint8_t scl_high, scl_low, sda_high, sda_low;
    
    printf("\n========== XL9555 Hardware Diagnosis ==========\r\n");
    printf("XL9555_SCL_PIN: 0x%04X (P13.1)\r\n", XL9555_SCL_PIN);
    printf("XL9555_SDA_PIN: 0x%04X (P13.0)\r\n", XL9555_SDA_PIN);
    printf("XL9555_I2C_DELAY: %d\r\n", XL9555_I2C_DELAY);
    printf("XL9555_I2C_ADDRESS_BASE: 0x%02X\r\n", XL9555_I2C_ADDRESS_BASE);
    
    printf("\n1. Checking I2C bus idle state (no I2C init yet)...\r\n");
    gpio_init(XL9555_SCL_PIN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(XL9555_SDA_PIN, GPI, 0, GPI_FLOATING_IN);
    system_delay_ms(5);
    scl_high = gpio_get_level(XL9555_SCL_PIN);
    sda_high = gpio_get_level(XL9555_SDA_PIN);
    printf("   SCL idle level: %d (should be 1 ideally)\r\n", scl_high);
    printf("   SDA idle level: %d (should be 1 ideally)\r\n", sda_high);
    
    if (sda_high == 0) {
        printf("   WARNING: SDA stuck at 0! This usually means:\r\n");
        printf("   - XL9555 chip not powered, or\r\n");
        printf("   - There's something pulling SDA to GND, or\r\n");
        printf("   - SDA is in communication with another device\r\n");
    }
    
    printf("\n2. Testing SCL push-pull output capability...\r\n");
    gpio_init(XL9555_SCL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_set_level(XL9555_SCL_PIN, GPIO_HIGH);
    system_delay_us(10);
    scl_high = gpio_get_level(XL9555_SCL_PIN);
    printf("   SCL set HIGH: level = %d\r\n", scl_high);
    
    gpio_set_level(XL9555_SCL_PIN, GPIO_LOW);
    system_delay_us(10);
    scl_low = gpio_get_level(XL9555_SCL_PIN);
    printf("   SCL set LOW:  level = %d\r\n", scl_low);
    
    if (scl_high == 1 && scl_low == 0) {
        printf("   ? SCL output OK\r\n");
    } else {
        printf("   ? SCL output FAIL! Check GPIO configuration\r\n");
    }
    gpio_set_level(XL9555_SCL_PIN, GPIO_HIGH);
    
    printf("\n3. Testing SDA open-drain output capability...\r\n");
    gpio_init(XL9555_SDA_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    system_delay_us(10);
    sda_high = gpio_get_level(XL9555_SDA_PIN);
    printf("   SDA released (high-Z): level = %d\r\n", sda_high);
    
    gpio_set_level(XL9555_SDA_PIN, GPIO_LOW);
    system_delay_us(10);
    sda_low = gpio_get_level(XL9555_SDA_PIN);
    printf("   SDA set LOW:           level = %d\r\n", sda_low);
    
    gpio_set_level(XL9555_SDA_PIN, GPIO_HIGH);  // Release
    system_delay_us(10);
    sda_high = gpio_get_level(XL9555_SDA_PIN);
    printf("   SDA released (high-Z): level = %d\r\n", sda_high);
    
    if (sda_low == 0) {
        printf("   ? SDA can pull LOW\r\n");
        if (sda_high == 0) {
            printf("   ? WARNING: SDA cannot pull HIGH! No external pull-up or stuck low!\r\n");
        } else {
            printf("   ? SDA can return HIGH\r\n");
        }
    } else {
        printf("   ? SDA pull-LOW failed! GPIO configuration issue\r\n");
    }
    
    printf("\n4. Attempting I2C START signal...\r\n");
    gpio_init(XL9555_SCL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(XL9555_SDA_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    
    gpio_set_level(XL9555_SCL_PIN, GPIO_HIGH);
    gpio_set_level(XL9555_SDA_PIN, GPIO_HIGH);
    system_delay_us(20);
    printf("   Initial: SCL=%d, SDA=%d\r\n", 
           gpio_get_level(XL9555_SCL_PIN), gpio_get_level(XL9555_SDA_PIN));
    
    gpio_set_level(XL9555_SDA_PIN, GPIO_LOW);
    system_delay_us(20);
    printf("   After SDA pull-low: SCL=%d, SDA=%d\r\n", 
           gpio_get_level(XL9555_SCL_PIN), gpio_get_level(XL9555_SDA_PIN));
    
    gpio_set_level(XL9555_SCL_PIN, GPIO_HIGH);
    gpio_set_level(XL9555_SDA_PIN, GPIO_HIGH);
    system_delay_ms(1);
    
    printf("\n5. CRITICAL CHECKS:\r\n");
    printf("   a) XL9555 power supply:\r\n");
    printf("      - Use multimeter to check VCC pin (should be ~3.3V or ~5V)\r\n");
    printf("      - Use multimeter to check GND pin (should be 0V)\r\n");
    printf("   b) XL9555 address pins (A0, A1, A2):\r\n");
    printf("      - Should all be tied to GND for address 0x20\r\n");
    printf("      - Do NOT leave floating!\r\n");
    printf("   c) XL9555 I2C pins:\r\n");
    printf("      - SCL and SDA should be physically connected to P13.1 and P13.0\r\n");
    printf("      - Check for cold joints or bridge solder\r\n");
    printf("   d) If SDA is stuck low:\r\n");
    printf("      - Try XL9555 power cycle (turn off and on again)\r\n");
    printf("      - Or check if INT pin needs pull-up\r\n");
    printf("\n===============================================\r\n\n");
}

/**
 * @brief 鍒濆鍖?XL9555 搴旂敤灞? * @note 鎵弿璁惧鍦板潃锛岄厤缃柟鍚?鏋佹�?э紝骞朵笅鍙戦�?璁よ�?鍑虹姸鎬? */
// 初�?�化应用层：�?描地址、配�?方向和极性、�?�置默�?�输�?
void xl9555_app_init(void)
{
    // Init sequence: recover bus, scan device, configure IO direction/polarity, apply default outputs.
    xl9555_bus_recover();

    g_xl9555_ready = 0;
    for (uint16_t addr = XL9555_I2C_ADDRESS_BASE; addr <= XL9555_I2C_ADDRESS_MAX; addr++) {
        if (xl9555_app_try_init_with_addr((uint8_t)addr)) {
            g_xl9555_ready = 1;
            break;
        }
    }

    g_output_control.led_enable = XL9555_LED_OFF;
    g_output_control.led_run = XL9555_LED_OFF;
    g_output_control.lora_aux = 0;
    g_output_control.lora_mode = XL9555_LORA_MODE_0;

    if (g_xl9555_ready) {
        xl9555_app_set_all_outputs(g_output_control);
    }

    memset(&g_last_input_status, 0, sizeof(g_last_input_status));

    for (int i = 0; i < 5; i++) {
        g_debounce_counters[i] = 0;
    }
}

/**
 * @brief 鑾峰彇搴曞眰 XL9555 璁惧�?瀵�?�薄鎸囬�?
 * @return 璁惧�?瀵�?�薄鍦板潃锛坴oid*�?? */
void* xl9555_app_get_device(void)
{
    return (void*)&g_xl9555_dev;
}

uint8 xl9555_app_is_ready(void)
{
    return g_xl9555_ready;
}

uint8 xl9555_app_get_addr(void)
{
    return g_xl9555_addr;
}

/**
 * @brief 璇�?�彇鍗曚釜鎸夐敭鐘舵�?? * @param key_num 鎸�?�敭缂栧彿锛?~5�?? * @return 1=鎸�?�笅�??=鏈寜涓? */
uint8_t xl9555_app_get_key(uint8_t key_num)
{
    uint8_t gpio_num, level;

    if (!g_xl9555_ready) {
        return 0;
    }
    
    if (key_num < 1 || key_num > 5) {
        return XL9555_KEY_RELEASED;
    }
    
    switch (key_num) {
        case 1: gpio_num = XL9555_KEY_1; break;
        case 2: gpio_num = XL9555_KEY_2; break;
        case 3: gpio_num = XL9555_KEY_3; break;
        case 4: gpio_num = XL9555_KEY_4; break;
        case 5: gpio_num = XL9555_KEY_5; break;
        default: gpio_num = XL9555_KEY_1; break;
    }
    
    xl9555_get_gpio_level(&g_xl9555_dev, gpio_num, &level);
    
    return (level == XL9555_KEY_PRESSED) ? 1 : 0;
}

/**
 * @brief 璇�?�彇鍗曚釜鎷ㄧ爜�?�?鍏崇姸鎬? * @param switch_num �?�?鍏崇�?鍙凤�?1~2�?? * @return 1=鎵撳紑锛?=鍏抽�?
 */
uint8_t xl9555_app_get_switch(uint8_t switch_num)
{
    uint8_t gpio_num, level;

    if (!g_xl9555_ready) {
        return 0;
    }
    
    if (switch_num < 1 || switch_num > 2) {
        return XL9555_SWITCH_OFF;
    }
    
    if (switch_num == 1) {
        gpio_num = XL9555_SWITCH_1;
    } else {
        gpio_num = XL9555_SWITCH_2;
    }
    
    xl9555_get_gpio_level(&g_xl9555_dev, gpio_num, &level);
    
    return (level == XL9555_SWITCH_ON) ? 1 : 0;
}

/**
 * @brief 璇�?�彇鍏ㄩ儴杈撳叆鐘舵�?侊紙鎸�?�敭+鎷ㄧ爜锛? * @param status 杈撳�?缁撴�?浣撴寚閽? */
// 读取所有输入状态（按键+拨码�?
void xl9555_app_get_all_inputs(xl9555_input_status_t *status)
{
    if (status == NULL) {
        return;
    }
    
    status->key1 = xl9555_app_get_key(1);
    status->key2 = xl9555_app_get_key(2);
    status->key3 = xl9555_app_get_key(3);
    status->key4 = xl9555_app_get_key(4);
    status->key5 = xl9555_app_get_key(5);
    
    status->switch1 = xl9555_app_get_switch(1);
    status->switch2 = xl9555_app_get_switch(2);
    
    status->reserved = 0;
}

/**
 * @brief 鎸�?�敭鎵弿锛堝甫娑堟姈锛? * @param key_pressed 杈撳�?鎸�?�敭缂栧�?
 * @return 1=�?�?娴�??埌鏈夋晥鎸�?�敭�??=鏃犳寜閿? */
uint8_t xl9555_app_scan_keys(uint8_t *key_pressed)
{
    static uint32_t last_scan_time = 0;
    uint32_t current_time;
    uint8_t key_state;
    uint8_t any_key_pressed = 0;
    
    current_time = system_getval_ms();
    
    if ((current_time - last_scan_time) < DEBOUNCE_INTERVAL_MS) {
        return 0;
    }
    
    last_scan_time = current_time;
    
    for (uint8_t i = 1; i <= 5; i++) {
        key_state = xl9555_app_get_key(i);
        
        if (key_state == 1) {
            g_debounce_counters[i-1]++;
            if (g_debounce_counters[i-1] >= DEBOUNCE_THRESHOLD) {
                if (key_pressed != NULL) {
                    *key_pressed = i;
                }
                any_key_pressed = 1;
                g_debounce_counters[i-1] = DEBOUNCE_THRESHOLD;
            }
        } else {
            g_debounce_counters[i-1] = 0;
        }
    }
    
    return any_key_pressed;
}

/**
 * @brief 鎺у埗浣胯�? LED
 * @param state 1=浜�?0=�?? */
// 控制使能 LED �?�?
void xl9555_app_led_enable(uint8_t state)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.led_enable = (state != 0) ? 1 : 0;
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LED_ENABLE, 
                         g_output_control.led_enable ? XL9555_LED_ON : XL9555_LED_OFF);
}

/**
 * @brief 鎺у埗杩愯�? LED
 * @param state 1=浜�?0=�?? */
// 控制运�?? LED �?�?
void xl9555_app_led_run(uint8_t state)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.led_run = (state != 0) ? 1 : 0;
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LED_RUN, 
                         g_output_control.led_run ? XL9555_LED_ON : XL9555_LED_OFF);
}

/**
 * @brief 缈昏浆浣�?�? LED 鐘舵�?? */
// 翻转使能 LED 状�?
void xl9555_app_led_enable_toggle(void)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.led_enable = !g_output_control.led_enable;
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LED_ENABLE, 
                         g_output_control.led_enable ? XL9555_LED_ON : XL9555_LED_OFF);
}

/**
 * @brief 缈昏浆杩�?�? LED 鐘舵�?? */
// 翻转运�?? LED 状�?
void xl9555_app_led_run_toggle(void)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.led_run = !g_output_control.led_run;
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LED_RUN, 
                         g_output_control.led_run ? XL9555_LED_ON : XL9555_LED_OFF);
}

uint8_t xl9555_app_lora_get_aux(void)
{
    uint8_t level = 0;

    if (!g_xl9555_ready) {
        return 0;
    }

    xl9555_get_gpio_level(&g_xl9555_dev, XL9555_LORA_AUX, &level);
    g_output_control.lora_aux = (level != 0U) ? 1U : 0U;
    return g_output_control.lora_aux;
}

/**
 * @brief 璁剧�? LoRa AUX 鎺у埗�?? * @param state 1=楂�?�數骞�?�紝0=浣庣數骞? */
// LoRa AUX is driven by the module. Keep this API as a no-drive compatibility shim.
void xl9555_app_lora_set_aux(uint8_t state)
{
    (void)state;

    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.lora_aux = xl9555_app_lora_get_aux();
}

/**
 * @brief 璁剧�? LoRa �?″紡鎺у埗�?? * @param mode 0 �??1
 */
// 设置 LoRa 模式控制�?
void xl9555_app_lora_set_mode(uint8_t mode)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.lora_mode = (mode != 0) ? 1 : 0;
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LORA_MOD, g_output_control.lora_mode);
}

/**
 * @brief 缈昏�? LoRa �?″紡鎺у埗�?? */
// 翻转 LoRa 模式控制�?
void xl9555_app_lora_toggle_mode(void)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control.lora_mode = !g_output_control.lora_mode;
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LORA_MOD, g_output_control.lora_mode);
}

/**
 * @brief 鎵归噺璁剧疆鎵€鏈�?�緭鍑烘帶鍒朵綅
 * @param control 杈撳�?鎺у埗缁撴�?�?? */
// 批量设置输出控制状�?
void xl9555_app_set_all_outputs(xl9555_output_control_t control)
{
    if (!g_xl9555_ready) {
        return;
    }

    g_output_control = control;
    
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LED_ENABLE, 
                         g_output_control.led_enable ? XL9555_LED_ON : XL9555_LED_OFF);
    
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LED_RUN, 
                         g_output_control.led_run ? XL9555_LED_ON : XL9555_LED_OFF);
    
    xl9555_set_gpio_level(&g_xl9555_dev, XL9555_LORA_MOD, g_output_control.lora_mode);
}

/**
 * @brief 鑾峰彇褰撳�?�杈撳嚭鎺у埗鐘舵�?? * @param control 杈撳�?缁撴�?浣撴寚閽? */
// 读取当前输出控制状�?
void xl9555_app_get_all_outputs(xl9555_output_control_t *control)
{
    if (control == NULL) {
        return;
    }
    
    *control = g_output_control;
}

/**
 * @brief 閰嶇�? XL9555 �?曡剼鏂瑰�?
 * @note IO0 杈撳叆锛孖O1(8~11) 杈撳�?�??2~15 杈撳�?
 */
// 配置引脚方向（IO0 输入，IO1 部分输出�?
void xl9555_app_configure_pin_directions(void)
{
    // Direction map for current board:
    // IO0(GPIO0~7)=input, GPIO8/9/11=output, GPIO10(LORA_AUX)=input, GPIO12~15=input.
    uint16_t direction_mask = 0xF4FF;  // 0b1111010011111111
    
    xl9555_set_full_gpio_mode(&g_xl9555_dev, direction_mask);
    
    #if 0
    uint16_t verify_mask;
    xl9555_get_full_gpio_mode(&g_xl9555_dev, &verify_mask);
    if (verify_mask != direction_mask) {
        printf("XL9555 direction config mismatch: expected 0x%04X, got 0x%04X\r\n", 
               direction_mask, verify_mask);
    }
    #endif
}

/**
 * @brief 閰嶇疆杈撳叆鏋佹�?? * @note 鍏ㄩ儴璁剧疆涓轰笉鍙嶇浉�??x0000�?? */
// 配置输入极性为不反�?
void xl9555_app_configure_polarity(void)
{
    // Keep all pins as non-inverted and clear any temporary probe pattern.
    xl9555_set_full_gpio_polarity(&g_xl9555_dev, 0x0000);
}

/**
 * @brief 鎵归噺璇诲彇 16 �??GPIO 杈撳叆鍊? * @param levels 杈撳�?鍊兼寚閽? */
// 批量读取 16 �? GPIO 输入
void xl9555_app_read_all_gpio(uint16_t *levels)
{
    if (levels == NULL) {
        return;
    }

    if (!g_xl9555_ready) {
        *levels = 0;
        return;
    }

    xl9555_get_full_gpio_level(&g_xl9555_dev, levels);
}

/**
 * @brief 鎵归噺鍐欏叆 16 �??GPIO 杈撳�?�?? * @param levels 杈撳�?鎺╃�?
 */
// 批量写入 16 �? GPIO 输出
void xl9555_app_write_all_gpio(uint16_t levels)
{
    if (!g_xl9555_ready) {
        return;
    }

    xl9555_set_full_gpio_level(&g_xl9555_dev, levels);
}

/**
 * @brief XL9555 鑷鎺ュ彛锛堥�?璁ゅ叧闂級
 * @note �?氳繃 XL9555_APP_ENABLE_SELF_TEST 鎺у埗鏄惁鍚敤
 */
// XL9555 �?检接口（默认关�?�?
void xl9555_app_test(void)
{
#if (XL9555_APP_ENABLE_SELF_TEST == 0)
    return;
#else
    static uint8_t has_run = 0;
    uint8_t scl_idle = 0, sda_idle = 0;
    uint8_t mode0 = 0, mode1 = 0;
    uint8_t pol0_before = 0, pol1_before = 0;
    uint8_t pol0_after = 0, pol1_after = 0;
    uint8_t in0 = 0, in1 = 0;
    uint8_t found = 0;
    uint8_t found_addr = 0;

    if (has_run) {
        return;
    }
    has_run = 1;

    printf("\r\n========== XL9555 I2C SELF TEST ==========" "\r\n");

    gpio_init(XL9555_SCL_PIN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(XL9555_SDA_PIN, GPI, 0, GPI_FLOATING_IN);
    system_delay_ms(2);
    scl_idle = gpio_get_level(XL9555_SCL_PIN);
    sda_idle = gpio_get_level(XL9555_SDA_PIN);
    printf("Bus idle level: SCL=%d SDA=%d\r\n", scl_idle, sda_idle);

    if ((scl_idle == 0) || (sda_idle == 0)) {
        printf("WARN: I2C line is not HIGH in idle state. Check pull-up/power/wiring.\r\n");
    }

    xl9555_bus_recover();

    for (uint8_t addr = XL9555_I2C_ADDRESS_BASE; addr <= XL9555_I2C_ADDRESS_MAX; addr++) {
        xl9555_init_desc(&g_xl9555_dev, addr, XL9555_I2C_DELAY, XL9555_SCL_PIN, XL9555_SDA_PIN);

        mode0 = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_MODE_PORT0);
        mode1 = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_MODE_PORT1);
        pol0_before = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0);
        pol1_before = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1);

        soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0, 0x5A);
        soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1, 0xA5);
        pol0_after = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0);
        pol1_after = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1);

        soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT0, pol0_before);
        soft_iic_write_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_POLARITY_PORT1, pol1_before);
        printf("Probe 0x%02X: mode=%02X %02X, pol(before)=%02X %02X, pol(after)=%02X %02X\r\n",
               addr, mode0, mode1, pol0_before, pol1_before, pol0_after, pol1_after);

        if ((pol0_after == 0x5A) && (pol1_after == 0xA5)) {
            found = 1;
            found_addr = addr;
            break;
        }
    }

    if (!found) {
        g_xl9555_ready = 0;
        printf("I2C TEST FAIL: No XL9555 response at 0x20~0x27\r\n");
        xl9555_app_diagnose();
        printf("============================================\r\n\r\n");
        return;
    }

    g_xl9555_addr = found_addr;
    g_xl9555_ready = 1;

    xl9555_app_configure_pin_directions();
    xl9555_app_configure_polarity();

    printf("I2C TEST PASS: XL9555 found at 0x%02X\r\n", found_addr);

    for (uint8_t i = 0; i < 5; i++) {
        in0 = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_INPUT_PORT0);
        in1 = soft_iic_read_8bit_register(g_xl9555_dev.soft_iic_obj, XL9555_REG_INPUT_PORT1);
        printf("Input sample[%d]: PORT0=0x%02X PORT1=0x%02X\r\n", i, in0, in1);
        system_delay_ms(20);
    }

    printf("============================================\r\n\r\n");
#endif
}


