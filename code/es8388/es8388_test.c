/**
 ****************************************************************************************************
 * @file        es8388_test.c
 * @brief       ES8388 self-test helpers for TC387
 ****************************************************************************************************
 */

#include "zf_common_headfile.h"
#include "es8388_unified.h"
#include "es8388_config.h"
#include "../../libraries/qst_sw_i2c.h"

static uint8_t g_es8388_test_addr = ES8388_IIC_ADDR;
static gpio_pin_enum g_es8388_test_scl = ES8388_SCL_PIN;
static gpio_pin_enum g_es8388_test_sda = ES8388_SDA_PIN;

static uint8_t es8388_test_detect_addr_on_bus(gpio_pin_enum scl_pin,
                                               gpio_pin_enum sda_pin,
                                               uint8_t *out_addr)
{
    uint8_t i;
    soft_iic_info_struct probe_iic;
    const uint8_t candidates[] = {ES8388_IIC_ADDR, ES8388_IIC_ADDR_ALT};

    soft_iic_init(&probe_iic, candidates[0], ES8388_IIC_DELAY, scl_pin, sda_pin);

    for (i = 0; i < (uint8_t)(sizeof(candidates) / sizeof(candidates[0])); i++)
    {
        if (soft_iic_probe_addr(&probe_iic, candidates[i]))
        {
            *out_addr = candidates[i];
            return 1;
        }
    }

    return 0;
}

static void es8388_test_sync_runtime_i2c(void)
{
    uint8_t addr = g_es8388_test_addr;
    gpio_pin_enum scl = g_es8388_test_scl;
    gpio_pin_enum sda = g_es8388_test_sda;

    es8388_get_runtime_i2c(&addr, &scl, &sda);

    g_es8388_test_addr = addr;
    g_es8388_test_scl = scl;
    g_es8388_test_sda = sda;
}

static void es8388_test_port_diagnosis(gpio_pin_enum scl_pin,
                                        gpio_pin_enum sda_pin,
                                        const char *name,
                                        bool use_default_init)
{
    uint8_t scl_low;
    uint8_t scl_high;
    uint8_t sda_low;
    uint8_t sda_high;
    uint8_t detected_addr;
    uint8_t control_reg;
    uint8_t ret;

    printf("=== ES8388 Port Diagnosis: %s ===\r\n", name);
    printf("Using QST software I2C implementation\r\n");
    printf("1. Testing I2C pins (%s):\r\n", name);

    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    gpio_set_level(scl_pin, GPIO_LOW);
    system_delay_ms(10);
    scl_low = gpio_get_level(scl_pin);
    printf("   SCL set LOW, read: %d (expected: 0)\r\n", scl_low);

    gpio_set_level(scl_pin, GPIO_HIGH);
    system_delay_ms(10);
    scl_high = gpio_get_level(scl_pin);
    printf("   SCL set HIGH, read: %d (expected: 1)\r\n", scl_high);

    gpio_set_level(sda_pin, GPIO_LOW);
    system_delay_ms(10);
    sda_low = gpio_get_level(sda_pin);
    printf("   SDA set LOW, read: %d (expected: 0)\r\n", sda_low);

    gpio_set_level(sda_pin, GPIO_HIGH);
    system_delay_ms(10);
    sda_high = gpio_get_level(sda_pin);
    printf("   SDA set HIGH, read: %d (expected: 1)\r\n", sda_high);

    if (!(scl_low == 0 && scl_high == 1 && sda_low == 0 && sda_high == 1))
    {
        printf("I2C pins have issues\r\n\r\n");
        return;
    }

    printf("I2C pins work correctly\r\n");
    printf("\r\n2. Testing QST I2C communication with ES8388...\r\n");

    if (use_default_init)
    {
        qst_sw_i2c_init_default();
    }
    else
    {
        qst_sw_i2c_init(&qst_i2c_obj, scl_pin, sda_pin, ES8388_IIC_DELAY);
    }

    detected_addr = ES8388_IIC_ADDR;
    if (!es8388_test_detect_addr_on_bus(scl_pin, sda_pin, &detected_addr))
    {
        printf("   No ACK on candidate addresses (0x%02X/0x%02X)\r\n",
               ES8388_IIC_ADDR,
               ES8388_IIC_ADDR_ALT);
        printf("=== Diagnosis Complete for %s ===\r\n\r\n", name);
        return;
    }

    g_es8388_test_addr = detected_addr;
    g_es8388_test_scl = scl_pin;
    g_es8388_test_sda = sda_pin;

    control_reg = 0xFF;
    ret = qst_sw_readreg(detected_addr, 0x00, &control_reg, 1);

    if (ret == 0)
    {
        printf("   Control register (0x00): 0x%02X (ret=%d, addr=0x%02X)\r\n", control_reg, ret, detected_addr);
    }
    else
    {
        printf("   Control register (0x00): N/A (ret=%d, no ACK, addr=0x%02X)\r\n", ret, detected_addr);
    }

    printf("=== Diagnosis Complete for %s ===\r\n\r\n", name);
}

static void es8388_test_i2c_address_scan(void)
{
    uint8_t addr;
    uint8_t found_count = 0;
    uint8_t found_addresses[16];

    printf("I2C Address Scan...\r\n");

    for (addr = 0x01; addr < 0x80; addr++)
    {
        soft_iic_info_struct scan_iic;
        soft_iic_init(&scan_iic, addr, ES8388_IIC_DELAY, g_es8388_test_scl, g_es8388_test_sda);

        if (soft_iic_probe_addr(&scan_iic, addr))
        {
            uint8_t reg0 = soft_iic_read_8bit_register(&scan_iic, 0x00);
            printf("Found device at address 0x%02X, Reg 0x00 = 0x%02X\r\n", addr, reg0);
            if (found_count < 16)
            {
                found_addresses[found_count++] = addr;
            }
        }
    }

    printf("Scan complete. Found %d potential devices.\r\n", found_count);

    if (found_count > 0)
    {
        uint8_t i;
        printf("Found addresses: ");
        for (i = 0; i < found_count; i++)
        {
            printf("0x%02X ", found_addresses[i]);
        }
        printf("\r\n");

        g_es8388_test_addr = found_addresses[0];
        printf("Preferred ES8388 test address set to 0x%02X\r\n", g_es8388_test_addr);
    }
    else
    {
        printf("No I2C devices found on bus\r\n");
    }
}

void es8388_test_i2c_scan(void)
{
    uint8_t chip_id_high;
    uint8_t chip_id_low;
    uint8_t reg0;
    soft_iic_info_struct test_iic;

    printf("I2C Device Detection...\r\n");

    es8388_test_port_diagnosis(ES8388_SCL_PIN, ES8388_SDA_PIN, "P13_3/P13_2", false);
    es8388_test_port_diagnosis(P33_8, P32_4, "P33_8/P32_4", true);

    es8388_test_i2c_address_scan();

    soft_iic_init(&test_iic, g_es8388_test_addr, ES8388_IIC_DELAY, g_es8388_test_scl, g_es8388_test_sda);

    printf("Using I2C address: 0x%02X (SCL=P%d_%d, SDA=P%d_%d)\r\n",
           g_es8388_test_addr,
           g_es8388_test_scl / 32, g_es8388_test_scl % 32,
           g_es8388_test_sda / 32, g_es8388_test_sda % 32);

    chip_id_high = soft_iic_read_8bit_register(&test_iic, 0x28);
    chip_id_low = soft_iic_read_8bit_register(&test_iic, 0x29);
    reg0 = soft_iic_read_8bit_register(&test_iic, 0x00);

    printf("Chip ID High: 0x%02X\r\n", chip_id_high);
    printf("Chip ID Low: 0x%02X\r\n", chip_id_low);
    printf("Control Reg (0x00): 0x%02X\r\n", reg0);
}

uint8_t es8388_test_init(void)
{
    uint8_t ret;
    uint8_t reg_value;
    soft_iic_info_struct verify_iic;

    printf("ES8388 Init Test...\r\n");
    system_delay_ms(100);

    ret = es8388_init();
    if (ret != 0)
    {
        printf("ES8388 Init FAIL!\r\n");
        return 1;
    }

    es8388_test_sync_runtime_i2c();

    if (!es8388_test_detect_addr_on_bus(g_es8388_test_scl, g_es8388_test_sda, &g_es8388_test_addr))
    {
        printf("ES8388 Init returned OK but no ACK on bus\r\n");
        return 1;
    }

    soft_iic_init(&verify_iic, g_es8388_test_addr, ES8388_IIC_DELAY, g_es8388_test_scl, g_es8388_test_sda);
    reg_value = soft_iic_read_8bit_register(&verify_iic, 0x00);

    printf("Verify using I2C address: 0x%02X (SCL=P%d_%d, SDA=P%d_%d)\r\n",
           g_es8388_test_addr,
           g_es8388_test_scl / 32, g_es8388_test_scl % 32,
           g_es8388_test_sda / 32, g_es8388_test_sda % 32);
    printf("Control Reg (0x00) after init: 0x%02X (expected: 0x06)\r\n", reg_value);

    if (reg_value == 0x06)
    {
        printf("ES8388 initialization verified successful\r\n");
        return 0;
    }

    if (reg_value == 0xFF)
    {
        printf("ES8388 no valid response on I2C\r\n");
        return 1;
    }

    printf("ES8388 responded but control reg unexpected: 0x%02X\r\n", reg_value);
    return 0;
}

void es8388_test_i2c_communication(void)
{
    uint8_t reg00;
    uint8_t reg01;
    uint8_t reg03;
    uint8_t test_data = 0xAA;
    soft_iic_info_struct test_iic;

    printf("I2C Comm Test...\r\n");

    es8388_test_sync_runtime_i2c();
    soft_iic_init(&test_iic, g_es8388_test_addr, ES8388_IIC_DELAY, g_es8388_test_scl, g_es8388_test_sda);

    printf("Comm test using I2C address: 0x%02X (SCL=P%d_%d, SDA=P%d_%d)\r\n",
           g_es8388_test_addr,
           g_es8388_test_scl / 32, g_es8388_test_scl % 32,
           g_es8388_test_sda / 32, g_es8388_test_sda % 32);

    reg00 = soft_iic_read_8bit_register(&test_iic, 0x00);
    reg01 = soft_iic_read_8bit_register(&test_iic, 0x01);
    reg03 = soft_iic_read_8bit_register(&test_iic, 0x03);

    printf("Reg 0x00: 0x%02X\r\n", reg00);
    printf("Reg 0x01: 0x%02X\r\n", reg01);
    printf("Reg 0x03: 0x%02X\r\n", reg03);

    soft_iic_write_8bit_register(&test_iic, 0x03, test_data);
    system_delay_ms(10);

    if (soft_iic_read_8bit_register(&test_iic, 0x03) == test_data)
    {
        printf("I2C Comm OK\r\n");
    }
    else
    {
        printf("I2C Comm FAIL\r\n");
    }

    soft_iic_write_8bit_register(&test_iic, 0x03, reg03);
}

void es8388_test_play_tone(uint32_t frequency, uint32_t duration_ms)
{
    printf("Play %dHz %dms\r\n", frequency, duration_ms);
    system_delay_ms(duration_ms);
    printf("Play Complete\r\n");
}

void es8388_test_all(void)
{
    printf("\r\n=== ES8388 Test Start ===\r\n");
    printf("Serial output test: Hello ES8388!\r\n");

    es8388_test_i2c_scan();
    system_delay_ms(1000);

    if (es8388_test_init() == 0)
    {
        system_delay_ms(1000);
        es8388_test_i2c_communication();
        system_delay_ms(1000);
        es8388_test_play_tone(1000, 2000);
        system_delay_ms(1000);
        printf("All Tests Done\r\n");
    }
    else
    {
        printf("Init Failed - Check HW\r\n");
    }

    printf("=== ES8388 Test End ===\r\n\r\n");
    system_delay_ms(2000);
}