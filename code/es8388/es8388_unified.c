/**
 ****************************************************************************************************
 * @file        es8388_unified.c
 * @author      锟斤拷锟斤拷原锟斤拷锟脚讹拷(ALIENTEK) / 锟斤拷煽萍锟?
 * @version     V2.0
 * @date        2026-03-23
 * @brief       ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷TC387锟斤拷植锟芥�? 锟斤拷锟缴诧拷锟脚猴拷录锟斤拷锟斤拷锟斤拷实锟斤�?
 * @license     Copyright (c) 2020-2032, 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟接科硷拷锟斤拷锟睫癸拷�?
 * 
 * 锟斤拷锟斤拷说锟斤拷锟斤�?
 * 1. ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷I2C锟侥达拷锟斤拷锟斤拷写锟斤拷
 * 2. ES8388锟斤拷频锟斤拷锟脚癸拷锟杰ｏ拷DAC锟斤拷锟斤拷�?
 * 3. ES8388录锟斤拷锟斤拷锟杰ｏ拷ADC锟斤拷锟诫）
 * 4. 锟斤拷锟斤拷I2S锟接口匡拷锟斤拷
 * 
 * 锟斤拷植说锟斤拷锟斤�?
 * 1. 锟斤拷原STM32F407锟斤拷锟斤拷锟斤拷植锟斤拷Infineon TC387平台
 * 2. 使锟斤拷锟斤拷煽萍锟絋C387锟斤拷源锟斤拷锟斤拷锟斤拷锟絀IC锟斤拷锟斤拷
 * 3. 锟斤拷锟斤拷TC387锟斤拷GPIO锟斤拷系统锟接匡�?
 * 4. 锟斤拷锟缴诧拷锟脚猴拷录锟斤拷锟斤拷锟杰碉拷锟斤拷锟斤拷锟侥硷拷
 ****************************************************************************************************
 * @attention
 *
 * 实锟斤拷平台:锟斤拷锟斤拷原锟斤拷 探锟斤拷锟斤�?F407锟斤拷锟斤拷锟藉（原锟芥�?
 *          锟斤拷煽萍锟?TC387锟斤拷锟斤拷锟藉（锟斤拷植锟芥）
 * 锟斤拷锟斤拷锟斤拷频:www.yuanzige.com
 * 锟斤拷锟斤拷锟斤拷坛:www.openedv.com
 * 锟斤拷司锟斤拷址:www.alientek.com
 * 锟斤拷锟斤拷锟街?openedv.taobao.com
 *
 * 锟睫革拷说锟斤拷
 * V1.0 20211116 锟斤拷一锟轿凤拷锟斤拷锟斤拷原锟芥）
 * V2.0 20260323 TC387锟斤拷植锟芥（锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
 * V3.0 20260323 锟斤拷锟缴诧拷锟脚猴拷录锟斤拷锟斤拷锟斤�?
 *
 ****************************************************************************************************
 */

#include "es8388_unified.h"
#include "../../libraries/zf_driver/zf_driver_soft_iic.h"
#include "../../libraries/zf_driver/zf_driver_delay.h"
#include "zf_driver_soft_iis.h"
#include <stdio.h>

// ==================== 鍏ㄥ眬鍙橀噺瀹氫�?====================

// ES8388 I2C瀵硅薄锛堢嫭绔嬬殑锛屼笉涓嶮T6701鍏变韩锛?
static soft_iic_info_struct es8388_iic_obj;

// 鎾斁鍣ㄥ叏灞€鍙橀�?
static uint8_t es8388_play_busy = 0;
static uint8_t es8388_play_stop_request = 0;
static soft_iis_info_struct es8388_iis_tx_obj;  // 锟斤拷锟酵讹拷锟襟（诧拷锟斤拷锟矫ｏ�?

// 录锟斤拷锟斤拷亟峁癸拷宥拷�?
typedef struct {
    es8388_record_state_enum state;
    es8388_record_config_struct config;
    soft_iis_info_struct iis_obj;
    uint32_t total_samples;
    uint32_t buffer_size;
} es8388_record_struct;

// 录锟斤拷锟斤拷锟饺拷直锟斤拷锟?
static es8388_record_struct g_es8388_record;
static es8388_record_callback_t g_record_callback = NULL;
static void *g_record_user_data = NULL;

// 默锟斤拷录锟斤拷锟斤拷锟斤拷
static const es8388_record_config_struct DEFAULT_RECORD_CONFIG = {
    .sample_rate = 44100,      // 44.1kHz锟斤拷锟斤拷锟斤�?
    .bits_per_sample = 16,     // 16位锟斤拷锟斤�?
    .channels = 1,             // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷朔锟酵拷锟轿拷锟斤拷锟斤拷锟斤拷锟?
    .input_source = 1          // 锟斤拷朔锟斤拷锟斤拷�?
};

// 锟斤拷频锟斤拷锟斤拷锟斤�?
#define ES8388_AUDIO_BUFFER_SIZE 512
static int16 es8388_audio_buffer[ES8388_AUDIO_BUFFER_SIZE];
#define ES8388_RECORD_TRY_TIMEOUT_US  (300U)
// 锟斤拷前锟斤拷效锟斤拷I2C锟斤拷锟竭诧拷锟斤拷锟斤拷支锟斤拷锟皆讹拷探锟解�?
static uint8_t g_es8388_i2c_addr = ES8388_IIC_ADDR;
static gpio_pin_enum g_es8388_scl_pin = ES8388_SCL_PIN;
static gpio_pin_enum g_es8388_sda_pin = ES8388_SDA_PIN;
static uint8_t g_es8388_ready = 0;
static uint8_t g_es8388_record_iis_format = SOFT_IIS_FORMAT_I2S;
#define ES8388_MSC_BIT (0x80U)
#define ES8388_MCLKDIV2_BIT (0x40U)
#define ES8388_BCLK_INV_BIT (0x20U)
#define ES8388_BCLKDIV_MASK (0x1FU)
#define ES8388_BCLKDIV_MCLK_DIV_8 (0x06U)
#define ES8388_ADCLRCK_DIV_MCLK_DIV_256 (0x02U)

static int16_t es8388_pick_mono_sample(int16_t left, int16_t right)
{
    int32_t abs_l = (left >= 0) ? left : -(int32_t)left;
    int32_t abs_r = (right >= 0) ? right : -(int32_t)right;
    return (abs_r > abs_l) ? right : left;
}


static void es8388_record_warmup_frames(uint16_t frames)
{
    int16_t left = 0;
    int16_t right = 0;

    for (uint16_t i = 0; i < frames; i++)
    {
        soft_iis_receive_16bit(&g_es8388_record.iis_obj, &left, &right);
    }
}
static uint8_t es8388_clamp_record_iis_format(uint8_t fmt)
{
    if (fmt > (uint8_t)SOFT_IIS_FORMAT_RIGHT_JUSTIFIED) {
        return (uint8_t)SOFT_IIS_FORMAT_I2S;
    }
    return fmt;
}
static void es8388_set_iis_role(uint8_t codec_master)
{
    uint8_t clock_reg = es8388_read_reg(0x08);
    if (clock_reg == 0xFFU) {
        clock_reg = 0x00U;
    }
    if (codec_master) {
        clock_reg &= (uint8_t)(~(ES8388_MCLKDIV2_BIT | ES8388_BCLK_INV_BIT | ES8388_BCLKDIV_MASK));
        clock_reg |= (uint8_t)(ES8388_MSC_BIT | ES8388_BCLKDIV_MCLK_DIV_8);
    } else {
        clock_reg = 0x00U;
    }
    es8388_write_reg(0x08, clock_reg);
}
static void es8388_attach_record_mclk(uint32_t sample_rate)
{
    g_es8388_record.iis_obj.sample_rate = sample_rate;
    g_es8388_record.iis_obj.mclk_pin = HORN_MCLK_PIN;
    if (HORN_MCLK_PIN != 0xFFFFFFFF) {
        g_es8388_record.iis_obj.port_mclk = (void *)get_port(HORN_MCLK_PIN);
        gpio_init(HORN_MCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    } else {
        g_es8388_record.iis_obj.port_mclk = NULL;
    }
}

static uint8_t es8388_probe_device(uint8_t dev_addr)
{
    return soft_iic_probe_addr(&es8388_iic_obj, dev_addr);
}

static uint8_t es8388_auto_select_i2c(void)
{
    uint8_t retry = 0;
    uint8_t pin_idx = 0;
    uint8_t addr_idx = 0;
    uint8_t probe_ok = 0;
    const gpio_pin_enum scl_candidates[2] = {ES8388_SCL_PIN, P33_8};
    const gpio_pin_enum sda_candidates[2] = {ES8388_SDA_PIN, P32_4};
    const uint8_t addr_candidates[2] = {ES8388_IIC_ADDR, ES8388_IIC_ADDR_ALT};

#if ES8388_IIC_FORCE_BUS
    g_es8388_scl_pin = ES8388_FORCE_SCL_PIN;
    g_es8388_sda_pin = ES8388_FORCE_SDA_PIN;
    g_es8388_i2c_addr = ES8388_FORCE_IIC_ADDR;
    soft_iic_init(&es8388_iic_obj, g_es8388_i2c_addr, ES8388_IIC_DELAY, g_es8388_scl_pin, g_es8388_sda_pin);

    probe_ok = es8388_probe_device(g_es8388_i2c_addr);
    printf("[ES8388] FORCE I2C: SCL=P%d_%d SDA=P%d_%d ADDR=0x%02X ACK=%u\r\n",
           (g_es8388_scl_pin / 32), (g_es8388_scl_pin % 32),
           (g_es8388_sda_pin / 32), (g_es8388_sda_pin % 32),
           g_es8388_i2c_addr,
           probe_ok);
    return probe_ok;
#else
    for (retry = 0; retry < ES8388_IIC_PROBE_RETRY; retry++)
    {
#if ES8388_IIC_VERBOSE_LOG
        printf("[ES8388] I2C detect round %u/%u\r\n", (unsigned)(retry + 1U), (unsigned)ES8388_IIC_PROBE_RETRY);
#endif

        for (pin_idx = 0; pin_idx < 2; pin_idx++)
        {
            soft_iic_init(&es8388_iic_obj, 0x00, ES8388_IIC_DELAY, scl_candidates[pin_idx], sda_candidates[pin_idx]);

            for (addr_idx = 0; addr_idx < (uint8_t)(sizeof(addr_candidates) / sizeof(addr_candidates[0])); addr_idx++)
            {
                probe_ok = es8388_probe_device(addr_candidates[addr_idx]);
#if ES8388_IIC_VERBOSE_LOG
                printf("[ES8388] probe SCL=P%d_%d SDA=P%d_%d ADDR=0x%02X -> ACK=%u\r\n",
                       (scl_candidates[pin_idx] / 32), (scl_candidates[pin_idx] % 32),
                       (sda_candidates[pin_idx] / 32), (sda_candidates[pin_idx] % 32),
                       addr_candidates[addr_idx],
                       probe_ok);
#endif
                if (probe_ok)
                {
                    g_es8388_scl_pin = scl_candidates[pin_idx];
                    g_es8388_sda_pin = sda_candidates[pin_idx];
                    g_es8388_i2c_addr = addr_candidates[addr_idx];
                    // Re-bind the runtime I2C object to the detected slave address.
                    // During probing we temporarily switch addr and restore old addr (often 0x00),
                    // so we must set the final address explicitly for subsequent register R/W.
                    soft_iic_init(&es8388_iic_obj, g_es8388_i2c_addr, ES8388_IIC_DELAY, g_es8388_scl_pin, g_es8388_sda_pin);
                    printf("[ES8388] I2C detected on SCL=P%d_%d SDA=P%d_%d ADDR=0x%02X\r\n",
                           (g_es8388_scl_pin / 32), (g_es8388_scl_pin % 32),
                           (g_es8388_sda_pin / 32), (g_es8388_sda_pin % 32),
                           g_es8388_i2c_addr);
                    return 1;
                }
            }
        }

        system_delay_ms(20);
    }

    // 浣跨敤榛樿閰嶇疆锛屼絾鏍囪妫€娴嬪け�?
    g_es8388_scl_pin = ES8388_SCL_PIN;
    g_es8388_sda_pin = ES8388_SDA_PIN;
    g_es8388_i2c_addr = ES8388_IIC_ADDR;
    soft_iic_init(&es8388_iic_obj, g_es8388_i2c_addr, ES8388_IIC_DELAY, g_es8388_scl_pin, g_es8388_sda_pin);
    printf("[ES8388] I2C detect failed, using configured SCL=P%d_%d SDA=P%d_%d ADDR=0x%02X\r\n",
           (g_es8388_scl_pin / 32), (g_es8388_scl_pin % 32),
           (g_es8388_sda_pin / 32), (g_es8388_sda_pin % 32),
           g_es8388_i2c_addr);
    return 0;
#endif
}

// ==================== 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷实锟斤拷 ====================

/**
 * @brief       ES8388锟斤拷始锟斤�?
 */
uint8_t es8388_init(void)
{
    // 锟斤拷始锟斤拷QST锟斤拷锟斤拷I2C锟斤拷锟皆讹拷探锟解常锟斤拷锟斤拷锟斤拷/锟斤拷址锟斤拷锟?
    if (!es8388_auto_select_i2c())
    {
        g_es8388_ready = 0;
        return 1;
    }

    // 锟斤拷锟斤拷位ES8388
    es8388_write_reg(0, 0x80);
    es8388_write_reg(0, 0x00);
    system_delay_ms(100);                  // 锟饺达拷锟斤拷位

    es8388_write_reg(0x01, 0x58);
    es8388_write_reg(0x01, 0x50);
    es8388_write_reg(0x02, 0xF3);
    es8388_write_reg(0x02, 0xF0);

    es8388_write_reg(0x03, 0x09);   // 锟斤拷朔锟狡拷玫锟皆达拷乇�?
    es8388_write_reg(0x00, 0x06);   // 使锟杰参匡拷 500K锟斤拷锟斤拷使锟斤拷
    es8388_write_reg(0x04, 0x00);   // DAC锟斤拷源锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿猴拷通锟斤拷
    es8388_write_reg(0x08, 0x00);   // MCLK锟斤拷锟斤拷�?
    es8388_write_reg(0x2B, 0x80);   // DAC锟斤拷锟斤拷 DACLRC锟斤拷ADCLRC锟斤拷同

    es8388_write_reg(0x09, 0x88);   // ADC L/R PGA锟斤拷锟斤拷锟斤拷锟斤拷�?24dB
    es8388_write_reg(0x0C, 0x4C);   // ADC 锟斤拷锟斤拷选锟斤拷为left data = left ADC, right data = left ADC  锟斤拷频锟斤拷锟斤拷�?6bit
    es8388_write_reg(0x0D, 0x02);   // ADC锟斤拷锟斤拷 MCLK/锟斤拷锟斤拷锟斤�?256
    es8388_write_reg(0x10, 0x00);   // ADC锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡斤拷锟脚猴拷衰锟斤�?L  锟斤拷锟斤拷为锟斤拷小锟斤拷锟斤拷锟斤拷
    es8388_write_reg(0x11, 0x00);   // ADC锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡斤拷锟脚猴拷衰锟斤�?R  锟斤拷锟斤拷为锟斤拷小锟斤拷锟斤拷锟斤拷

    es8388_write_reg(0x17, 0x18);   // DAC 锟斤拷频锟斤拷锟斤拷�?6bit
    es8388_write_reg(0x18, 0x02);   // DAC 锟斤拷锟斤拷 MCLK/锟斤拷锟斤拷锟斤�?256
    es8388_write_reg(0x1A, 0x00);   // DAC锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡斤拷锟脚猴拷衰锟斤�?L  锟斤拷锟斤拷为锟斤拷小锟斤拷锟斤拷锟斤拷
    es8388_write_reg(0x1B, 0x00);   // DAC锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡斤拷锟脚猴拷衰锟斤�?R  锟斤拷锟斤拷为锟斤拷小锟斤拷锟斤拷锟斤拷
    es8388_write_reg(0x27, 0xB8);   // L锟斤拷频锟斤�?
    es8388_write_reg(0x2A, 0xB8);   // R锟斤拷频锟斤�?
    
    system_delay_ms(100);
    
    // 锟斤拷锟斤拷I2S锟斤拷式锟斤拷锟斤拷锟捷筹拷锟斤�?
    es8388_i2s_cfg(ES8388_DEFAULT_I2S_FORMAT, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(0);
    
    // 锟斤拷锟斤拷DAC/ADC
    es8388_adda_cfg(ES8388_DAC_ENABLE, ES8388_ADC_ENABLE);
    
    // 锟斤拷锟斤拷锟斤拷锟酵拷锟?
    es8388_output_cfg(ES8388_OUTPUT_O1_ENABLE, ES8388_OUTPUT_O2_ENABLE);
    
    // 锟斤拷锟斤拷锟斤拷朔锟斤拷锟斤拷�?
    es8388_mic_gain(ES8388_MIC_GAIN);
    
    // 锟斤拷锟斤拷默锟斤拷锟斤拷锟斤拷
    es8388_hpvol_set(ES8388_DEFAULT_HP_VOLUME);
    es8388_spkvol_set(ES8388_DEFAULT_SPK_VOLUME);
    
    // 锟截硷拷锟侥达拷锟斤拷锟截讹拷校锟介，锟斤拷锟解“写失锟杰碉拷锟皆凤拷锟截成癸拷锟斤�?
    if (es8388_read_reg(0x00) == 0xFF)
    {
        g_es8388_ready = 0;
        return 2;
    }

    g_es8388_ready = 1;
    return 0;
}

/**
 * @brief       ES8388写锟侥达拷锟斤拷
 */
uint8_t es8388_write_reg(uint8_t reg, uint8_t val)
{
    // 浣跨敤ES8388鐙珛鐨�?C瀵硅薄鍜屽簱鎻愪緵鐨凙PI
    soft_iic_write_8bit_register(&es8388_iic_obj, reg, val);
    return 0;
}

/**
 * @brief       ES8388锟斤拷锟侥达拷锟斤拷
 */
uint8_t es8388_read_reg(uint8_t reg)
{
    // 浣跨敤ES8388鐙珛鐨�?C瀵硅薄鍜屽簱鎻愪緵鐨凙PI
    return soft_iic_read_8bit_register(&es8388_iic_obj, reg);
}

uint8_t es8388_get_runtime_i2c(uint8_t *addr, gpio_pin_enum *scl_pin, gpio_pin_enum *sda_pin)
{
    if (addr != NULL)
    {
        *addr = g_es8388_i2c_addr;
    }
    if (scl_pin != NULL)
    {
        *scl_pin = g_es8388_scl_pin;
    }
    if (sda_pin != NULL)
    {
        *sda_pin = g_es8388_sda_pin;
    }
    return 0;
}

uint8_t es8388_is_ready(void)
{
    return g_es8388_ready;
}

/**
 * @brief       锟斤拷锟斤拷ES8388锟斤拷锟斤拷模式
 */
void es8388_i2s_cfg(uint8_t fmt, uint8_t len)
{
    fmt &= 0x03;
    len &= 0x07;    /* 锟睫讹拷锟斤拷围 */
    es8388_write_reg(23, (fmt << 1) | (len << 3));  /* R23,ES8388锟斤拷锟斤拷模式锟斤拷锟斤拷 */
}

/**
 * @brief       锟斤拷锟矫讹拷锟斤拷锟斤拷锟斤拷
 */
void es8388_hpvol_set(uint8_t volume)
{
    if (volume > 33)
    {
        volume = 33;
    }
    
    es8388_write_reg(0x2E, volume);
    es8388_write_reg(0x2F, volume);
}

/**
 * @brief       锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
 */
void es8388_spkvol_set(uint8_t volume)
{
    if (volume > 33)
    {
        volume = 33;
    }
    
    es8388_write_reg(0x30, volume);
    es8388_write_reg(0x31, volume);
}

static void es8388_record_i2s_cfg(uint8_t fmt, uint8_t len)
{
    uint8_t adc_fmt = 0x4CU;
    (void)len;

    if (fmt == (uint8_t)SOFT_IIS_FORMAT_LEFT_JUSTIFIED) {
        adc_fmt = 0x4DU;
    } else if (fmt == (uint8_t)SOFT_IIS_FORMAT_RIGHT_JUSTIFIED) {
        adc_fmt = 0x4EU;
    }

    es8388_write_reg(0x2B, 0x80);
    es8388_write_reg(0x0C, adc_fmt);
    es8388_write_reg(0x0D, ES8388_ADCLRCK_DIV_MCLK_DIV_256);
    es8388_write_reg(0x17, 0x18);
    es8388_write_reg(0x18, ES8388_ADCLRCK_DIV_MCLK_DIV_256);
}
/**
 * @brief       锟斤拷锟斤拷3D锟斤拷锟斤拷锟斤�?
 */
void es8388_3d_set(uint8_t depth)
{ 
    depth &= 0x7;       /* 锟睫讹拷锟斤拷围 */
    es8388_write_reg(0x1D, depth << 2);    /* R7,3D锟斤拷锟斤拷锟斤拷锟斤拷 */
}

/**
 * @brief       ES8388 DAC/ADC锟斤拷锟斤拷
 */
void es8388_adda_cfg(uint8_t dacen, uint8_t adcen)
{
    uint8_t tempreg = 0;
    
    tempreg |= ((!dacen) << 0);
    tempreg |= ((!adcen) << 1);
    tempreg |= ((!dacen) << 2);
    tempreg |= ((!adcen) << 3);
    es8388_write_reg(0x02, tempreg);
}

/**
 * @brief       ES8388 DAC锟斤拷锟酵拷锟斤拷锟斤拷锟?
 */
void es8388_output_cfg(uint8_t o1en, uint8_t o2en)
{
    uint8_t tempreg = 0;
    tempreg |= o1en * (3 << 4);
    tempreg |= o2en * (3 << 2);
    es8388_write_reg(0x04, tempreg);
}

/**
 * @brief       ES8388 MIC锟斤拷锟斤拷锟斤拷锟斤拷(MIC PGA锟斤拷锟斤拷)
 */
void es8388_mic_gain(uint8_t gain)
{
    gain &= 0x0F;
    gain |= gain << 4;
    es8388_write_reg(0x09, gain);       /* R9,锟斤拷锟斤拷通锟斤拷PGA锟斤拷锟斤拷锟斤拷锟斤拷 */
}

/**
 * @brief       ES8388 ALC锟斤拷锟斤拷
 */
void es8388_alc_ctrl(uint8_t sel, uint8_t maxgain, uint8_t mingain)
{
    uint8_t tempreg = 0;
    
    tempreg = sel << 6;
    tempreg |= (maxgain & 0x07) << 3;
    tempreg |= mingain & 0x07;
    es8388_write_reg(0x12, tempreg);     /* R18,ALC锟斤拷锟斤拷 */
}

/**
 * @brief       ES8388 ADC锟斤拷锟酵拷锟斤拷锟斤拷锟?
 */
void es8388_input_cfg(uint8_t in)
{
    uint8_t reg = 0x00;
    in &= 0x03;
    reg = (uint8_t)((5U * in) << 4);
    es8388_write_reg(0x0A, reg);   /* ADC1 锟斤拷锟斤拷通锟斤拷选锟斤拷L/R  INPUT1 */
}

// ==================== 锟斤拷锟脚癸拷锟斤拷锟节诧拷锟斤拷锟斤拷 ====================

// 通锟斤拷锟斤拷频锟斤拷锟捷凤拷锟酵猴拷锟斤拷锟斤拷锟斤拷锟矫伙拷锟皆讹拷锟斤拷锟斤拷频锟斤拷锟斤拷使锟矫ｏ拷
static void es8388_send_audio_data(const int16_t *data, uint32_t samples)
{
    if (samples == 0 || data == NULL) return;
    
    uint32_t samples_sent = 0;
    
    while (samples_sent < samples && !es8388_play_stop_request)
    {
        uint32_t batch_size = ES8388_AUDIO_BUFFER_SIZE;
        if (samples_sent + batch_size > samples) {
            batch_size = samples - samples_sent;
        }
        
        // 锟斤拷锟斤拷锟斤拷锟捷碉拷锟斤拷锟斤拷锟斤拷
        for (uint32_t i = 0; i < batch_size; i++) {
            es8388_audio_buffer[i] = data[samples_sent + i];
        }
        
        // 通锟斤拷I2S锟斤拷锟斤拷锟斤拷频锟斤拷锟捷碉拷ES8388
        soft_iis_send_16bit_array(&es8388_iis_tx_obj, es8388_audio_buffer, batch_size, 0);
        
        samples_sent += batch_size;
    }
}

// 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟脚撅拷锟斤拷锟斤拷通锟矫ｏ�?
static void es8388_play_silence(uint32_t duration_ms)
{
    uint32_t samples_needed = duration_ms * HORN_SAMPLE_RATE / 1000;
    
    // 锟斤拷锟姐缓锟斤拷锟斤拷
    for (uint32_t i = 0; i < ES8388_AUDIO_BUFFER_SIZE; i++) {
        es8388_audio_buffer[i] = 0;
    }
    
    uint32_t samples_sent = 0;
    
    while (samples_sent < samples_needed && !es8388_play_stop_request)
    {
        uint32_t batch_size = ES8388_AUDIO_BUFFER_SIZE;
        if (samples_sent + batch_size > samples_needed) {
            batch_size = samples_needed - samples_sent;
        }
        
        // 锟斤拷锟酵撅拷锟斤拷锟斤拷锟斤拷
        soft_iis_send_16bit_array(&es8388_iis_tx_obj, es8388_audio_buffer, batch_size, 0);
        
        samples_sent += batch_size;
    }
}

// ==================== 锟斤拷锟脚癸拷锟杰接匡拷实锟斤拷 ====================

/**
 * @brief       锟斤拷始锟斤拷ES8388锟斤拷频锟斤拷锟斤拷系统
 */
uint8_t es8388_play_init(void)
{
    uint8_t ret;
    
    // 1. 锟斤拷始锟斤拷ES8388锟斤拷通锟斤拷I2C锟斤拷锟矫ｏ�?
    ret = es8388_init();
    if (ret != 0) {
        return ret;  // ES8388锟斤拷始锟斤拷失锟斤�?
    }
    
    // 2. 锟斤拷锟斤拷ES8388锟斤拷I2S锟斤拷式
    es8388_i2s_cfg(ES8388_DEFAULT_I2S_FORMAT, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(0);
    
    // 3. 锟斤拷始锟斤拷IIS为锟斤拷锟斤拷锟斤拷模式
    soft_iis_init_master_tx(&es8388_iis_tx_obj, 
                            HORN_SAMPLE_RATE, 
                            SOFT_IIS_BITS_16, 
                            SOFT_IIS_FORMAT_I2S,
                            HORN_BCK_PIN,   // BCK锟斤拷锟斤拷 -> ES8388 BCLK
                            HORN_WS_PIN,    // WS锟斤拷锟斤拷  -> ES8388 LRCK
                            HORN_SD_PIN,    // SD锟斤拷锟斤拷  -> ES8388 DIN
                            HORN_MCLK_PIN); // MCLK锟斤拷锟斤拷
    
    // 4. 锟斤拷始IIS锟斤拷锟斤拷
    soft_iis_start(&es8388_iis_tx_obj);
    
    // 5. 锟斤拷始锟斤拷状�?
    es8388_play_busy = 0;
    es8388_play_stop_request = 0;
    
    return 0;
}

/**
 * @brief       锟斤拷锟斤拷锟斤拷频锟斤拷锟斤拷锟斤拷锟皆接口ｏ�?
 * @note        锟剿猴拷锟斤拷为锟斤拷锟斤拷锟皆接口ｏ拷实锟绞诧拷锟脚癸拷锟斤拷锟斤拷锟矫伙拷锟皆讹拷锟斤拷锟斤拷频锟斤拷锟斤拷
 *              锟斤拷锟借播锟脚凤拷锟斤拷锟斤拷锟斤拷锟斤拷使锟斤拷horn_simple模锟斤拷锟斤拷锟斤拷锟绞碉拷锟斤拷锟狡碉拷锟斤拷�?
 */
void es8388_play_beep(es8388_play_mode_enum mode)
{
    // 锟斤拷锟斤拷忙状�?
    es8388_play_busy = 1;
    es8388_play_stop_request = 0;
    
    // 注锟解：锟剿达拷锟斤拷实锟街撅拷锟斤拷锟斤拷锟狡碉拷锟斤拷锟?
    // 锟矫伙拷应使锟斤拷es8388_send_audio_data()锟斤拷锟斤拷锟斤拷锟斤拷锟皆讹拷锟斤拷锟斤拷频锟斤拷锟斤�?
    // 锟斤拷使锟斤拷锟斤拷锟斤拷锟斤拷频锟斤拷锟缴库（锟斤拷audio_generator锟斤拷锟斤拷锟斤拷锟斤拷频锟斤拷锟斤�?
    
    // 示锟斤拷锟斤拷锟斤拷锟斤�?锟诫静锟斤拷锟斤拷为占位锟斤�?
    printf("ES8388锟斤拷锟斤拷模式锟斤�?d锟斤拷锟斤拷锟矫伙拷锟皆讹拷锟斤拷锟斤拷频锟斤拷锟捷ｏ拷\r\n", mode);
    es8388_play_silence(1000);
    
    // 锟斤拷锟矫ψ刺?
    es8388_play_busy = 0;
}

/**
 * @brief       停止锟斤拷前锟斤拷锟斤拷
 */
void es8388_play_stop(void)
{
    es8388_play_stop_request = 1;
    
    // 锟饺达拷锟斤拷锟斤拷停止
    while (es8388_play_busy) {
        // 锟斤拷锟斤拷锟斤拷时
        for (volatile int i = 0; i < 1000; i++);
    }
}

/**
 * @brief       锟斤拷椴ワ拷锟斤拷欠锟斤拷锟斤拷诮锟斤拷�?
 */
uint8_t es8388_play_is_busy(void)
{
    return es8388_play_busy;
}

/**
 * @brief       锟斤拷锟矫诧拷锟斤拷锟斤拷锟斤拷
 */
void es8388_set_play_volume(uint8_t volume, uint8_t is_headphone)
{
    if (is_headphone) {
        es8388_hpvol_set(volume);
    } else {
        es8388_spkvol_set(volume);
    }
}

/**
 * @brief       ES8388锟斤拷锟斤拷锟斤拷示
 */
void es8388_play_demo(void)
{
    printf("ES8388锟斤拷锟斤拷锟斤拷示锟斤拷始...\r\n");
    
    // 锟斤拷示锟斤拷锟叫诧拷锟斤拷模式锟斤拷注锟解：锟斤拷要锟矫伙拷锟皆讹拷锟斤拷锟斤拷频锟斤拷锟捷ｏ�?
    printf("锟斤拷示锟斤拷锟斤拷模式1锟斤�?锟斤拷锟斤拷锟斤拷锟絓r\n");
    es8388_play_beep(ES8388_PLAY_BEEP_1S);
    while(es8388_play_is_busy()); // 锟饺达拷锟斤拷锟?
    
    // 锟斤拷锟?锟斤�?
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式2锟斤�?锟斤拷锟斤拷锟斤拷锟絓r\n");
    es8388_play_beep(ES8388_PLAY_BEEP_2S);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式3锟斤�?锟斤拷锟斤拷锟斤拷锟絓r\n");
    es8388_play_beep(ES8388_PLAY_BEEP_3S);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式4锟斤�?锟轿凤拷锟斤拷锟斤拷\r\n");
    es8388_play_beep(ES8388_PLAY_BEEP_2_TIMES);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式5锟斤�?锟轿凤拷锟斤拷锟斤拷\r\n");
    es8388_play_beep(ES8388_PLAY_BEEP_3_TIMES);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式6锟斤�?锟轿凤拷锟斤拷锟斤拷\r\n");
    es8388_play_beep(ES8388_PLAY_BEEP_4_TIMES);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式7锟斤拷锟斤拷锟斤拷锟斤拷\r\n");
    es8388_play_beep(ES8388_PLAY_LONG_SHORT);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式8锟斤拷锟斤拷锟劫凤拷锟斤拷锟斤拷\r\n");
    es8388_play_beep(ES8388_PLAY_RAPID);
    while(es8388_play_is_busy());
    
    printf("锟斤拷锟?锟斤�?..\r\n");
    for(volatile int i = 0; i < 2000000; i++);
    
    printf("锟斤拷示锟斤拷锟斤拷模式9锟斤拷锟斤拷锟斤拷锟斤拷\r\n");
    es8388_play_beep(ES8388_PLAY_ALARM);
    while(es8388_play_is_busy());
    
    printf("ES8388锟斤拷锟斤拷锟斤拷示锟斤拷锟絓r\n");
}

// ==================== 录锟斤拷锟斤拷锟斤拷锟节诧拷锟斤拷锟斤拷 ====================

// 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷I2S锟斤拷锟斤拷
static uint8_t es8388_record_init_iis(void)
{
    // ????????????????????????I2S???????????????
    soft_iis_bits_enum iis_bits;
    switch (g_es8388_record.config.bits_per_sample) {
        case 16: iis_bits = SOFT_IIS_BITS_16; break;
        case 24: iis_bits = SOFT_IIS_BITS_24; break;
        case 32: iis_bits = SOFT_IIS_BITS_32; break;
        default: iis_bits = SOFT_IIS_BITS_16; break;
    }
    // ???????????2S??????????????????
    // ?????????????????????????????????????????CK?????S???????????????SD???????????????P11_9?????S_SDOUT?????
#if ES8388_RECORD_MCU_MASTER
    soft_iis_init_master_rx(&g_es8388_record.iis_obj,
                            g_es8388_record.config.sample_rate,
                            iis_bits,
                            (soft_iis_format_enum)g_es8388_record_iis_format,
                            HORN_BCK_PIN,
                            HORN_WS_PIN,
                            ES8388_ADC_SDOUT_PIN,
                            HORN_MCLK_PIN);
#else
    soft_iis_init_slave_rx(&g_es8388_record.iis_obj,
                           iis_bits,
                           (soft_iis_format_enum)g_es8388_record_iis_format,
                           HORN_BCK_PIN,
                           HORN_WS_PIN,
                           ES8388_ADC_SDOUT_PIN);
    es8388_attach_record_mclk(g_es8388_record.config.sample_rate);
#endif
    printf("[ES8388] record clock=%s\r\n", ES8388_RECORD_MCU_MASTER ? "MCU_MASTER" : "CODEC_MASTER");
    return 0;
}

// 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷ES8388 ADC
static uint8_t es8388_record_config_adc(void)
{
    // 1. 锟饺关憋拷DAC锟斤拷锟斤拷锟斤拷锟斤拷诓锟斤拷牛锟?
    es8388_adda_cfg(1, 1);  // DAC锟斤拷锟矫ｏ拷ADC锟斤拷锟斤拷
    es8388_output_cfg(0, 0);
    es8388_hpvol_set(0);
    es8388_spkvol_set(0);
    
    // 2. 锟斤拷锟斤拷锟斤拷锟斤拷�?
    es8388_input_cfg(g_es8388_record.config.input_source);
    
    // 3. 锟斤拷锟斤拷锟斤拷朔锟斤拷锟斤拷妫拷锟斤拷锟斤拷锟斤拷谢锟饺★拷锟绞癸拷锟侥拷锟街碉拷锟?
    es8388_mic_gain(ES8388_MIC_GAIN);
    es8388_alc_ctrl(0, 0, 0);
    es8388_write_reg(0x10, 0x10);
    es8388_write_reg(0x11, 0x10);
    
    // 4. 锟斤拷锟斤拷I2S锟斤拷式锟斤拷锟诫播锟斤拷使锟斤拷锟斤拷同锟斤拷锟斤拷锟矫ｏ�?
    es8388_record_i2s_cfg(g_es8388_record_iis_format, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(ES8388_RECORD_MCU_MASTER ? 0U : 1U);
    
    return 0;
}

// 锟津单碉拷录锟斤拷锟斤拷锟捷达拷锟斤拷锟斤拷锟斤拷示锟斤拷锟斤拷
static void example_record_processor(int16_t *data, uint32_t samples, void *user_data)
{
    // 示锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷频锟斤拷锟捷碉拷平锟斤拷锟斤拷锟?
    int32_t sum = 0;
    for (uint32_t i = 0; i < samples; i++) {
        sum += (data[i] > 0 ? data[i] : -data[i]);
    }
    int16_t avg_amplitude = (int16_t)(sum / samples);
    
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟接革拷锟洁处锟斤拷锟斤拷锟界：
    // 1. 锟斤拷锟芥到SD锟斤�?
    // 2. 锟斤拷锟斤拷识锟斤拷锟斤�?
    // 3. 锟斤拷频锟斤拷锟斤拷
}

// ==================== 录锟斤拷锟斤拷锟杰接匡拷实锟斤拷 ====================

/**
 * @brief       锟斤拷始锟斤拷ES8388录锟斤拷系统
 */
uint8_t es8388_record_init(es8388_record_config_struct *config)
{
    // 1. 锟斤拷始锟斤拷状�?
    g_es8388_record.state = ES8388_RECORD_STOPPED;
    g_es8388_record.total_samples = 0;
    g_es8388_record.buffer_size = 0;
    
    // 2. 锟斤拷锟斤拷录锟斤拷锟斤拷锟斤拷
    if (config != NULL) {
        g_es8388_record.config = *config;
    } else {
        g_es8388_record.config = DEFAULT_RECORD_CONFIG;
    }
    
    // 3. 确锟斤拷ES8388锟窖筹拷始锟斤拷
    // 注锟解：锟斤拷锟街帮拷丫锟斤拷锟绞硷拷锟斤拷锟斤拷锟斤拷殴锟斤拷埽锟斤拷锟斤拷锊伙拷锟揭拷馗锟斤拷锟绞硷拷锟?
    // 锟斤拷锟斤拷要确锟斤拷ES8388锟斤拷I2C通锟斤拷锟斤拷锟斤拷
    
    // 4. 锟斤拷锟斤拷ES8388 ADC
    uint8_t ret = es8388_record_config_adc();
    if (ret != 0) {
        g_es8388_record.state = ES8388_RECORD_ERROR;
        return ret;
    }
    
    // 5. 锟斤拷始锟斤拷I2S锟斤拷锟斤拷
    ret = es8388_record_init_iis();
    if (ret != 0) {
        g_es8388_record.state = ES8388_RECORD_ERROR;
        return ret;
    }
    
    return 0;
}

/**
 * @brief       锟斤拷始录锟斤拷
 */
uint8_t es8388_record_start(void)
{
    if (g_es8388_record.state == ES8388_RECORD_RUNNING) {
        return 0;  // 锟窖撅拷锟斤拷录锟斤拷锟斤拷
    }
    
    // 1. 锟斤拷锟矫硷拷锟斤拷锟斤�?
    g_es8388_record.total_samples = 0;
    
    // 2. 锟斤拷始I2S锟斤拷锟斤拷
    soft_iis_start(&g_es8388_record.iis_obj);
#if !ES8388_RECORD_MCU_MASTER
    system_delay_ms(20);
    es8388_record_i2s_cfg(g_es8388_record_iis_format, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(1U);
    system_delay_ms(20);
    printf("[ES8388] codec master relatch 0x08=0x%02X 0x0C=0x%02X 0x0D=0x%02X 0x17=0x%02X 0x18=0x%02X 0x2B=0x%02X\r\n",
           es8388_read_reg(0x08),
           es8388_read_reg(0x0C),
           es8388_read_reg(0x0D),
           es8388_read_reg(0x17),
           es8388_read_reg(0x18),
           es8388_read_reg(0x2B));
#else
    es8388_record_warmup_frames(64U);
#endif
    
    // 3. 锟斤拷锟斤拷状�?
    g_es8388_record.state = ES8388_RECORD_RUNNING;
    
    return 0;
}

uint8_t es8388_record_relock_clock(void)
{
#if ES8388_RECORD_MCU_MASTER
    return 0U;
#else
    uint8_t ret;

    printf("[ES8388] record clock relock begin state=%u\r\n",
           (unsigned)g_es8388_record.state);

    ret = es8388_record_config_adc();
    if (ret != 0U)
    {
        g_es8388_record.state = ES8388_RECORD_ERROR;
        return ret;
    }

    if (g_es8388_record.state != ES8388_RECORD_RUNNING)
    {
        soft_iis_start(&g_es8388_record.iis_obj);
        g_es8388_record.state = ES8388_RECORD_RUNNING;
    }

    es8388_record_i2s_cfg(g_es8388_record_iis_format, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(1U);
    system_delay_ms(20);
    es8388_record_i2s_cfg(g_es8388_record_iis_format, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(1U);
    system_delay_ms(20);

    printf("[ES8388] record clock relock ready 0x08=0x%02X 0x02=0x%02X 0x0C=0x%02X 0x0D=0x%02X 0x17=0x%02X 0x18=0x%02X 0x2B=0x%02X\r\n",
           es8388_read_reg(0x08),
           es8388_read_reg(0x02),
           es8388_read_reg(0x0C),
           es8388_read_reg(0x0D),
           es8388_read_reg(0x17),
           es8388_read_reg(0x18),
           es8388_read_reg(0x2B));
    return 0U;
#endif
}

uint8_t es8388_record_recover_clock(void)
{
#if ES8388_RECORD_MCU_MASTER
    return 0U;
#else
    uint8_t ret;

    printf("[ES8388] record clock recover begin state=%u\r\n",
           (unsigned)g_es8388_record.state);

    soft_iis_stop(&g_es8388_record.iis_obj);
    g_es8388_record.state = ES8388_RECORD_STOPPED;

    es8388_set_iis_role(0U);
    es8388_adda_cfg(0U, 0U);
    es8388_output_cfg(0U, 0U);
    system_delay_ms(10);

    es8388_write_reg(0x01, 0x58);
    system_delay_ms(2);
    es8388_write_reg(0x01, 0x50);
    es8388_write_reg(0x02, 0xF3);
    system_delay_ms(2);
    es8388_write_reg(0x02, 0xF0);
    system_delay_ms(5);

    ret = es8388_record_config_adc();
    if (ret != 0U)
    {
        g_es8388_record.state = ES8388_RECORD_ERROR;
        return ret;
    }

    es8388_set_iis_role(0U);
    ret = es8388_record_init_iis();
    if (ret != 0U)
    {
        g_es8388_record.state = ES8388_RECORD_ERROR;
        return ret;
    }

    printf("[ES8388] record clock recover ready 0x08=0x%02X 0x02=0x%02X 0x0C=0x%02X 0x17=0x%02X 0x2B=0x%02X\r\n",
           es8388_read_reg(0x08),
           es8388_read_reg(0x02),
           es8388_read_reg(0x0C),
           es8388_read_reg(0x17),
           es8388_read_reg(0x2B));
    return 0U;
#endif
}

/**
 * @brief       停止录锟斤拷
 */
uint8_t es8388_record_stop(void)
{
    // 1. 停止I2S锟斤拷锟斤拷
    soft_iis_stop(&g_es8388_record.iis_obj);
    
    // 2. 锟斤拷锟斤拷状�?
    g_es8388_record.state = ES8388_RECORD_STOPPED;
    
    return 0;
}

/**
 * @brief       锟斤拷停录锟斤拷
 */
uint8_t es8388_record_stop_keep_clock(void)
{
#if ES8388_RECORD_MCU_MASTER
    return es8388_record_stop();
#else
    g_es8388_record.state = ES8388_RECORD_STOPPED;
    printf("[ES8388] record stop keep clock 0x08=0x%02X 0x0D=0x%02X 0x18=0x%02X\r\n",
           es8388_read_reg(0x08),
           es8388_read_reg(0x0D),
           es8388_read_reg(0x18));
    return 0U;
#endif
}
uint8_t es8388_record_pause(void)
{
    if (g_es8388_record.state != ES8388_RECORD_RUNNING) {
        return 1;  // 锟斤拷锟斤拷锟斤拷锟斤拷状�?
    }
    
    // 停止I2S锟斤拷锟斤拷
    soft_iis_stop(&g_es8388_record.iis_obj);
    
    // 锟斤拷锟斤拷状�?
    g_es8388_record.state = ES8388_RECORD_PAUSED;
    
    return 0;
}

/**
 * @brief       锟斤拷锟斤拷录锟斤拷
 */
uint8_t es8388_record_resume(void)
{
    if (g_es8388_record.state != ES8388_RECORD_PAUSED) {
        return 1;  // 锟斤拷锟斤拷锟斤拷停状�?
    }
    
    // 锟斤拷锟铰匡拷始I2S锟斤拷锟斤拷
    soft_iis_start(&g_es8388_record.iis_obj);
    
    // 锟斤拷锟斤拷状�?
    g_es8388_record.state = ES8388_RECORD_RUNNING;
    
    return 0;
}

/**
 * @brief       锟斤拷取锟斤拷频锟斤拷锟捷ｏ拷锟斤拷锟斤拷式锟斤�?
 */
uint32_t es8388_record_read(int16_t *buffer, uint32_t samples)
{
    if (g_es8388_record.state != ES8388_RECORD_RUNNING) {
        return 0;  // 锟斤拷锟斤拷录锟斤拷状�?
    }
    
    // 锟斤拷锟斤拷通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    uint32_t actual_samples = samples;
    if (g_es8388_record.config.channels == 1) {
        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟斤�?
        int16_t left = 0;
        int16_t right = 0;
        uint32_t i = 0;
        for (i = 0; i < samples; i++) {
            soft_iis_receive_16bit(&g_es8388_record.iis_obj, &left, &right);
            buffer[i] = es8388_pick_mono_sample(left, right);
        }
    } else {
        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷要锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
        // 注锟解：锟斤拷锟斤拷蚧锟斤拷锟斤拷锟街伙拷锟饺★拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷�?
        int16_t left, right;
        for (uint32_t i = 0; i < samples; i++) {
            soft_iis_receive_16bit(&g_es8388_record.iis_obj, &left, &right);
            buffer[i] = left;  // 只锟芥储锟斤拷锟斤拷锟斤�?
        }
    }
    
    // 锟斤拷锟斤拷统锟斤拷
    g_es8388_record.total_samples += actual_samples;
    
    // 锟斤拷锟斤拷谢氐锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷�?
    if (g_record_callback != NULL) {
        g_record_callback(buffer, actual_samples, g_record_user_data);
    }
    
    return actual_samples;
}

/**
 * @brief       锟斤拷取锟斤拷频锟斤拷锟捷ｏ拷锟斤拷锟斤拷锟斤拷式锟斤拷
 */
uint32_t es8388_record_try_read(int16_t *buffer, uint32_t samples)
{
    uint32_t actual_samples = 0;

#if ES8388_RECORD_MCU_MASTER
    return es8388_record_read(buffer, samples);
#endif

    if (g_es8388_record.state != ES8388_RECORD_RUNNING) {
        return 0;
    }

    if (g_es8388_record.config.channels == 1) {
        int16_t left = 0;
        uint32_t i = 0;

        for (i = 0; i < samples; i++) {
            if (!soft_iis_receive_16bit_left_try(&g_es8388_record.iis_obj, &left, ES8388_RECORD_TRY_TIMEOUT_US)) {
                break;
            }
            buffer[i] = left;
        }
        actual_samples = i;
    } else {
        int16_t left = 0;
        int16_t right = 0;
        uint32_t i = 0;
        for (i = 0; i < samples; i++) {
            if (!soft_iis_receive_16bit_try(&g_es8388_record.iis_obj, &left, &right, ES8388_RECORD_TRY_TIMEOUT_US)) {
                break;
            }
            buffer[i] = left;
        }
        actual_samples = i;
    }

    if (actual_samples > 0) {
        g_es8388_record.total_samples += actual_samples;
        if (g_record_callback != NULL) {
            g_record_callback(buffer, actual_samples, g_record_user_data);
        }
    }

    return actual_samples;
}

/**
 * @brief       锟斤拷取录锟斤拷状�?
 */
es8388_record_state_enum es8388_record_get_state(void)
{
    return g_es8388_record.state;
}

/**
 * @brief       锟斤拷取锟斤拷录锟狡碉拷锟斤拷锟斤拷锟斤拷锟斤拷
 */
uint32_t es8388_record_get_total_samples(void)
{
    return g_es8388_record.total_samples;
}

/**
 * @brief       锟斤拷取录锟斤拷时锟斤拷锟斤拷锟诫）
 */
uint32_t es8388_record_get_duration_seconds(void)
{
    if (g_es8388_record.config.sample_rate == 0) {
        return 0;
    }
    return g_es8388_record.total_samples / g_es8388_record.config.sample_rate;
}

/**
 * @brief       锟斤拷锟斤拷锟斤拷朔锟斤拷锟斤拷�?
 */
void es8388_record_set_mic_gain(uint8_t gain)
{
    if (gain > 8) gain = 8;
    es8388_mic_gain(gain);
}

/**
 * @brief       锟斤拷锟斤拷锟斤拷锟斤拷�?
 */
void es8388_record_set_input_source(uint8_t source)
{
    if (source > 3) source = 3;
    g_es8388_record.config.input_source = source;
    es8388_input_cfg(source);
}

void es8388_record_set_iis_format(uint8_t fmt)
{
    uint8_t apply_fmt = es8388_clamp_record_iis_format(fmt);
    uint8_t was_running = (g_es8388_record.state == ES8388_RECORD_RUNNING) ? 1 : 0;

    if (g_es8388_record_iis_format == apply_fmt) {
        return;
    }

    g_es8388_record_iis_format = apply_fmt;
    es8388_record_i2s_cfg(g_es8388_record_iis_format, ES8388_DEFAULT_DATA_LEN);
    es8388_set_iis_role(ES8388_RECORD_MCU_MASTER ? 0U : 1U);

    if ((g_es8388_record.state == ES8388_RECORD_RUNNING) || (g_es8388_record.state == ES8388_RECORD_PAUSED)) {
        soft_iis_stop(&g_es8388_record.iis_obj);
        (void)es8388_record_init_iis();
        if (was_running) {
            soft_iis_start(&g_es8388_record.iis_obj);
        }
    }
}

uint8_t es8388_record_get_iis_format(void)
{
    return g_es8388_record_iis_format;
}

uint8_t es8388_record_override_ws_pin(gpio_pin_enum ws_pin)
{
    uint8_t was_running = (g_es8388_record.state == ES8388_RECORD_RUNNING) ? 1U : 0U;

    if ((gpio_pin_enum)g_es8388_record.iis_obj.ws_pin == ws_pin)
    {
        return 0;
    }

    if ((g_es8388_record.state == ES8388_RECORD_RUNNING) || (g_es8388_record.state == ES8388_RECORD_PAUSED))
    {
        soft_iis_stop(&g_es8388_record.iis_obj);
    }

    g_es8388_record.iis_obj.ws_pin = ws_pin;
    g_es8388_record.iis_obj.port_ws = (void *)get_port(ws_pin);
    gpio_init(ws_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);

    if (was_running)
    {
        soft_iis_start(&g_es8388_record.iis_obj);
    }

    printf("[ES8388] record ws override=P%d_%d\r\n",
           (unsigned)(ws_pin / 32),
           (unsigned)(ws_pin % 32));
    return 0;
}
/**
 * @brief       锟斤拷锟斤拷录锟斤拷锟截碉拷锟斤拷锟斤拷
 */
void es8388_record_set_callback(es8388_record_callback_t callback, void *user_data)
{
    g_record_callback = callback;
    g_record_user_data = user_data;
}

/**
 * @brief       录锟斤拷锟斤拷示锟斤拷锟斤拷
 */
void es8388_record_demo(void)
{
    printf("ES8388录锟斤拷锟斤拷示锟斤拷始...\r\n");
    
    // 1. 锟斤拷始锟斤拷录锟斤拷系�?
    uint8_t ret = es8388_record_init(NULL);
    if (ret != 0) {
        printf("录锟斤拷系统锟斤拷始锟斤拷失锟斤�? %d\r\n", ret);
        return;
    }
    printf("录锟斤拷系统锟斤拷始锟斤拷锟缴癸拷\r\n");
    
    // 2. 锟斤拷锟矫回碉拷锟斤拷锟斤拷锟斤拷锟斤拷选锟斤拷
    es8388_record_set_callback(example_record_processor, NULL);
    
    // 3. 锟斤拷始录锟斤拷
    ret = es8388_record_start();
    if (ret != 0) {
        printf("锟斤拷始录锟斤拷失锟斤拷: %d\r\n", ret);
        return;
    }
    printf("锟斤拷始录锟斤拷...\r\n");
    
    // 4. 录锟斤拷5锟斤拷锟斤拷
    printf("录锟斤拷5锟斤拷锟斤拷�?..\r\n");
    int16_t audio_buffer[512];
    uint32_t total_recorded = 0;
    uint32_t target_samples = 5 * g_es8388_record.config.sample_rate;
    
    while (total_recorded < target_samples) {
        // 每锟轿讹拷取512锟斤拷锟斤拷锟斤�?
        uint32_t samples = es8388_record_read(audio_buffer, 512);
        total_recorded += samples;
        
        // 锟斤拷示锟斤拷锟斤拷
        if (total_recorded % (g_es8388_record.config.sample_rate) == 0) {
            uint32_t seconds = total_recorded / g_es8388_record.config.sample_rate;
            printf("锟斤拷录锟斤�? %u 锟斤拷\r\n", seconds);
        }
    }
    
    // 5. 停止录锟斤拷
    es8388_record_stop();
    printf("录锟斤拷锟斤拷锟絓r\n");
    
    // 6. 锟斤拷示统锟斤拷锟斤拷息
    printf("录锟斤拷统锟斤拷:\r\n");
    printf("  锟斤拷锟斤拷锟斤拷锟斤拷: %u\r\n", g_es8388_record.total_samples);
    printf("  录锟斤拷时锟斤拷: %u 锟斤拷\r\n", es8388_record_get_duration_seconds());
    printf("  锟斤拷锟斤拷锟斤�? %u Hz\r\n", g_es8388_record.config.sample_rate);
    printf("  锟斤拷锟斤拷位锟斤拷: %u 位\r\n", g_es8388_record.config.bits_per_sample);
    printf("  通锟斤拷锟斤�? %u\r\n", g_es8388_record.config.channels);
    
    printf("ES8388录锟斤拷锟斤拷示锟斤拷锟絓r\n");
}

// ==================== 锟竭硷拷锟斤拷锟斤拷实锟斤拷 ====================

/**
 * @brief       锟叫伙拷锟斤拷锟斤拷模式
 */
uint8_t es8388_switch_mode(uint8_t mode)
{
    if (mode == 0) {
        // 锟叫伙拷锟斤拷锟斤拷锟斤拷模�?
        // 停止录锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟铰硷拷锟斤拷锟?
        if (g_es8388_record.state == ES8388_RECORD_RUNNING) {
            es8388_record_stop();
        }
        
        // 锟截憋拷ADC锟斤拷锟斤拷锟斤拷DAC
        es8388_adda_cfg(1, 0);
        es8388_set_iis_role(0);
        
        // 锟斤拷始锟斤拷I2S锟斤拷锟斤拷
        soft_iis_init_master_tx(&es8388_iis_tx_obj, 
                                HORN_SAMPLE_RATE, 
                                SOFT_IIS_BITS_16, 
                                SOFT_IIS_FORMAT_I2S,
                                HORN_BCK_PIN,
                                HORN_WS_PIN,
                                HORN_SD_PIN,
                                HORN_MCLK_PIN);
        soft_iis_start(&es8388_iis_tx_obj);
        
    } else if (mode == 1) {
        // 锟叫伙拷锟斤拷录锟斤拷模�?
        // 停止锟斤拷锟脚ｏ拷锟斤拷锟斤拷锟斤拷诓锟斤拷牛锟?
        if (es8388_play_busy) {
            es8388_play_stop();
        }
        
        // 锟截憋拷DAC锟斤拷锟斤拷锟斤拷ADC
        es8388_adda_cfg(1, 1);
        es8388_record_i2s_cfg(g_es8388_record_iis_format, ES8388_DEFAULT_DATA_LEN);
        es8388_set_iis_role(ES8388_RECORD_MCU_MASTER ? 0U : 1U);
        
        // 锟斤拷始锟斤拷I2S锟斤拷锟斤拷
#if ES8388_RECORD_MCU_MASTER
        soft_iis_init_master_rx(&g_es8388_record.iis_obj,
                                g_es8388_record.config.sample_rate,
                                SOFT_IIS_BITS_16,
                                (soft_iis_format_enum)g_es8388_record_iis_format,
                                HORN_BCK_PIN,
                                HORN_WS_PIN,
                                ES8388_ADC_SDOUT_PIN,
                                HORN_MCLK_PIN);
#else
        soft_iis_init_slave_rx(&g_es8388_record.iis_obj,
                               SOFT_IIS_BITS_16,
                               (soft_iis_format_enum)g_es8388_record_iis_format,
                               HORN_BCK_PIN,
                               HORN_WS_PIN,
                               ES8388_ADC_SDOUT_PIN);
        es8388_attach_record_mclk(g_es8388_record.config.sample_rate);
#endif
        soft_iis_start(&g_es8388_record.iis_obj);
    }
    
    return 0;
}

/**
 * @brief       锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷示
 */
void es8388_full_demo(void)
{
    printf("========== ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷示锟斤拷始 ==========\r\n");
    
    // 1. 锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤�?
    printf("\r\n=== 锟斤�?锟斤拷锟街ｏ拷ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷始锟斤�?===\r\n");
    if (es8388_init() == 0) {
        printf("ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷锟缴癸拷\r\n");
    } else {
        printf("ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷失锟斤拷\r\n");
        return;
    }
    
    // 锟斤拷时
    for(volatile int i = 0; i < 1000000; i++);
    
    // 2. 锟斤拷示锟斤拷锟脚癸拷锟斤拷
    printf("\r\n=== 锟斤�?锟斤拷锟街ｏ拷ES8388锟斤拷锟脚癸拷锟斤拷锟斤拷示 ===\r\n");
    if (es8388_play_init() == 0) {
        printf("ES8388锟斤拷锟斤拷系统锟斤拷始锟斤拷锟缴癸拷\r\n");
        es8388_play_demo();
        printf("锟斤拷锟脚癸拷锟斤拷锟斤拷示锟斤拷锟絓r\n");
    } else {
        printf("ES8388锟斤拷锟斤拷系统锟斤拷始锟斤拷失锟斤拷\r\n");
    }
    
    // 锟斤拷时
    for(volatile int i = 0; i < 2000000; i++);
    
    // 3. 锟斤拷示录锟斤拷锟斤拷锟斤拷
    printf("\r\n=== 锟斤�?锟斤拷锟街ｏ拷ES8388录锟斤拷锟斤拷锟斤拷锟斤拷示 ===\r\n");
    es8388_record_demo();
    
    // 4. 锟斤拷示模式锟叫伙拷
    printf("\r\n=== 锟斤�?锟斤拷锟街ｏ拷ES8388模式锟叫伙拷锟斤拷示 ===\r\n");
    printf("锟叫伙拷锟斤拷锟斤拷锟斤拷模�?..\r\n");
    es8388_switch_mode(0);
    printf("锟斤拷锟斤拷模式锟斤拷锟斤拷\r\n");
    
    // 锟斤拷时
    for(volatile int i = 0; i < 1000000; i++);
    
    printf("锟叫伙拷锟斤拷录锟斤拷模�?..\r\n");
    es8388_switch_mode(1);
    printf("录锟斤拷模式锟斤拷锟斤拷\r\n");
    
    printf("\r\n========== ES8388锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷示锟斤拷锟斤拷 ==========\r\n");
}




