/**
 ****************************************************************************************************
 * @file        es8388_config.h
 * @author      正点原子团队(ALIENTEK) / 逐飞科技
 * @version     V2.0
 * @date        2026-03-23
 * @brief       ES8388 配置文件（TC387平台）
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 * 
 * 配置说明：
 * 1. I2C引脚配置：根据硬件连接设置SCL和SDA引脚
 * 2. I2C地址：ES8388在2-wire模式下为7位地址，由CE(AD0)决定
 * 3. I2C延时：控制I2C通信速度
 * 4. 音频配置：采样率、格式等
 ****************************************************************************************************
 * @attention
 *
 * 请根据实际硬件连接修改以下配置
 *
 ****************************************************************************************************
 */

#ifndef __ES8388_CONFIG_H
#define __ES8388_CONFIG_H

#include "../../libraries/zf_common/zf_common_typedef.h"

// ==================== I2C引脚配置 ====================
// 请根据实际硬件连接修改以下引脚
// 引脚定义格式：P00_0 表示 Port 00, Pin 0

// I2C SCL引脚（时钟）- 对应ES_SCL
#define ES8388_SCL_PIN      P13_3

// I2C SDA引脚（数据）- 对应ES_SDA
#define ES8388_SDA_PIN      P13_2
// ==================== I2C参数配置 ====================

// ES8388的I2C地址（7位地址）：
// CE(AD0)=0 -> 0x10；CE(AD0)=1 -> 0x11
#define ES8388_IIC_ADDR     0x11
#define ES8388_IIC_ADDR_ALT ((uint8)(ES8388_IIC_ADDR ^ 0x01))
// I2C自动探测重试次数
#define ES8388_IIC_PROBE_RETRY   5

// I2C探测详细日志：1开启 0关闭
#define ES8388_IIC_VERBOSE_LOG   1

// 强制使用固定I2C总线（调试用）：1开启 0关闭
#define ES8388_IIC_FORCE_BUS     0

// 强制模式下使用的引脚/地址（7位地址）
#define ES8388_FORCE_SCL_PIN     ES8388_SCL_PIN
#define ES8388_FORCE_SDA_PIN     ES8388_SDA_PIN
#define ES8388_FORCE_IIC_ADDR    ES8388_IIC_ADDR

// I2C延时参数（控制通信速度，值越大速度越慢）
// 0：1370KHz 10：1020KHz 20：757KHz 30: 633KHz  
// 40: 532Khz  50: 448KHz  60: 395KHz  70: 359KHz  
// 80: 324KHz  100: 268KHz 1000：32KHz
#define ES8388_IIC_DELAY    300      // 降低速度，增加兼容性

// ==================== 音频配置 ====================

// 默认采样率（Hz）
#define ES8388_DEFAULT_SAMPLE_RATE   44100

// 默认I2S格式
// 0: 飞利浦标准I2S
// 1: MSB(左对齐)
// 2: LSB(右对齐)
// 3: PCM/DSP
#define ES8388_DEFAULT_I2S_FORMAT    0

// 默认数据长度
// 0: 24bit
// 1: 20bit
// 2: 18bit
// 3: 16bit
// 4: 32bit
#define ES8388_DEFAULT_DATA_LEN      3      // 16bit

// Recording clock ownership:
// 1: MCU drives BCK/WS with soft_iis master-rx, ES8388 outputs ADC data as I2S slave.
// 0: ES8388 drives BCK/WS, MCU receives as soft_iis slave-rx.
#define ES8388_RECORD_MCU_MASTER     0

// ==================== 音量配置 ====================

// 默认耳机音量（0~33）
#define ES8388_DEFAULT_HP_VOLUME     20

// 默认喇叭音量（0~33）
#define ES8388_DEFAULT_SPK_VOLUME    20

// ==================== 功能配置 ====================

// 是否启用DAC（1:启用，0:禁用）
#define ES8388_DAC_ENABLE            1

// 是否启用ADC（1:启用，0:禁用）
#define ES8388_ADC_ENABLE            0

// DAC输出通道配置
// o1en: 通道1使能(1)/禁止(0)
// o2en: 通道2使能(1)/禁止(0)
#define ES8388_OUTPUT_O1_ENABLE      1
#define ES8388_OUTPUT_O2_ENABLE      1

// 麦克风增益设置（0~8，对应0~24dB，3dB/Step）
#define ES8388_MIC_GAIN              4      // 12dB

// ==================== IIS引脚配置 ====================
// 请根据实际硬件连接修改以下引脚定义
// 引脚定义格式：P00_0 表示 Port 00, Pin 0

// BCK (位时钟) 引脚 - 对应ES_SCLK/I2S_BCLK_P11_6
#define HORN_BCK_PIN     P11_6

// WS (字选择/左右声道时钟) 引脚 - 对应ES_LRCK/I2S_LRCK_P11_2
#define HORN_WS_PIN       P11_2

// SD (数据) 引脚 - 对应ES_SDIN/I2S_SDIN_P11_3
#define HORN_SD_PIN       P11_3

// ES8388 ADC data out pin (codec SDOUT -> MCU input).
// Default wiring uses P11_9 on this board.
#define ES8388_ADC_SDOUT_PIN P11_9

// MCLK (主时钟) 引脚 (可选，如不使用设为0xFFFFFFFF)
// 对应ES_MCLK/I2S_MCLK_P11_11
#define HORN_MCLK_PIN     P11_11  // 使用MCLK

// ==================== 音频参数配置 ====================

// 采样率 (Hz)
#define HORN_SAMPLE_RATE  44100

// 鸣笛基准频率 (Hz)
#define HORN_BASE_FREQ    2500

// 警报鸣笛双频 (Hz)
#define HORN_ALARM_FREQ1  1600
#define HORN_ALARM_FREQ2  2500

// 音频振幅 (0-32767)
#define HORN_AMPLITUDE    22000

// ==================== 时间参数配置 ====================

// 单次鸣笛时长 (毫秒)
#define HORN_BEEP_TIME_MS  1000

// 鸣笛间隔时间 (毫秒)
#define HORN_INTERVAL_MS   1000

// 急促鸣笛时长 (毫秒)
#define HORN_RAPID_BEEP_MS 500

// 急促鸣笛间隔 (毫秒)
#define HORN_RAPID_INTERVAL_MS 500

// 急促鸣笛次数
#define HORN_RAPID_COUNT   5

// 警报鸣笛切换时间 (毫秒)
#define HORN_ALARM_SWITCH_MS 1000

// 警报鸣笛次数
#define HORN_ALARM_COUNT   5

#endif /* __ES8388_CONFIG_H */
