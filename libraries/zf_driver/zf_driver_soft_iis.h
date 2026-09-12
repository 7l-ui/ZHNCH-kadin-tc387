/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC387 开源库的一部分
*
* TC387 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_driver_soft_iis
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-03-22       Cline              first version
********************************************************************************************************************/

#ifndef _ZF_DRIVER_SOFT_IIS_H_
#define _ZF_DRIVER_SOFT_IIS_H_

#include "../../libraries/zf_common/zf_common_typedef.h"
#include "../../libraries/zf_driver/zf_driver_gpio.h"

// IIS 模式枚举
typedef enum
{
    SOFT_IIS_MODE_MASTER_TX,        // 主模式发送
    SOFT_IIS_MODE_MASTER_RX,        // 主模式接收
    SOFT_IIS_MODE_SLAVE_TX,         // 从模式发送
    SOFT_IIS_MODE_SLAVE_RX          // 从模式接收
} soft_iis_mode_enum;

// IIS 数据格式枚举
typedef enum
{
    SOFT_IIS_FORMAT_I2S,           // 标准I2S格式 (WS变化前一个时钟发送MSB)
    SOFT_IIS_FORMAT_LEFT_JUSTIFIED, // 左对齐格式 (WS变化时发送MSB)
    SOFT_IIS_FORMAT_RIGHT_JUSTIFIED // 右对齐格式 (WS变化后发送MSB)
} soft_iis_format_enum;

// IIS 数据位宽枚举
typedef enum
{
    SOFT_IIS_BITS_16,              // 16位数据
    SOFT_IIS_BITS_24,              // 24位数据
    SOFT_IIS_BITS_32               // 32位数据
} soft_iis_bits_enum;

// IIS 配置结构体
typedef struct
{
    uint32                  bck_pin;        // BCK/SCK 引脚
    uint32                  ws_pin;         // WS/LRCK 引脚  
    uint32                  sd_pin;         // SD/DATA 引脚
    uint32                  mclk_pin;       // MCLK 引脚 (可选，如不使用设为0xFFFFFFFF)
    
    soft_iis_mode_enum      mode;           // IIS 模式
    soft_iis_format_enum    format;         // 数据格式
    soft_iis_bits_enum      bits;           // 数据位宽
    
    uint32                  sample_rate;    // 采样率 (Hz)
    uint32                  bck_freq;       // 位时钟频率 (Hz)
    uint32                  delay;          // 软件延时计数 (用于控制时序)
    
    void                   *port_bck;       // BCK 端口地址
    void                   *port_ws;        // WS 端口地址
    void                   *port_sd;        // SD 端口地址
    void                   *port_mclk;      // MCLK 端口地址
} soft_iis_info_struct;

//==================================================SOFT_IIS 基础函数====================================================

// IIS 初始化函数
void soft_iis_init_master_tx(soft_iis_info_struct *iis_obj, 
                             uint32 sample_rate, 
                             soft_iis_bits_enum bits,
                             soft_iis_format_enum format,
                             gpio_pin_enum bck_pin, 
                             gpio_pin_enum ws_pin, 
                             gpio_pin_enum sd_pin,
                             gpio_pin_enum mclk_pin);

void soft_iis_init_master_rx(soft_iis_info_struct *iis_obj, 
                             uint32 sample_rate, 
                             soft_iis_bits_enum bits,
                             soft_iis_format_enum format,
                             gpio_pin_enum bck_pin, 
                             gpio_pin_enum ws_pin, 
                             gpio_pin_enum sd_pin,
                             gpio_pin_enum mclk_pin);

void soft_iis_init_slave_tx(soft_iis_info_struct *iis_obj, 
                            soft_iis_bits_enum bits,
                            soft_iis_format_enum format,
                            gpio_pin_enum bck_pin, 
                            gpio_pin_enum ws_pin, 
                            gpio_pin_enum sd_pin);

void soft_iis_init_slave_rx(soft_iis_info_struct *iis_obj, 
                            soft_iis_bits_enum bits,
                            soft_iis_format_enum format,
                            gpio_pin_enum bck_pin, 
                            gpio_pin_enum ws_pin, 
                            gpio_pin_enum sd_pin);

// IIS 数据发送函数
void soft_iis_send_16bit(soft_iis_info_struct *iis_obj, int16 left_data, int16 right_data);
void soft_iis_send_24bit(soft_iis_info_struct *iis_obj, int32 left_data, int32 right_data);
void soft_iis_send_32bit(soft_iis_info_struct *iis_obj, int32 left_data, int32 right_data);

// IIS 数据接收函数
void soft_iis_receive_16bit(soft_iis_info_struct *iis_obj, int16 *left_data, int16 *right_data);
void soft_iis_receive_24bit(soft_iis_info_struct *iis_obj, int32 *left_data, int32 *right_data);
void soft_iis_receive_32bit(soft_iis_info_struct *iis_obj, int32 *left_data, int32 *right_data);
uint8 soft_iis_receive_16bit_try(soft_iis_info_struct *iis_obj, int16 *left_data, int16 *right_data, uint32 timeout_us);
uint8 soft_iis_receive_16bit_left_try(soft_iis_info_struct *iis_obj, int16 *left_data, uint32 timeout_us);
void soft_iis_receive_16bit_left(soft_iis_info_struct *iis_obj, int16 *left_data);

// IIS 批量数据发送/接收
void soft_iis_send_16bit_array(soft_iis_info_struct *iis_obj, const int16 *data, uint32 len, uint8 stereo);
void soft_iis_send_24bit_array(soft_iis_info_struct *iis_obj, const int32 *data, uint32 len, uint8 stereo);
void soft_iis_send_32bit_array(soft_iis_info_struct *iis_obj, const int32 *data, uint32 len, uint8 stereo);

void soft_iis_receive_16bit_array(soft_iis_info_struct *iis_obj, int16 *data, uint32 len, uint8 stereo);
void soft_iis_receive_24bit_array(soft_iis_info_struct *iis_obj, int32 *data, uint32 len, uint8 stereo);
void soft_iis_receive_32bit_array(soft_iis_info_struct *iis_obj, int32 *data, uint32 len, uint8 stereo);
uint32 soft_iis_receive_16bit_array_try(soft_iis_info_struct *iis_obj, int16 *data, uint32 len, uint8 stereo, uint32 timeout_us);

// IIS 控制函数
void soft_iis_start(soft_iis_info_struct *iis_obj);
void soft_iis_stop(soft_iis_info_struct *iis_obj);
void soft_iis_set_sample_rate(soft_iis_info_struct *iis_obj, uint32 sample_rate);
uint32 soft_iis_get_sample_rate(soft_iis_info_struct *iis_obj);

//==================================================SOFT_IIS 基础函数====================================================

#endif
