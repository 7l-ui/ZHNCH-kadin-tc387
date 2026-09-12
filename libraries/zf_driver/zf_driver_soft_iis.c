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

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_pwm.h"
#include "zf_driver_timer.h"
#include "zf_driver_soft_iis.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 延时
// 参数说明     delay           延时次数
// 返回参数     void
// 使用示例     soft_iis_delay(1);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
#define soft_iis_delay(x)  for(uint32 i = x; i--; )

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS GPIO 宏定义
// 参数说明     obj            IIS 对象指针
// 返回参数     void
// 使用示例     soft_iis_gpio_high_bck(iis_obj);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
#define soft_iis_gpio_high_bck(obj)  ((Ifx_P *)(obj)->port_bck)->OMR.U = 1 << (((obj)->bck_pin)&0x1f)
#define soft_iis_gpio_low_bck(obj)   ((Ifx_P *)(obj)->port_bck)->OMR.U = 65536 << (((obj)->bck_pin)&0x1f)
#define soft_iis_gpio_high_ws(obj)   ((Ifx_P *)(obj)->port_ws)->OMR.U = 1 << (((obj)->ws_pin)&0x1f)
#define soft_iis_gpio_low_ws(obj)    ((Ifx_P *)(obj)->port_ws)->OMR.U = 65536 << (((obj)->ws_pin)&0x1f)
#define soft_iis_gpio_high_sd(obj)   ((Ifx_P *)(obj)->port_sd)->OMR.U = 1 << (((obj)->sd_pin)&0x1f)
#define soft_iis_gpio_low_sd(obj)    ((Ifx_P *)(obj)->port_sd)->OMR.U = 65536 << (((obj)->sd_pin)&0x1f)
#define soft_iis_gpio_high_mclk(obj) if((obj)->mclk_pin != 0xFFFFFFFF) ((Ifx_P *)(obj)->port_mclk)->OMR.U = 1 << (((obj)->mclk_pin)&0x1f)
#define soft_iis_gpio_low_mclk(obj)  if((obj)->mclk_pin != 0xFFFFFFFF) ((Ifx_P *)(obj)->port_mclk)->OMR.U = 65536 << (((obj)->mclk_pin)&0x1f)
#define soft_iis_gpio_get_bck_fast(obj) (((((Ifx_P *)(obj)->port_bck)->IN.U) >> (((obj)->bck_pin)&0x1f)) & 0x1U)
#define soft_iis_gpio_get_ws_fast(obj)  (((((Ifx_P *)(obj)->port_ws)->IN.U) >> (((obj)->ws_pin)&0x1f)) & 0x1U)
#define soft_iis_gpio_get_sd_fast(obj)  (((((Ifx_P *)(obj)->port_sd)->IN.U) >> (((obj)->sd_pin)&0x1f)) & 0x1U)

#define SOFT_IIS_MCLK_RATIO               (256U)
#define SOFT_IIS_MCLK_PWM_CLK_HZ          (20000000U)
#define SOFT_IIS_MCLK_PWM_DUTY            (PWM_DUTY_MAX / 2U)
#define SOFT_IIS_MCLK_PWM_MIN_PERIOD      (2U)
#define SOFT_IIS_MASTER_RX_16BIT_EXT_SLOT (0U)
#define SOFT_IIS_MASTER_TX_DELAY          (0U)
#define SOFT_IIS_MASTER_RX_DELAY          (0U)
#define SOFT_IIS_NO_WS_SYNC_BITS          (128U)
#define SOFT_IIS_EDGE_WAIT_LOOPS          (200U)
#define SOFT_IIS_SLAVE_RX_SAMPLE_FALLING  (1U)

static soft_iis_info_struct *g_soft_iis_no_ws_obj = NULL;
static uint8 g_soft_iis_no_ws_synced = 0;
static uint8 g_soft_iis_no_ws_log_once = 0;

static void soft_iis_reset_no_ws_sync(soft_iis_info_struct *iis_obj)
{
    if((NULL == iis_obj) || (g_soft_iis_no_ws_obj == iis_obj))
    {
        g_soft_iis_no_ws_obj = iis_obj;
        g_soft_iis_no_ws_synced = 0;
        g_soft_iis_no_ws_log_once = 0;
    }
}

static uint16 soft_iis_extract_16bit(const uint8 *bits, uint8 offset)
{
    uint16 value = 0;

    for(uint8 i = 0; i < 16; i++)
    {
        value <<= 1;
        value |= (uint16)bits[offset + i];
    }

    return value;
}

static uint8 soft_iis_popcount16(uint16 value)
{
    uint8 count = 0;

    while(value)
    {
        count += (uint8)(value & 1U);
        value >>= 1;
    }

    return count;
}

static uint8 soft_iis_get_mclk_pwm_channel(gpio_pin_enum pin, pwm_channel_enum *channel)
{
    switch (pin)
    {
        case P11_11: *channel = ATOM2_CH6_P11_11; return 1;
        case P13_1:  *channel = ATOM2_CH6_P13_1;  return 1;
        case P20_6:  *channel = ATOM2_CH6_P20_6;  return 1;
        case P32_0:  *channel = ATOM2_CH6_P32_0;  return 1;
        default: return 0;
    }
}

static uint32 soft_iis_calc_mclk_pwm_freq(uint32 sample_rate)
{
    uint32 target_freq = sample_rate * SOFT_IIS_MCLK_RATIO;
    uint32 period = 0;

    if (target_freq == 0)
    {
        target_freq = SOFT_IIS_MCLK_RATIO * 16000U;
    }

    period = (SOFT_IIS_MCLK_PWM_CLK_HZ + target_freq / 2U) / target_freq;
    if (period < SOFT_IIS_MCLK_PWM_MIN_PERIOD)
    {
        period = SOFT_IIS_MCLK_PWM_MIN_PERIOD;
    }

    return SOFT_IIS_MCLK_PWM_CLK_HZ / period;
}

static void soft_iis_start_mclk(soft_iis_info_struct *iis_obj)
{
    pwm_channel_enum pwm_channel = 0;

    if (iis_obj->mclk_pin == 0xFFFFFFFF)
    {
        return;
    }

    if (soft_iis_get_mclk_pwm_channel((gpio_pin_enum)iis_obj->mclk_pin, &pwm_channel))
    {
        uint32 mclk_freq = soft_iis_calc_mclk_pwm_freq(iis_obj->sample_rate);
        pwm_init(pwm_channel, mclk_freq, SOFT_IIS_MCLK_PWM_DUTY);
        printf("[SOFT_IIS] mclk pwm start pin=%u ch=%u fs=%lu mclk=%lu\r\n",
               (unsigned)iis_obj->mclk_pin,
               (unsigned)pwm_channel,
               (unsigned long)iis_obj->sample_rate,
               (unsigned long)mclk_freq);
        return;
    }

    printf("[SOFT_IIS] mclk gpio fallback pin=%u\r\n", (unsigned)iis_obj->mclk_pin);
    soft_iis_gpio_high_mclk(iis_obj);
}

static void soft_iis_stop_mclk(soft_iis_info_struct *iis_obj)
{
    pwm_channel_enum pwm_channel = 0;

    if (iis_obj->mclk_pin == 0xFFFFFFFF)
    {
        return;
    }

    if (soft_iis_get_mclk_pwm_channel((gpio_pin_enum)iis_obj->mclk_pin, &pwm_channel))
    {
        pwm_set_duty(pwm_channel, 0);
        gpio_init((gpio_pin_enum)iis_obj->mclk_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
        printf("[SOFT_IIS] mclk pwm stop pin=%u ch=%u\r\n",
               (unsigned)iis_obj->mclk_pin,
               (unsigned)pwm_channel);
        return;
    }

    soft_iis_gpio_low_mclk(iis_obj);
}

static uint8 soft_iis_wait_ws_level_timeout(soft_iis_info_struct *iis_obj, uint8 level, uint32 timeout_us)
{
    uint32 start_us = system_getval_us();
    while (soft_iis_gpio_get_ws_fast(iis_obj) != level)
    {
        if ((system_getval_us() - start_us) >= timeout_us)
        {
            return 0;
        }
    }
    return 1;
}

static uint8 soft_iis_wait_bck_level_timeout(soft_iis_info_struct *iis_obj, uint8 level, uint32 timeout_us)
{
    uint32 start_us = system_getval_us();
    while (soft_iis_gpio_get_bck_fast(iis_obj) != level)
    {
        if ((system_getval_us() - start_us) >= timeout_us)
        {
            return 0;
        }
    }
    return 1;
}

static uint8 soft_iis_wait_ws_transition_timeout(soft_iis_info_struct *iis_obj, uint8 level, uint32 timeout_us)
{
    uint32 start_us = system_getval_us();

    while (soft_iis_gpio_get_ws_fast(iis_obj) == level)
    {
        if ((system_getval_us() - start_us) >= timeout_us)
        {
            return 0;
        }
    }

    while (soft_iis_gpio_get_ws_fast(iis_obj) != level)
    {
        if ((system_getval_us() - start_us) >= timeout_us)
        {
            return 0;
        }
    }

    return 1;
}

static uint8 soft_iis_get_word_bits(soft_iis_info_struct *iis_obj)
{
    uint8 bits = 16;

    switch(iis_obj->bits)
    {
        case SOFT_IIS_BITS_16: bits = 16; break;
        case SOFT_IIS_BITS_24: bits = 24; break;
        case SOFT_IIS_BITS_32: bits = 32; break;
        default: break;
    }

    return bits;
}

static uint8 soft_iis_get_master_rx_slot_bits(soft_iis_info_struct *iis_obj)
{
    uint8 slot_bits = soft_iis_get_word_bits(iis_obj);

#if SOFT_IIS_MASTER_RX_16BIT_EXT_SLOT
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode && SOFT_IIS_BITS_16 == iis_obj->bits)
    {
        slot_bits = 32;
    }
#endif

    return slot_bits;
}

static void soft_iis_wait_ws_transition(soft_iis_info_struct *iis_obj, uint8 level)
{
    while(soft_iis_gpio_get_ws_fast(iis_obj) == level)
    {
        // wait for current channel to finish
    }

    while(soft_iis_gpio_get_ws_fast(iis_obj) != level)
    {
        // wait for target channel to start
    }
}

static void soft_iis_sync_rx_channel(soft_iis_info_struct *iis_obj, uint8 ws_level)
{
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
        if(ws_level)
        {
            soft_iis_gpio_high_ws(iis_obj);
        }
        else
        {
            soft_iis_gpio_low_ws(iis_obj);
        }
        soft_iis_delay(iis_obj->delay);
        return;
    }

    soft_iis_wait_ws_transition(iis_obj, ws_level);
}

static uint8 soft_iis_sync_rx_channel_timeout(soft_iis_info_struct *iis_obj, uint8 ws_level, uint32 timeout_us)
{
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
        if(ws_level)
        {
            soft_iis_gpio_high_ws(iis_obj);
        }
        else
        {
            soft_iis_gpio_low_ws(iis_obj);
        }
        soft_iis_delay(iis_obj->delay);
        return 1;
    }

    return soft_iis_wait_ws_transition_timeout(iis_obj, ws_level, timeout_us);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送一位数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     bit           要发送的位（0或1）
// 返回参数     void
// 使用示例     soft_iis_send_bit(iis_obj, 1);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void soft_iis_send_bit(soft_iis_info_struct *iis_obj, uint8 bit)
{
    // 根据数据位设置SD线电平
    if(bit)
    {
        soft_iis_gpio_high_sd(iis_obj);
    }
    else
    {
        soft_iis_gpio_low_sd(iis_obj);
    }
    
    soft_iis_delay(iis_obj->delay);
    
    // BCK上升沿（发送数据）
    soft_iis_gpio_high_bck(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // BCK下降沿（准备下一位）
    soft_iis_gpio_low_bck(iis_obj);
    soft_iis_delay(iis_obj->delay);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收一位数据
// 参数说明     *iis_obj       IIS 对象指针
// 返回参数     uint8         接收到的位（0或1）
// 使用示例     bit = soft_iis_receive_bit(iis_obj);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 soft_iis_receive_bit(soft_iis_info_struct *iis_obj)
{
    uint8 bit = 0;

    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
    
    // BCK上升沿（采样数据）
    soft_iis_gpio_high_bck(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 读取SD线电平
    bit = (uint8)soft_iis_gpio_get_sd_fast(iis_obj);
    
    // BCK下降沿
    soft_iis_gpio_low_bck(iis_obj);
    soft_iis_delay(iis_obj->delay);
    }
    else
    {
#if SOFT_IIS_SLAVE_RX_SAMPLE_FALLING
        while(soft_iis_gpio_get_bck_fast(iis_obj) == 0)
        {
            // wait for BCK high
        }

        while(soft_iis_gpio_get_bck_fast(iis_obj) != 0)
        {
            // wait for BCK falling edge
        }
#else
        while(soft_iis_gpio_get_bck_fast(iis_obj) != 0)
        {
            // wait for BCK low
        }

        while(soft_iis_gpio_get_bck_fast(iis_obj) == 0)
        {
            // wait for BCK rising edge
        }
#endif

        bit = (uint8)soft_iis_gpio_get_sd_fast(iis_obj);
    }
    
    return bit;
}

static uint8 soft_iis_receive_bit_timeout(soft_iis_info_struct *iis_obj, uint8 *bit, uint32 timeout_us)
{
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
        soft_iis_gpio_high_bck(iis_obj);
        soft_iis_delay(iis_obj->delay);
        *bit = (uint8)soft_iis_gpio_get_sd_fast(iis_obj);
        soft_iis_gpio_low_bck(iis_obj);
        soft_iis_delay(iis_obj->delay);
        return 1;
    }

#if SOFT_IIS_SLAVE_RX_SAMPLE_FALLING
    if(!soft_iis_wait_bck_level_timeout(iis_obj, 1, timeout_us))
    {
        return 0;
    }

    if(!soft_iis_wait_bck_level_timeout(iis_obj, 0, timeout_us))
    {
        return 0;
    }
#else
    if(!soft_iis_wait_bck_level_timeout(iis_obj, 0, timeout_us))
    {
        return 0;
    }

    if(!soft_iis_wait_bck_level_timeout(iis_obj, 1, timeout_us))
    {
        return 0;
    }
#endif

    *bit = (uint8)soft_iis_gpio_get_sd_fast(iis_obj);
    return 1;
}

static uint8 soft_iis_receive_bit_loop_timeout(soft_iis_info_struct *iis_obj, uint8 *bit)
{
    uint32 loops = SOFT_IIS_EDGE_WAIT_LOOPS;

#if SOFT_IIS_SLAVE_RX_SAMPLE_FALLING
    while(soft_iis_gpio_get_bck_fast(iis_obj) == 0U)
    {
        if(0U == loops--)
        {
            return 0;
        }
    }

    loops = SOFT_IIS_EDGE_WAIT_LOOPS;
    while(soft_iis_gpio_get_bck_fast(iis_obj) != 0U)
    {
        if(0U == loops--)
        {
            return 0;
        }
    }
#else
    while(soft_iis_gpio_get_bck_fast(iis_obj) != 0U)
    {
        if(0U == loops--)
        {
            return 0;
        }
    }

    loops = SOFT_IIS_EDGE_WAIT_LOOPS;
    while(soft_iis_gpio_get_bck_fast(iis_obj) == 0U)
    {
        if(0U == loops--)
        {
            return 0;
        }
    }
#endif

    *bit = (uint8)soft_iis_gpio_get_sd_fast(iis_obj);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送一个数据字（根据位宽）
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     data          要发送的数据
// 返回参数     void
// 使用示例     soft_iis_send_word(iis_obj, 0x1234);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void soft_iis_send_word(soft_iis_info_struct *iis_obj, uint32 data)
{
    uint32 mask = 0;
    uint8 bits_to_send = 0;
    
    // 根据位宽设置掩码和位数
    switch(iis_obj->bits)
    {
        case SOFT_IIS_BITS_16:
            mask = 0x8000;
            bits_to_send = 16;
            break;
        case SOFT_IIS_BITS_24:
            mask = 0x800000;
            bits_to_send = 24;
            break;
        case SOFT_IIS_BITS_32:
            mask = 0x80000000;
            bits_to_send = 32;
            break;
        default:
            bits_to_send = 16;
            mask = 0x8000;
            break;
    }
    
    // 根据格式调整发送顺序
    if(iis_obj->format == SOFT_IIS_FORMAT_I2S)
    {
        // I2S格式：MSB先发送，WS变化前一个时钟
        for(uint8 i = 0; i < bits_to_send; i++)
        {
            soft_iis_send_bit(iis_obj, (data & mask) ? 1 : 0);
            mask >>= 1;
        }
    }
    else if(iis_obj->format == SOFT_IIS_FORMAT_LEFT_JUSTIFIED)
    {
        // 左对齐格式：MSB先发送，WS变化时
        for(uint8 i = 0; i < bits_to_send; i++)
        {
            soft_iis_send_bit(iis_obj, (data & mask) ? 1 : 0);
            mask >>= 1;
        }
    }
    else // SOFT_IIS_FORMAT_RIGHT_JUSTIFIED
    {
        // 右对齐格式：LSB先发送
        mask = 0x1;
        for(uint8 i = 0; i < bits_to_send; i++)
        {
            soft_iis_send_bit(iis_obj, (data & mask) ? 1 : 0);
            mask <<= 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收一个数据字（根据位宽）
// 参数说明     *iis_obj       IIS 对象指针
// 返回参数     uint32        接收到的数据
// 使用示例     data = soft_iis_receive_word(iis_obj);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint32 soft_iis_receive_word(soft_iis_info_struct *iis_obj)
{
    uint32 data = 0;
    uint8 bits_to_receive = 0;
    
    // 根据位宽设置位数
    switch(iis_obj->bits)
    {
        case SOFT_IIS_BITS_16:
            bits_to_receive = 16;
            break;
        case SOFT_IIS_BITS_24:
            bits_to_receive = 24;
            break;
        case SOFT_IIS_BITS_32:
            bits_to_receive = 32;
            break;
        default:
            bits_to_receive = 16;
            break;
    }
    
    // 根据格式调整接收顺序
    if(iis_obj->format == SOFT_IIS_FORMAT_I2S)
    {
        // I2S格式：MSB先接收
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            data <<= 1;
            data |= soft_iis_receive_bit(iis_obj);
        }
    }
    else if(iis_obj->format == SOFT_IIS_FORMAT_LEFT_JUSTIFIED)
    {
        // 左对齐格式：MSB先接收
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            data <<= 1;
            data |= soft_iis_receive_bit(iis_obj);
        }
    }
    else // SOFT_IIS_FORMAT_RIGHT_JUSTIFIED
    {
        // 右对齐格式：LSB先接收
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            data >>= 1;
            data |= (soft_iis_receive_bit(iis_obj) << (bits_to_receive - 1));
        }
    }
    
    return data;
}

static uint8 soft_iis_receive_word_timeout(soft_iis_info_struct *iis_obj, uint32 *data, uint32 timeout_us)
{
    uint32 value = 0;
    uint8 bits_to_receive = soft_iis_get_word_bits(iis_obj);
    uint8 bit = 0;
    (void)timeout_us;

    if(iis_obj->format == SOFT_IIS_FORMAT_I2S || iis_obj->format == SOFT_IIS_FORMAT_LEFT_JUSTIFIED)
    {
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            value <<= 1;
            if(!soft_iis_receive_bit_loop_timeout(iis_obj, &bit))
            {
                return 0;
            }
            value |= bit;
        }
    }
    else
    {
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            if(!soft_iis_receive_bit_loop_timeout(iis_obj, &bit))
            {
                return 0;
            }
            value >>= 1;
            value |= ((uint32)bit << (bits_to_receive - 1));
        }
    }

    *data = value;
    return 1;
}

static void soft_iis_discard_master_rx_padding(soft_iis_info_struct *iis_obj)
{
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
        uint8 data_bits = soft_iis_get_word_bits(iis_obj);
        uint8 slot_bits = soft_iis_get_master_rx_slot_bits(iis_obj);
        if(iis_obj->format == SOFT_IIS_FORMAT_I2S && slot_bits > 0U)
        {
            slot_bits--;
        }

        while(slot_bits > data_bits)
        {
            (void)soft_iis_receive_bit(iis_obj);
            slot_bits--;
        }
    }
}

static uint8 soft_iis_discard_master_rx_padding_timeout(soft_iis_info_struct *iis_obj, uint32 timeout_us)
{
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
        uint8 data_bits = soft_iis_get_word_bits(iis_obj);
        uint8 slot_bits = soft_iis_get_master_rx_slot_bits(iis_obj);
        uint8 bit = 0;
        if(iis_obj->format == SOFT_IIS_FORMAT_I2S && slot_bits > 0U)
        {
            slot_bits--;
        }

        while(slot_bits > data_bits)
        {
            if(!soft_iis_receive_bit_timeout(iis_obj, &bit, timeout_us))
            {
                return 0;
            }
            slot_bits--;
        }
    }

    return 1;
}

static void soft_iis_discard_i2s_delay(soft_iis_info_struct *iis_obj)
{
    if(SOFT_IIS_MODE_MASTER_RX != iis_obj->mode && iis_obj->format == SOFT_IIS_FORMAT_I2S)
    {
        (void)soft_iis_receive_bit(iis_obj);
    }
}

static uint8 soft_iis_discard_i2s_delay_timeout(soft_iis_info_struct *iis_obj, uint32 timeout_us)
{
    uint8 bit = 0;

    if(SOFT_IIS_MODE_MASTER_RX != iis_obj->mode && iis_obj->format == SOFT_IIS_FORMAT_I2S)
    {
        return soft_iis_receive_bit_timeout(iis_obj, &bit, timeout_us);
    }

    return 1;
}

static uint8 soft_iis_receive_word_fast_after_sync(soft_iis_info_struct *iis_obj, uint32 *data)
{
    uint32 value = 0;
    uint8 bits_to_receive = soft_iis_get_word_bits(iis_obj);

    if(iis_obj->format == SOFT_IIS_FORMAT_I2S || iis_obj->format == SOFT_IIS_FORMAT_LEFT_JUSTIFIED)
    {
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            value <<= 1;
            value |= soft_iis_receive_bit(iis_obj);
        }
    }
    else
    {
        for(uint8 i = 0; i < bits_to_receive; i++)
        {
            value >>= 1;
            value |= ((uint32)soft_iis_receive_bit(iis_obj) << (bits_to_receive - 1));
        }
    }

    *data = value;
    return 1;
}

static uint8 soft_iis_discard_padding_fast_after_sync(soft_iis_info_struct *iis_obj)
{
    if(SOFT_IIS_MODE_MASTER_RX == iis_obj->mode)
    {
        soft_iis_discard_master_rx_padding(iis_obj);
    }
    return 1;
}

static uint8 soft_iis_can_fallback_no_ws(soft_iis_info_struct *iis_obj)
{
    (void)iis_obj;
    return 0U;
}

static uint8 soft_iis_sync_no_ws_16bit_timeout(soft_iis_info_struct *iis_obj, uint32 timeout_us)
{
    uint8 raw_bits[SOFT_IIS_NO_WS_SYNC_BITS] = {0};
    uint8 bit = 0;
    uint8 best_offset = 0;
    uint16 best_score = 0xFFFFU;

    for(uint8 i = 0; i < SOFT_IIS_NO_WS_SYNC_BITS; i++)
    {
        if(!soft_iis_receive_bit_timeout(iis_obj, &bit, timeout_us))
        {
            g_soft_iis_no_ws_synced = 0;
            return 0;
        }
        raw_bits[i] = bit;
    }

    for(uint8 offset = 0; offset < 16; offset++)
    {
        uint16 score = 0;

        for(uint8 base = offset; (base + 31U) < SOFT_IIS_NO_WS_SYNC_BITS; base += 32U)
        {
            uint16 left_word = soft_iis_extract_16bit(raw_bits, base);
            uint16 right_word = soft_iis_extract_16bit(raw_bits, (uint8)(base + 16U));
            score += soft_iis_popcount16((uint16)(left_word ^ right_word));
        }

        if(score < best_score)
        {
            best_score = score;
            best_offset = offset;
        }
    }

    for(uint8 i = 0; i < best_offset; i++)
    {
        if(!soft_iis_receive_bit_timeout(iis_obj, &bit, timeout_us))
        {
            g_soft_iis_no_ws_synced = 0;
            return 0;
        }
    }

    g_soft_iis_no_ws_obj = iis_obj;
    g_soft_iis_no_ws_synced = 1;
    if(!g_soft_iis_no_ws_log_once)
    {
        printf("[SOFT_IIS] slave_rx fallback: use BCK-only 16bit sync, ws static=%u\r\n",
               (unsigned)gpio_get_level(iis_obj->ws_pin));
        g_soft_iis_no_ws_log_once = 1;
    }
    return 1;
}

static uint8 soft_iis_receive_frame_no_ws_timeout(soft_iis_info_struct *iis_obj, uint32 *left_data, uint32 *right_data, uint32 timeout_us)
{
    if(g_soft_iis_no_ws_obj != iis_obj)
    {
        g_soft_iis_no_ws_obj = iis_obj;
        g_soft_iis_no_ws_synced = 0;
    }

    if(!g_soft_iis_no_ws_synced)
    {
        if(!soft_iis_sync_no_ws_16bit_timeout(iis_obj, timeout_us))
        {
            return 0;
        }
    }

    if(!soft_iis_receive_word_timeout(iis_obj, left_data, timeout_us))
    {
        g_soft_iis_no_ws_synced = 0;
        return 0;
    }

    if(!soft_iis_receive_word_timeout(iis_obj, right_data, timeout_us))
    {
        g_soft_iis_no_ws_synced = 0;
        return 0;
    }

    return 1;
}

static void soft_iis_receive_frame(soft_iis_info_struct *iis_obj, uint32 *left_data, uint32 *right_data)
{
    soft_iis_sync_rx_channel(iis_obj, 0);
    soft_iis_discard_i2s_delay(iis_obj);
    *left_data = soft_iis_receive_word(iis_obj);
    soft_iis_discard_master_rx_padding(iis_obj);

    soft_iis_sync_rx_channel(iis_obj, 1);
    soft_iis_discard_i2s_delay(iis_obj);
    *right_data = soft_iis_receive_word(iis_obj);
    soft_iis_discard_master_rx_padding(iis_obj);
}

static uint8 soft_iis_receive_frame_timeout(soft_iis_info_struct *iis_obj, uint32 *left_data, uint32 *right_data, uint32 timeout_us)
{
    if(!soft_iis_sync_rx_channel_timeout(iis_obj, 0, timeout_us))
    {
        if(soft_iis_can_fallback_no_ws(iis_obj))
        {
            return soft_iis_receive_frame_no_ws_timeout(iis_obj, left_data, right_data, timeout_us);
        }
        return 0;
    }

    if(!soft_iis_discard_i2s_delay_timeout(iis_obj, timeout_us))
    {
        return 0;
    }
    if(!soft_iis_receive_word_timeout(iis_obj, left_data, timeout_us))
    {
        return 0;
    }
    if(!soft_iis_discard_master_rx_padding_timeout(iis_obj, timeout_us))
    {
        return 0;
    }

    if(!soft_iis_sync_rx_channel_timeout(iis_obj, 1, timeout_us))
    {
        return 0;
    }

    if(!soft_iis_discard_i2s_delay_timeout(iis_obj, timeout_us))
    {
        return 0;
    }
    if(!soft_iis_receive_word_timeout(iis_obj, right_data, timeout_us))
    {
        return 0;
    }
    if(!soft_iis_discard_master_rx_padding_timeout(iis_obj, timeout_us))
    {
        return 0;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 初始化 - 主模式发送
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     sample_rate    采样率 (Hz)
// 参数说明     bits           数据位宽
// 参数说明     format         数据格式
// 参数说明     bck_pin        BCK 引脚
// 参数说明     ws_pin         WS 引脚
// 参数说明     sd_pin         SD 引脚
// 参数说明     mclk_pin       MCLK 引脚（可选，0xFFFFFFFF表示不使用）
// 返回参数     void
// 使用示例     soft_iis_init_master_tx(&iis_obj, 44100, SOFT_IIS_BITS_16, SOFT_IIS_FORMAT_I2S, P00_0, P00_1, P00_2, 0xFFFFFFFF);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_init_master_tx(soft_iis_info_struct *iis_obj, 
                             uint32 sample_rate, 
                             soft_iis_bits_enum bits,
                             soft_iis_format_enum format,
                             gpio_pin_enum bck_pin, 
                             gpio_pin_enum ws_pin, 
                             gpio_pin_enum sd_pin,
                             gpio_pin_enum mclk_pin)
{
    zf_assert(bck_pin != ws_pin && bck_pin != sd_pin && ws_pin != sd_pin);
    
    iis_obj->bck_pin = bck_pin;
    iis_obj->ws_pin = ws_pin;
    iis_obj->sd_pin = sd_pin;
    iis_obj->mclk_pin = mclk_pin;
    
    iis_obj->mode = SOFT_IIS_MODE_MASTER_TX;
    iis_obj->bits = bits;
    iis_obj->format = format;
    iis_obj->sample_rate = sample_rate;
    
    // 计算位时钟频率和延迟
    // 位时钟频率 = 采样率 * 通道数(2) * 位宽
    uint32 bits_per_sample = 0;
    switch(bits)
    {
        case SOFT_IIS_BITS_16: bits_per_sample = 16; break;
        case SOFT_IIS_BITS_24: bits_per_sample = 24; break;
        case SOFT_IIS_BITS_32: bits_per_sample = 32; break;
    }
    iis_obj->bck_freq = sample_rate * 2 * bits_per_sample;
    
    // 简单延迟计算（实际应根据系统时钟调整）
    iis_obj->delay = SOFT_IIS_MASTER_TX_DELAY;

    printf("[SOFT_IIS] init master_tx fs=%lu bck_target=%lu delay=%lu bits=%u fmt=%u\r\n",
           (unsigned long)sample_rate,
           (unsigned long)iis_obj->bck_freq,
           (unsigned long)iis_obj->delay,
           (unsigned)bits,
           (unsigned)format);
    
    // 获取端口地址
    iis_obj->port_bck = (void *)get_port(bck_pin);
    iis_obj->port_ws = (void *)get_port(ws_pin);
    iis_obj->port_sd = (void *)get_port(sd_pin);
    if(mclk_pin != 0xFFFFFFFF)
    {
        iis_obj->port_mclk = (void *)get_port(mclk_pin);
    }
    else
    {
        iis_obj->port_mclk = NULL;
    }
    
    // 初始化引脚
    gpio_init(bck_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);      // BCK输出
    gpio_init(ws_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);       // WS输出
    gpio_init(sd_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);       // SD输出
    
    if(mclk_pin != 0xFFFFFFFF)
    {
        gpio_init(mclk_pin, GPO, GPIO_LOW, GPO_PUSH_PULL); // MCLK输出
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 初始化 - 主模式接收
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     sample_rate    采样率 (Hz)
// 参数说明     bits           数据位宽
// 参数说明     format         数据格式
// 参数说明     bck_pin        BCK 引脚
// 参数说明     ws_pin         WS 引脚
// 参数说明     sd_pin         SD 引脚
// 参数说明     mclk_pin       MCLK 引脚（可选，0xFFFFFFFF表示不使用）
// 返回参数     void
// 使用示例     soft_iis_init_master_rx(&iis_obj, 44100, SOFT_IIS_BITS_16, SOFT_IIS_FORMAT_I2S, P00_0, P00_1, P00_2, 0xFFFFFFFF);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_init_master_rx(soft_iis_info_struct *iis_obj, 
                             uint32 sample_rate, 
                             soft_iis_bits_enum bits,
                             soft_iis_format_enum format,
                             gpio_pin_enum bck_pin, 
                             gpio_pin_enum ws_pin, 
                             gpio_pin_enum sd_pin,
                             gpio_pin_enum mclk_pin)
{
    zf_assert(bck_pin != ws_pin && bck_pin != sd_pin && ws_pin != sd_pin);
    
    iis_obj->bck_pin = bck_pin;
    iis_obj->ws_pin = ws_pin;
    iis_obj->sd_pin = sd_pin;
    iis_obj->mclk_pin = mclk_pin;
    
    iis_obj->mode = SOFT_IIS_MODE_MASTER_RX;
    iis_obj->bits = bits;
    iis_obj->format = format;
    iis_obj->sample_rate = sample_rate;
    
    // 计算位时钟频率和延迟
    uint32 bits_per_sample = soft_iis_get_master_rx_slot_bits(iis_obj);
    iis_obj->bck_freq = sample_rate * 2 * bits_per_sample;
    iis_obj->delay = SOFT_IIS_MASTER_RX_DELAY;
    printf("[SOFT_IIS] init master_rx fs=%lu bck_target=%lu delay=%lu bits=%u fmt=%u\r\n",
           (unsigned long)sample_rate,
           (unsigned long)iis_obj->bck_freq,
           (unsigned long)iis_obj->delay,
           (unsigned)bits,
           (unsigned)format);
    
    // 获取端口地址
    iis_obj->port_bck = (void *)get_port(bck_pin);
    iis_obj->port_ws = (void *)get_port(ws_pin);
    iis_obj->port_sd = (void *)get_port(sd_pin);
    if(mclk_pin != 0xFFFFFFFF)
    {
        iis_obj->port_mclk = (void *)get_port(mclk_pin);
    }
    else
    {
        iis_obj->port_mclk = NULL;
    }
    
    // 初始化引脚
    gpio_init(bck_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);      // BCK输出
    gpio_init(ws_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);       // WS输出
    gpio_init(sd_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);     // SD输入
    
    if(mclk_pin != 0xFFFFFFFF)
    {
        gpio_init(mclk_pin, GPO, GPIO_LOW, GPO_PUSH_PULL); // MCLK输出
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 初始化 - 从模式发送
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     bits           数据位宽
// 参数说明     format         数据格式
// 参数说明     bck_pin        BCK 引脚
// 参数说明     ws_pin         WS 引脚
// 参数说明     sd_pin         SD 引脚
// 返回参数     void
// 使用示例     soft_iis_init_slave_tx(&iis_obj, SOFT_IIS_BITS_16, SOFT_IIS_FORMAT_I2S, P00_0, P00_1, P00_2);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_init_slave_tx(soft_iis_info_struct *iis_obj, 
                            soft_iis_bits_enum bits,
                            soft_iis_format_enum format,
                            gpio_pin_enum bck_pin, 
                            gpio_pin_enum ws_pin, 
                            gpio_pin_enum sd_pin)
{
    zf_assert(bck_pin != ws_pin && bck_pin != sd_pin && ws_pin != sd_pin);
    
    iis_obj->bck_pin = bck_pin;
    iis_obj->ws_pin = ws_pin;
    iis_obj->sd_pin = sd_pin;
    iis_obj->mclk_pin = 0xFFFFFFFF; // 从模式通常不使用MCLK
    
    iis_obj->mode = SOFT_IIS_MODE_SLAVE_TX;
    iis_obj->bits = bits;
    iis_obj->format = format;
    iis_obj->sample_rate = 0; // 从模式采样率由主设备决定
    iis_obj->bck_freq = 0;
    iis_obj->delay = 10;
    
    // 获取端口地址
    iis_obj->port_bck = (void *)get_port(bck_pin);
    iis_obj->port_ws = (void *)get_port(ws_pin);
    iis_obj->port_sd = (void *)get_port(sd_pin);
    iis_obj->port_mclk = NULL;
    
    // 初始化引脚
    gpio_init(bck_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);    // BCK输入
    gpio_init(ws_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);     // WS输入
    gpio_init(sd_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);       // SD输出
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 初始化 - 从模式接收
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     bits           数据位宽
// 参数说明     format         数据格式
// 参数说明     bck_pin        BCK 引脚
// 参数说明     ws_pin         WS 引脚
// 参数说明     sd_pin         SD 引脚
// 返回参数     void
// 使用示例     soft_iis_init_slave_rx(&iis_obj, SOFT_IIS_BITS_16, SOFT_IIS_FORMAT_I2S, P00_0, P00_1, P00_2);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_init_slave_rx(soft_iis_info_struct *iis_obj, 
                            soft_iis_bits_enum bits,
                            soft_iis_format_enum format,
                            gpio_pin_enum bck_pin, 
                            gpio_pin_enum ws_pin, 
                            gpio_pin_enum sd_pin)
{
    zf_assert(bck_pin != ws_pin && bck_pin != sd_pin && ws_pin != sd_pin);
    
    iis_obj->bck_pin = bck_pin;
    iis_obj->ws_pin = ws_pin;
    iis_obj->sd_pin = sd_pin;
    iis_obj->mclk_pin = 0xFFFFFFFF;
    
    iis_obj->mode = SOFT_IIS_MODE_SLAVE_RX;
    iis_obj->bits = bits;
    iis_obj->format = format;
    iis_obj->sample_rate = 0;
    iis_obj->bck_freq = 0;
    iis_obj->delay = 10;
    
    // 获取端口地址
    iis_obj->port_bck = (void *)get_port(bck_pin);
    iis_obj->port_ws = (void *)get_port(ws_pin);
    iis_obj->port_sd = (void *)get_port(sd_pin);
    iis_obj->port_mclk = NULL;
    
    // 初始化引脚
    gpio_init(bck_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);    // BCK输入
    gpio_init(ws_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);     // WS输入
    gpio_init(sd_pin, GPI, GPIO_LOW, GPI_FLOATING_IN);     // SD输入
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送16位立体声数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     left_data      左声道数据
// 参数说明     right_data     右声道数据
// 返回参数     void
// 使用示例     soft_iis_send_16bit(&iis_obj, 0x1234, 0x5678);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_send_16bit(soft_iis_info_struct *iis_obj, int16 left_data, int16 right_data)
{
    // WS低电平表示左声道
    soft_iis_gpio_low_ws(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 发送左声道数据
    soft_iis_send_word(iis_obj, (uint32)left_data);
    
    // WS高电平表示右声道
    soft_iis_gpio_high_ws(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 发送右声道数据
    soft_iis_send_word(iis_obj, (uint32)right_data);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送24位立体声数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     left_data      左声道数据
// 参数说明     right_data     右声道数据
// 返回参数     void
// 使用示例     soft_iis_send_24bit(&iis_obj, 0x123456, 0x789ABC);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_send_24bit(soft_iis_info_struct *iis_obj, int32 left_data, int32 right_data)
{
    // 只取24位有效数据
    uint32 left_24bit = left_data & 0xFFFFFF;
    uint32 right_24bit = right_data & 0xFFFFFF;
    
    // WS低电平表示左声道
    soft_iis_gpio_low_ws(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 发送左声道数据
    soft_iis_send_word(iis_obj, left_24bit);
    
    // WS高电平表示右声道
    soft_iis_gpio_high_ws(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 发送右声道数据
    soft_iis_send_word(iis_obj, right_24bit);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送32位立体声数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     left_data      左声道数据
// 参数说明     right_data     右声道数据
// 返回参数     void
// 使用示例     soft_iis_send_32bit(&iis_obj, 0x12345678, 0x9ABCDEF0);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_send_32bit(soft_iis_info_struct *iis_obj, int32 left_data, int32 right_data)
{
    // WS低电平表示左声道
    soft_iis_gpio_low_ws(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 发送左声道数据
    soft_iis_send_word(iis_obj, (uint32)left_data);
    
    // WS高电平表示右声道
    soft_iis_gpio_high_ws(iis_obj);
    soft_iis_delay(iis_obj->delay);
    
    // 发送右声道数据
    soft_iis_send_word(iis_obj, (uint32)right_data);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收16位立体声数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *left_data     左声道数据指针
// 参数说明     *right_data    右声道数据指针
// 返回参数     void
// 使用示例     soft_iis_receive_16bit(&iis_obj, &left, &right);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_receive_16bit(soft_iis_info_struct *iis_obj, int16 *left_data, int16 *right_data)
{
    uint32 left_word = 0;
    uint32 right_word = 0;

    soft_iis_receive_frame(iis_obj, &left_word, &right_word);

    *left_data = (int16)left_word;
    *right_data = (int16)right_word;
    return;

    if(iis_obj->mode == SOFT_IIS_MODE_MASTER_RX)
    {
        soft_iis_gpio_low_ws(iis_obj);
        soft_iis_delay(iis_obj->delay);

        *left_data = (int16)soft_iis_receive_word(iis_obj);
#if SOFT_IIS_MASTER_RX_16BIT_EXT_SLOT
        if(iis_obj->bits == SOFT_IIS_BITS_16)
        {
            (void)soft_iis_receive_word(iis_obj);
        }
#endif

        soft_iis_gpio_high_ws(iis_obj);
        soft_iis_delay(iis_obj->delay);

        *right_data = (int16)soft_iis_receive_word(iis_obj);
#if SOFT_IIS_MASTER_RX_16BIT_EXT_SLOT
        if(iis_obj->bits == SOFT_IIS_BITS_16)
        {
            (void)soft_iis_receive_word(iis_obj);
        }
#endif
        return;
    }

    // 等待WS变低（左声道开始）
    while(gpio_get_level(iis_obj->ws_pin) == 1)
    {
        // 等待
    }
    
    // 接收左声道数据
    *left_data = (int16)soft_iis_receive_word(iis_obj);
    
    // 等待WS变高（右声道开始）
    while(gpio_get_level(iis_obj->ws_pin) == 0)
    {
        // 等待
    }
    
    // 接收右声道数据
    *right_data = (int16)soft_iis_receive_word(iis_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 尝试接收16位立体声数据（带超时）
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *left_data     左声道数据指针
// 参数说明     *right_data    右声道数据指针
// 参数说明     timeout_us     每半声道等待WS翻转的超时（us）
// 返回参数     uint8          1-成功 0-超时
//-------------------------------------------------------------------------------------------------------------------
uint8 soft_iis_receive_16bit_try(soft_iis_info_struct *iis_obj, int16 *left_data, int16 *right_data, uint32 timeout_us)
{
    uint32 left_word = 0;
    uint32 right_word = 0;

    if(!soft_iis_receive_frame_timeout(iis_obj, &left_word, &right_word, timeout_us))
    {
        return 0;
    }

    *left_data = (int16)left_word;
    *right_data = (int16)right_word;
    return 1;

    if(iis_obj->mode == SOFT_IIS_MODE_MASTER_RX)
    {
        // 主模式下由本端产生时序，主动切换WS，避免等待外部翻转导致阻塞。
        soft_iis_gpio_low_ws(iis_obj);
        soft_iis_delay(iis_obj->delay);
        *left_data = (int16)soft_iis_receive_word(iis_obj);
#if SOFT_IIS_MASTER_RX_16BIT_EXT_SLOT
        if(iis_obj->bits == SOFT_IIS_BITS_16)
        {
            (void)soft_iis_receive_word(iis_obj);
        }
#endif

        soft_iis_gpio_high_ws(iis_obj);
        soft_iis_delay(iis_obj->delay);
        *right_data = (int16)soft_iis_receive_word(iis_obj);
#if SOFT_IIS_MASTER_RX_16BIT_EXT_SLOT
        if(iis_obj->bits == SOFT_IIS_BITS_16)
        {
            (void)soft_iis_receive_word(iis_obj);
        }
#endif
        return 1;
    }

    if (!soft_iis_wait_ws_level_timeout(iis_obj, 0, timeout_us))
    {
        return 0;
    }

    *left_data = (int16)soft_iis_receive_word(iis_obj);

    if (!soft_iis_wait_ws_level_timeout(iis_obj, 1, timeout_us))
    {
        return 0;
    }

    *right_data = (int16)soft_iis_receive_word(iis_obj);
    return 1;
}

void soft_iis_receive_16bit_left(soft_iis_info_struct *iis_obj, int16 *left_data)
{
    uint32 left_word = 0;

    soft_iis_sync_rx_channel(iis_obj, 0);
    soft_iis_discard_i2s_delay(iis_obj);
    left_word = soft_iis_receive_word(iis_obj);
    *left_data = (int16)left_word;
}

uint8 soft_iis_receive_16bit_left_try(soft_iis_info_struct *iis_obj, int16 *left_data, uint32 timeout_us)
{
    uint32 left_word = 0;

    if(!soft_iis_sync_rx_channel_timeout(iis_obj, 0, timeout_us))
    {
        if(soft_iis_can_fallback_no_ws(iis_obj))
        {
            uint32 right_word = 0;
            if(!soft_iis_receive_frame_no_ws_timeout(iis_obj, &left_word, &right_word, timeout_us))
            {
                return 0;
            }
            *left_data = (int16)left_word;
            return 1;
        }
        return 0;
    }

    if(!soft_iis_discard_i2s_delay_timeout(iis_obj, timeout_us))
    {
        return 0;
    }

    if(!soft_iis_receive_word_timeout(iis_obj, &left_word, timeout_us))
    {
        return 0;
    }

    *left_data = (int16)left_word;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收24位立体声数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *left_data     左声道数据指针
// 参数说明     *right_data    右声道数据指针
// 返回参数     void
// 使用示例     soft_iis_receive_24bit(&iis_obj, &left, &right);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_receive_24bit(soft_iis_info_struct *iis_obj, int32 *left_data, int32 *right_data)
{
    uint32 left_word = 0;
    uint32 right_word = 0;

    soft_iis_receive_frame(iis_obj, &left_word, &right_word);

    *left_data = (int32)(left_word & 0xFFFFFF);
    *right_data = (int32)(right_word & 0xFFFFFF);
    return;

    // 等待WS变低（左声道开始）
    while(gpio_get_level(iis_obj->ws_pin) == 1)
    {
        // 等待
    }
    
    // 接收左声道数据
    *left_data = (int32)(soft_iis_receive_word(iis_obj) & 0xFFFFFF);
    
    // 等待WS变高（右声道开始）
    while(gpio_get_level(iis_obj->ws_pin) == 0)
    {
        // 等待
    }
    
    // 接收右声道数据
    *right_data = (int32)(soft_iis_receive_word(iis_obj) & 0xFFFFFF);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收32位立体声数据
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *left_data     左声道数据指针
// 参数说明     *right_data    右声道数据指针
// 返回参数     void
// 使用示例     soft_iis_receive_32bit(&iis_obj, &left, &right);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_receive_32bit(soft_iis_info_struct *iis_obj, int32 *left_data, int32 *right_data)
{
    uint32 left_word = 0;
    uint32 right_word = 0;

    soft_iis_receive_frame(iis_obj, &left_word, &right_word);

    *left_data = (int32)left_word;
    *right_data = (int32)right_word;
    return;

    // 等待WS变低（左声道开始）
    while(gpio_get_level(iis_obj->ws_pin) == 1)
    {
        // 等待
    }
    
    // 接收左声道数据
    *left_data = (int32)soft_iis_receive_word(iis_obj);
    
    // 等待WS变高（右声道开始）
    while(gpio_get_level(iis_obj->ws_pin) == 0)
    {
        // 等待
    }
    
    // 接收右声道数据
    *right_data = (int32)soft_iis_receive_word(iis_obj);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送16位数据数组
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 返回参数     void
// 使用示例     soft_iis_send_16bit_array(&iis_obj, audio_data, 1024, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_send_16bit_array(soft_iis_info_struct *iis_obj, const int16 *data, uint32 len, uint8 stereo)
{
    if(stereo)
    {
        // 立体声：每两个数据为一对（左、右）
        for(uint32 i = 0; i < len; i += 2)
        {
            if(i + 1 < len)
            {
                soft_iis_send_16bit(iis_obj, data[i], data[i + 1]);
            }
            else
            {
                // 最后一个数据是单声道
                soft_iis_send_16bit(iis_obj, data[i], 0);
            }
        }
    }
    else
    {
        // 单声道：每个数据同时发送到左右声道
        for(uint32 i = 0; i < len; i++)
        {
            soft_iis_send_16bit(iis_obj, data[i], data[i]);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送24位数据数组
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 返回参数     void
// 使用示例     soft_iis_send_24bit_array(&iis_obj, audio_data, 1024, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_send_24bit_array(soft_iis_info_struct *iis_obj, const int32 *data, uint32 len, uint8 stereo)
{
    if(stereo)
    {
        for(uint32 i = 0; i < len; i += 2)
        {
            if(i + 1 < len)
            {
                soft_iis_send_24bit(iis_obj, data[i], data[i + 1]);
            }
            else
            {
                soft_iis_send_24bit(iis_obj, data[i], 0);
            }
        }
    }
    else
    {
        for(uint32 i = 0; i < len; i++)
        {
            soft_iis_send_24bit(iis_obj, data[i], data[i]);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 发送32位数据数组
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 返回参数     void
// 使用示例     soft_iis_send_32bit_array(&iis_obj, audio_data, 1024, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_send_32bit_array(soft_iis_info_struct *iis_obj, const int32 *data, uint32 len, uint8 stereo)
{
    if(stereo)
    {
        for(uint32 i = 0; i < len; i += 2)
        {
            if(i + 1 < len)
            {
                soft_iis_send_32bit(iis_obj, data[i], data[i + 1]);
            }
            else
            {
                soft_iis_send_32bit(iis_obj, data[i], 0);
            }
        }
    }
    else
    {
        for(uint32 i = 0; i < len; i++)
        {
            soft_iis_send_32bit(iis_obj, data[i], data[i]);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收16位数据数组
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 返回参数     void
// 使用示例     soft_iis_receive_16bit_array(&iis_obj, audio_data, 1024, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_receive_16bit_array(soft_iis_info_struct *iis_obj, int16 *data, uint32 len, uint8 stereo)
{
    if(stereo)
    {
        for(uint32 i = 0; i < len; i += 2)
        {
            int16 left, right;
            soft_iis_receive_16bit(iis_obj, &left, &right);
            data[i] = left;
            if(i + 1 < len)
            {
                data[i + 1] = right;
            }
        }
    }
    else
    {
        for(uint32 i = 0; i < len; i++)
        {
            int16 left, right;
            soft_iis_receive_16bit(iis_obj, &left, &right);
            data[i] = left; // 只取左声道数据
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 尝试接收16位数据数组（带超时）
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 参数说明     timeout_us     每半声道等待WS翻转的超时（us）
// 返回参数     uint32         成功接收的样本数
//-------------------------------------------------------------------------------------------------------------------
uint32 soft_iis_receive_16bit_array_try(soft_iis_info_struct *iis_obj, int16 *data, uint32 len, uint8 stereo, uint32 timeout_us)
{
    uint32 received = 0;

    if(stereo)
    {
        for(uint32 i = 0; i < len; i += 2)
        {
            int16 left, right;
            if (!soft_iis_receive_16bit_try(iis_obj, &left, &right, timeout_us))
            {
                break;
            }
            data[i] = left;
            received++;
            if(i + 1 < len)
            {
                data[i + 1] = right;
                received++;
            }
        }
    }
    else
    {
        for(uint32 i = 0; i < len; i++)
        {
            int16 left, right;
            if (!soft_iis_receive_16bit_try(iis_obj, &left, &right, timeout_us))
            {
                break;
            }
            data[i] = left; // 只取左声道数据
            received++;
        }
    }

    return received;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收24位数据数组
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 返回参数     void
// 使用示例     soft_iis_receive_24bit_array(&iis_obj, audio_data, 1024, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_receive_24bit_array(soft_iis_info_struct *iis_obj, int32 *data, uint32 len, uint8 stereo)
{
    if(stereo)
    {
        for(uint32 i = 0; i < len; i += 2)
        {
            int32 left, right;
            soft_iis_receive_24bit(iis_obj, &left, &right);
            data[i] = left;
            if(i + 1 < len)
            {
                data[i + 1] = right;
            }
        }
    }
    else
    {
        for(uint32 i = 0; i < len; i++)
        {
            int32 left, right;
            soft_iis_receive_24bit(iis_obj, &left, &right);
            data[i] = left;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 接收32位数据数组
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     *data          数据数组指针
// 参数说明     len            数据长度（样本数）
// 参数说明     stereo         是否立体声（1:立体声，0:单声道）
// 返回参数     void
// 使用示例     soft_iis_receive_32bit_array(&iis_obj, audio_data, 1024, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_receive_32bit_array(soft_iis_info_struct *iis_obj, int32 *data, uint32 len, uint8 stereo)
{
    if(stereo)
    {
        for(uint32 i = 0; i < len; i += 2)
        {
            int32 left, right;
            soft_iis_receive_32bit(iis_obj, &left, &right);
            data[i] = left;
            if(i + 1 < len)
            {
                data[i + 1] = right;
            }
        }
    }
    else
    {
        for(uint32 i = 0; i < len; i++)
        {
            int32 left, right;
            soft_iis_receive_32bit(iis_obj, &left, &right);
            data[i] = left;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 开始传输
// 参数说明     *iis_obj       IIS 对象指针
// 返回参数     void
// 使用示例     soft_iis_start(&iis_obj);
// 备注信息     主模式下启动时钟，从模式下准备接收
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_start(soft_iis_info_struct *iis_obj)
{
    soft_iis_reset_no_ws_sync(iis_obj);
    if(iis_obj->mode == SOFT_IIS_MODE_MASTER_TX || iis_obj->mode == SOFT_IIS_MODE_MASTER_RX)
    {
        // 主模式：启动时钟
        soft_iis_gpio_low_bck(iis_obj);
        soft_iis_gpio_low_ws(iis_obj);
    }
    
    if(iis_obj->mclk_pin != 0xFFFFFFFF)
    {
        soft_iis_start_mclk(iis_obj);
    }
    // 从模式不需要特殊操作
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 停止传输
// 参数说明     *iis_obj       IIS 对象指针
// 返回参数     void
// 使用示例     soft_iis_stop(&iis_obj);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_stop(soft_iis_info_struct *iis_obj)
{
    soft_iis_reset_no_ws_sync(iis_obj);
    if(iis_obj->mode == SOFT_IIS_MODE_MASTER_TX || iis_obj->mode == SOFT_IIS_MODE_MASTER_RX)
    {
        // 主模式：停止时钟
        soft_iis_gpio_low_bck(iis_obj);
        soft_iis_gpio_low_ws(iis_obj);
    }
    
    if(iis_obj->mclk_pin != 0xFFFFFFFF)
    {
        soft_iis_stop_mclk(iis_obj);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 设置采样率
// 参数说明     *iis_obj       IIS 对象指针
// 参数说明     sample_rate    采样率 (Hz)
// 返回参数     void
// 使用示例     soft_iis_set_sample_rate(&iis_obj, 48000);
// 备注信息     仅主模式有效
//-------------------------------------------------------------------------------------------------------------------
void soft_iis_set_sample_rate(soft_iis_info_struct *iis_obj, uint32 sample_rate)
{
    if(iis_obj->mode == SOFT_IIS_MODE_MASTER_TX || iis_obj->mode == SOFT_IIS_MODE_MASTER_RX)
    {
        iis_obj->sample_rate = sample_rate;
        
        // 重新计算位时钟频率
        uint32 bits_per_sample = 0;
        switch(iis_obj->bits)
        {
            case SOFT_IIS_BITS_16: bits_per_sample = 16; break;
            case SOFT_IIS_BITS_24: bits_per_sample = 24; break;
            case SOFT_IIS_BITS_32: bits_per_sample = 32; break;
        }
        iis_obj->bck_freq = sample_rate * 2 * bits_per_sample;
        
        // 根据新频率调整延迟（简单实现）
        if(sample_rate > 96000)
            iis_obj->delay = 2;
        else if(sample_rate > 48000)
            iis_obj->delay = 5;
        else
            iis_obj->delay = 10;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件 IIS 获取采样率
// 参数说明     *iis_obj       IIS 对象指针
// 返回参数     uint32         当前采样率
// 使用示例     rate = soft_iis_get_sample_rate(&iis_obj);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 soft_iis_get_sample_rate(soft_iis_info_struct *iis_obj)
{
    return iis_obj->sample_rate;
}
