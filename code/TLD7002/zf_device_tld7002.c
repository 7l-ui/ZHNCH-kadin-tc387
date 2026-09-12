/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_device_tld7002
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.12
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-01-22       Seekfree            first version
********************************************************************************************************************/
/********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  TLD7002驱动模块      单片机管脚
*                  RX                  查看 zf_device_tld7002.h 中 TLD7002_UART_RX 宏定义
*                  HSLIL               查看 zf_device_tld7002.h 中 TLD7002_UART_HLSIL 宏定义
*                  HSLIH               悬空
*                  GPIN0               查看 zf_device_tld7002.h 中 TLD7002_GPIN0_PIN 宏定义
*                  GPIN1               悬空
*                  VCC                 6-10V电源
*                  3V3                 3.3V电源（给模块上的同步电路供电）
*                  GND                 电源地
*                  ------------------------------------
********************************************************************************************************************/

#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "TLD7002FuncLayer.h"

#include "zf_device_tld7002.h"

TLD7002_NetworkInstance_t tld7002_device;
fifo_struct tld7002_fifo;           // 串口接受FIFO
uint8       tld7002_buffer[100];    // 串口接收缓冲区
uint8       tld7002_init_flag;      // TLD7002初始化标志位
uint16      tld7002_duty[16];       // 通道占空比 可设置范围0-10000
static tld7002_diag_result_struct tld7002_diag_result;

// TLD7002的配置数据，配置数据可以使用OTP_Wizard进行设置，保存为文件之后，打开配置文件的末尾可以获得完整的配置数组
uint16 tld7002_otp_reg[40] =
{
    0x0D09, 0x0D0D, 0x0D0D, 0x0D0D, 0x0D0D, 0x0D0D, 0x0D0D, 0x0D0D,
    0x3933, 0x3333, 0x4035, 0x5A51, 0x9966, 0x9980, 0x9999, 0xB3B3,
    0xFFFF, 0x0820, 0x0820, 0x0820, 0x0820, 0x0820, 0x0820, 0x0820,
    0x1020, 0x0000, 0x80CF, 0x094A, 0x0000, 0x2020, 0x0000, 0x0000,
    0x0000, 0x8007, 0x0001, 0x159C, 0x0000, 0x000C, 0x0000, 0xCAFE
};

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002缓冲区发送函数
//  参数说明      buffer          需要发送的数据地址
//  参数说明      length          需要发送的长度
//  返回参数      void
//  使用示例      内部调用无需关心
//-------------------------------------------------------------------------------------------------------------------
void tld7002_send_buffer(uint8 *buffer, uint32 length)
{
    // 清空缓冲区
    fifo_clear(&tld7002_fifo);
    // 发送消息
    uart_write_buffer(TLD7002_UART_INDEX, buffer, length);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002缓冲区读取函数
//  参数说明      buffer          保存数据的地址
//  参数说明      length          需要读取的长度
//  返回参数      boolean         0：读取失败  1：读取成功
//  使用示例      内部调用无需关心
//-------------------------------------------------------------------------------------------------------------------
boolean tld7002_read_buffer(uint8 *buffer, uint32 length)
{
    boolean copy_successful = 0;
    uint8   read_buffer[50];
    uint32  read_size = 0;
    uint32  read_position = 0;

    read_size = sizeof(read_buffer);
    fifo_read_buffer(&tld7002_fifo, read_buffer, &read_size, FIFO_READ_AND_CLEAN);

    if ((NULL != buffer) && (0U != length) && (read_size >= length))
    {
        read_position = read_size - length;
        for(; read_position < read_size; read_position++)
        {
            *buffer++ = read_buffer[read_position];
        }
        copy_successful = 1;
    }
    return copy_successful;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002缓冲区清空读取函数
//  参数说明      void
//  返回参数      void
//  使用示例      内部调用无需关心
//-------------------------------------------------------------------------------------------------------------------
void tld7002_clean_buffer(void)
{
    fifo_clear(&tld7002_fifo);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002 GPIN0电平设置函数
//  参数说明      pin             GPIN0引脚
//  参数说明      state           需要设置的电平状态
//  返回参数      void
//  使用示例      内部调用无需关心
//-------------------------------------------------------------------------------------------------------------------
void tld7002_gpin0_set_level(uint8 state)
{
    gpio_set_level(TLD7002_GPIN0_PIN, state);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002 占空比设置函数
//  参数说明      tld7002_id      芯片ID 默认为1
//  返回参数      void
//  使用示例
//  注意事项      根据定义的串口号，需要到zf_driver_uart.h文件中将对应的RX_BUFFER_SIZE定义修改为256
//-------------------------------------------------------------------------------------------------------------------
void tld7002_set_duty(uint8 tld7002_id)
{
    uint16 err_flag;
    //char send_buff[50];

    err_flag = TLD7002setDutyReadDiag((uint16 *)tld7002_duty, &tld7002_device, tld7002_id);
    if(err_flag)
    {
        //TLD7002_TRX_HWCR_ALL (&tld7002_device, send_buff,  tld7002_id);    /* clear all TLD7002 error flags */
    }
    system_delay_us_register(70);
    TLD7002broadcastDCsync(&tld7002_device);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002 串口回调函数
//  参数说明      void
//  返回参数      void
//  使用示例
//  注意事项      此函数需要再uart1_rx_isr中断内调用
//-------------------------------------------------------------------------------------------------------------------
void tld7002_callback(void)
{
    uint32 temp_data;
    if(tld7002_init_flag)
    {
        temp_data = uart_read_byte(TLD7002_UART_INDEX);
        fifo_write_element(&tld7002_fifo, temp_data);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      TLD7002 初始化函数
//  参数说明      void
//  返回参数      void
//  使用示例
//  注意事项      根据定义的串口号，需要到zf_driver_uart.h文件中将对应的RX_BUFFER_SIZE定义修改为256
//-------------------------------------------------------------------------------------------------------------------

static void tld7002_diag_clear_result(void)
{
    tld7002_diag_result.code = TLD7002_DIAG_OK;
    tld7002_diag_result.otp_emu_ok = 0;
    tld7002_diag_result.init_err = 0;
    tld7002_diag_result.duty_err = 0;
    tld7002_diag_result.vled_err = 0;
    tld7002_diag_result.dts_err = 0;
    tld7002_diag_result.vled_mv = 0;
    tld7002_diag_result.dts_c = 0;
}

const tld7002_diag_result_struct *tld7002_diag_get_result(void)
{
    return &tld7002_diag_result;
}

const char *tld7002_diag_code_string(uint32 code)
{
    if(TLD7002_DIAG_OK == code)
    {
        return "OK";
    }
    if(code & TLD7002_DIAG_OTP_EMU_FAIL)
    {
        return "OTP_EMU_FAIL";
    }
    if(code & TLD7002_DIAG_INIT_FAIL)
    {
        return "INIT_FAIL";
    }
    if(code & TLD7002_DIAG_DUTY_FAIL)
    {
        return "DUTY_DIAG_FAIL";
    }
    if(code & TLD7002_DIAG_VLED_FAIL)
    {
        return "VLED_READ_FAIL";
    }
    if(code & TLD7002_DIAG_DTS_FAIL)
    {
        return "DTS_READ_FAIL";
    }
    return "UNKNOWN";
}

uint32 tld7002_diag_init(uint8 tld7002_id)
{
    tld7002_diag_clear_result();

    fifo_init(&tld7002_fifo, FIFO_DATA_8BIT, tld7002_buffer, sizeof(tld7002_buffer));

    gpio_init(TLD7002_GPIN0_PIN, GPO, 0, GPO_PUSH_PULL);
    uart_init (TLD7002_UART_INDEX, TLD7002_UART_BAUD, TLD7002_UART_RX, TLD7002_UART_HLSIL);
    uart_rx_interrupt(TLD7002_UART_INDEX, 1);
    tld7002_init_flag = 1;

    TLD7002initDrivers(&tld7002_device);

    tld7002_diag_result.otp_emu_ok = OTPemuComplete(tld7002_otp_reg, &tld7002_device, tld7002_id, 100);
    if(0 == tld7002_diag_result.otp_emu_ok)
    {
        tld7002_diag_result.code |= TLD7002_DIAG_OTP_EMU_FAIL;
    }

    tld7002_diag_result.init_err = (uint16)TLD7002initDevice(&tld7002_device, tld7002_id);
    if(0 != tld7002_diag_result.init_err)
    {
        tld7002_diag_result.code |= TLD7002_DIAG_INIT_FAIL;
    }
    // Bring-up mode: do not write 0 duty here, or a failed strict diagnostic will blank the board.

    tld7002_diag_result.vled_err = (uint16)TLD7002readVLED(&tld7002_diag_result.vled_mv, &tld7002_device, tld7002_id);
    if(0 != tld7002_diag_result.vled_err)
    {
        tld7002_diag_result.code |= TLD7002_DIAG_VLED_FAIL;
    }

    tld7002_diag_result.dts_err = (uint16)TLD7002readDTS(&tld7002_diag_result.dts_c, &tld7002_device, tld7002_id);
    if(0 != tld7002_diag_result.dts_err)
    {
        tld7002_diag_result.code |= TLD7002_DIAG_DTS_FAIL;
    }

    return tld7002_diag_result.code;
}
void tld7002_init(void)
{
    (void)tld7002_diag_init(1);
}
