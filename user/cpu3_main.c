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
* 文件名称          cpu2_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "cpu0_main.h"
#include "isr.h"
#include "pid_config.h"
#include "../code/yaokong/yaokong.h"
#include "dianji.h"
#include "zf_device_absolute_encoder.h"
#include "zf_driver_encoder.h"

#define CTRL_RT_ABS_ENC_COUNTS_PER_REV           (4096)
#define CTRL_RT_ABS_ENC_COUNTS_TO_DEGREES        (1.0f / 39.0f)
#define CTRL_RT_ENCODER_SPEED_VALID_LIMIT        (16000)
#define CTRL_RT_ENCODER_FILTER_ALPHA             (0.16f)
#define CTRL_RT_ENCODER_SPEED_PERIOD_TICK_10NS   (500000U)
#define CTRL_RT_STEER_FEEDBACK_PERIOD_MS         (2U)
#define CTRL_RT_LOOP_DELAY_MS                    (1U)

#pragma section all "cpu3_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中


// 工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
// 工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
// 一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 enableInterrupts(); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的中断，因此需要我们自己手动调用 enableInterrupts(); 来开启中断的响应。


// **************************** 代码区域 ****************************
static int16 Ctrl3_WrapEncoderDelta(int16 raw, int16 center)
{
    int32 delta = (int32)raw - (int32)center;

    while (delta > (CTRL_RT_ABS_ENC_COUNTS_PER_REV / 2 - 1))
    {
        delta -= CTRL_RT_ABS_ENC_COUNTS_PER_REV;
    }
    while (delta < -(CTRL_RT_ABS_ENC_COUNTS_PER_REV / 2))
    {
        delta += CTRL_RT_ABS_ENC_COUNTS_PER_REV;
    }

    return (int16)delta;
}

static uint8 Ctrl3_RawInFeedbackLimit(int16 raw)
{
    int16 delta_counts = Ctrl3_WrapEncoderDelta(raw, g_steer_center_raw);
    float feedback_deg = (float)delta_counts * CTRL_RT_ABS_ENC_COUNTS_TO_DEGREES;

    return ((feedback_deg <= PID_STEER_FEEDBACK_HARD_LIMIT_DEG) &&
            (feedback_deg >= -PID_STEER_FEEDBACK_HARD_LIMIT_DEG)) ? 1U : 0U;
}

static void Ctrl3_PublishSteerFeedbackRaw(int16 raw, uint32 now_ms, float speed_counts_s)
{
    int16 delta_counts = Ctrl3_WrapEncoderDelta(raw, g_steer_center_raw);

    g_abs_encoder_raw = raw;
    g_steer_delta_counts = delta_counts;
    g_steer_feedback_speed_counts_s = speed_counts_s;
    g_steer_feedback_angle_deg = (float)delta_counts * CTRL_RT_ABS_ENC_COUNTS_TO_DEGREES;
    g_steer_feedback_update_ms = now_ms;
    g_steer_feedback_valid = 1U;
}

static void Ctrl3_UpdateEncoderFeedback(uint32 now_tick_10ns)
{
    static uint8 initialized = 0U;
    static uint32 last_speed_tick_10ns = 0U;
    static int16 last_ecod1_count = 0;
    static int16 last_ecod2_count = 0;
    int16 ecod1_count;
    int16 ecod2_count;

    ecod1_count = encoder_get_count(TIM2_ENCODER);
    ecod2_count = encoder_get_count(TIM4_ENCODER);
    g_ecod1_count = ecod1_count;
    g_ecod2_count = ecod2_count;

    if (0U == initialized)
    {
        initialized = 1U;
        last_speed_tick_10ns = now_tick_10ns;
        last_ecod1_count = ecod1_count;
        last_ecod2_count = ecod2_count;
        return;
    }

    if ((uint32)(now_tick_10ns - last_speed_tick_10ns) >= CTRL_RT_ENCODER_SPEED_PERIOD_TICK_10NS)
    {
        uint32 dt_tick_10ns = now_tick_10ns - last_speed_tick_10ns;
        int32 delta1 = (int32)((int16)(ecod1_count - last_ecod1_count));
        int32 delta2 = (int32)((int16)(ecod2_count - last_ecod2_count));
        int32 speed1 = (int32)(((int64)delta1 * 100000000LL) / (int64)dt_tick_10ns);
        int32 speed2 = (int32)(((int64)delta2 * 100000000LL) / (int64)dt_tick_10ns);

        if ((speed1 > CTRL_RT_ENCODER_SPEED_VALID_LIMIT) ||
            (speed1 < -CTRL_RT_ENCODER_SPEED_VALID_LIMIT))
        {
            speed1 = g_ecod1_speed;
        }
        if ((speed2 > CTRL_RT_ENCODER_SPEED_VALID_LIMIT) ||
            (speed2 < -CTRL_RT_ENCODER_SPEED_VALID_LIMIT))
        {
            speed2 = g_ecod2_speed;
        }

        g_ecod1_speed = (int16)speed1;
        g_ecod2_speed = (int16)speed2;
        g_ecod1_speed_filt = (int16)((float)g_ecod1_speed_filt +
                                     CTRL_RT_ENCODER_FILTER_ALPHA *
                                     ((float)speed1 - (float)g_ecod1_speed_filt));
        g_ecod2_speed_filt = (int16)((float)g_ecod2_speed_filt +
                                     CTRL_RT_ENCODER_FILTER_ALPHA *
                                     ((float)speed2 - (float)g_ecod2_speed_filt));

        last_ecod1_count = ecod1_count;
        last_ecod2_count = ecod2_count;
        last_speed_tick_10ns = now_tick_10ns;
    }
}

static int Ctrl3_UpdateSteerFeedback(void)
{
    static uint8 have_last_valid_raw = 0U;
    static int16 last_valid_raw = 0;
    static uint32 last_valid_update_ms = 0U;
    static uint8 raw_abnormal_fault_count = 0U;
    int16 raw_angle;
    int16 raw_jump_counts = 0;
    int abnormal_ret = 0;
    uint8 raw_reject_reason = CTRL_STEER_RAW_REJECT_NONE;
    float feedback_speed_counts_s = 0.0f;
    uint32 now_ms;

    if (0U == g_abs_encoder_ready)
    {
        g_steer_feedback_valid = 0U;
        g_steer_feedback_speed_counts_s = 0.0f;
        have_last_valid_raw = 0U;
        last_valid_update_ms = 0U;
        raw_abnormal_fault_count = 0U;
        return -1;
    }

    raw_angle = absolute_encoder_get_location();
    g_abs_encoder_raw = raw_angle;
    g_steer_raw_read = raw_angle;
    g_steer_raw_last_valid = (have_last_valid_raw != 0U) ?
                             last_valid_raw : 0;
    now_ms = system_getval_ms();
    if (0U != have_last_valid_raw)
    {
        raw_jump_counts = Ctrl3_WrapEncoderDelta(raw_angle, last_valid_raw);
    }
    g_steer_raw_jump_counts = raw_jump_counts;

#if PID_STEER_SENSOR_PROTECT_ENABLE
    if (raw_angle == PID_STEER_RAW_INVALID_VALUE)
    {
        abnormal_ret = -2;
        raw_reject_reason = CTRL_STEER_RAW_REJECT_INVALID;
    }
    else if (0U == Ctrl3_RawInFeedbackLimit(raw_angle))
    {
        abnormal_ret = -4;
        raw_reject_reason = CTRL_STEER_RAW_REJECT_RANGE;
    }

    if ((0 == abnormal_ret) && (0U != have_last_valid_raw))
    {
        if ((raw_jump_counts > PID_STEER_RAW_JUMP_LIMIT_COUNTS) ||
            (raw_jump_counts < -PID_STEER_RAW_JUMP_LIMIT_COUNTS))
        {
            abnormal_ret = -3;
            raw_reject_reason = CTRL_STEER_RAW_REJECT_JUMP;
        }
    }

    if (0 != abnormal_ret)
    {
        if (raw_abnormal_fault_count < 255U)
        {
            raw_abnormal_fault_count++;
        }
        g_steer_raw_last_reject_reason = raw_reject_reason;
        g_steer_raw_last_reject_streak = raw_abnormal_fault_count;
        g_steer_raw_reject_count++;
        g_steer_raw_last_reject_ms = now_ms;
        g_steer_raw_last_reject_raw = raw_angle;
        g_steer_raw_last_reject_ref_raw =
            (have_last_valid_raw != 0U) ? last_valid_raw : 0;
        g_steer_raw_last_reject_jump_counts = raw_jump_counts;

        if ((0U != have_last_valid_raw) &&
            (raw_abnormal_fault_count < PID_STEER_RAW_ABNORMAL_FAULT_COUNT))
        {
            Ctrl3_PublishSteerFeedbackRaw(last_valid_raw, now_ms, 0.0f);
            return (int)last_valid_raw;
        }

        if (0U != have_last_valid_raw)
        {
            g_abs_encoder_raw = last_valid_raw;
        }
        g_steer_feedback_valid = 0U;
        g_steer_feedback_speed_counts_s = 0.0f;
        if (raw_abnormal_fault_count >= PID_STEER_RAW_ABNORMAL_FAULT_COUNT)
        {
            g_steer_sensor_fault = 1U;
        }
        return abnormal_ret;
    }
#else
    (void)raw_jump_counts;
#endif

    raw_abnormal_fault_count = 0U;

    if ((0U != have_last_valid_raw) && (now_ms != last_valid_update_ms))
    {
        uint32 dt_ms = now_ms - last_valid_update_ms;
        feedback_speed_counts_s = ((float)raw_jump_counts * 1000.0f) / (float)dt_ms;
    }

    last_valid_raw = raw_angle;
    last_valid_update_ms = now_ms;
    have_last_valid_raw = 1U;
    g_steer_raw_last_valid = raw_angle;

    Ctrl3_PublishSteerFeedbackRaw(raw_angle, now_ms, feedback_speed_counts_s);
    return (int)raw_angle;
}

void core3_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等




    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // wait all cores ready
    ctrl_main_loop_heartbeat();
    while (TRUE)
    {
        static uint32 last_steer_feedback_ms = 0U;
        uint32 now_ms = system_getval_ms();

        Ctrl3_UpdateEncoderFeedback(system_getval());
        if ((0U == last_steer_feedback_ms) ||
            ((uint32)(now_ms - last_steer_feedback_ms) >= CTRL_RT_STEER_FEEDBACK_PERIOD_MS))
        {
            last_steer_feedback_ms = now_ms;
            (void)Ctrl3_UpdateSteerFeedback();
        }
        dianji_task();
        ctrl_main_loop_heartbeat();
        system_delay_ms(CTRL_RT_LOOP_DELAY_MS);

        // 此处编写需要循环执行的代码




        // 此处编写需要循环执行的代码
    }
}



#pragma section all restore
//by ahstu zhugeliang ltl
