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
* 文件名称          isr
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


#ifndef _isr_h
#define _isr_h

#include "zf_common_headfile.h"


typedef struct
{
    uint8 use_pid;
    uint8 steer_closed_loop;
    uint8 direct_pwm;
    float target_speed_mps;
    float target_angle_deg;
    int16 pwm_duty;
    uint8 steer_pwm_only;
    uint8 brake_to_stop;
} ctrl_cmd_t;

#define CTRL_STOP_REASON_NONE             (0U)
#define CTRL_STOP_REASON_SAFETY_PAUSE     (1U)
#define CTRL_STOP_REASON_MAIN_LOOP_STALL  (2U)
#define CTRL_STOP_REASON_STEER_SENSOR     (3U)
#define CTRL_STOP_REASON_STEER_STALL      (4U)
#define CTRL_STOP_REASON_REMOTE_MOTOR_OFF (5U)
#define CTRL_STOP_REASON_CMD_STALE        (6U)

#define CTRL_PID_RESET_REASON_NONE        (0U)
#define CTRL_PID_RESET_REASON_SAFETY_PAUSE (1U)
#define CTRL_PID_RESET_REASON_MAIN_LOOP_STALL (2U)
#define CTRL_PID_RESET_REASON_CMD_STALE   (3U)
#define CTRL_PID_RESET_REASON_STEER_SENSOR (4U)
#define CTRL_PID_RESET_REASON_STEER_STALL (5U)
#define CTRL_PID_RESET_REASON_REMOTE_MOTOR_OFF (6U)
#define CTRL_PID_RESET_REASON_DIRECT_PWM  (7U)
#define CTRL_PID_RESET_REASON_USE_PID_OFF (8U)
#define CTRL_PID_RESET_REASON_ZERO_TARGET (9U)
#define CTRL_PID_RESET_REASON_DRIVE_STALL (10U)
#define CTRL_PID_RESET_REASON_STEER_FEEDBACK_STALE (11U)

#define CTRL_PID_LOW_REASON_NONE          (0U)
#define CTRL_PID_LOW_REASON_RIGHT         (1U)
#define CTRL_PID_LOW_REASON_LEFT          (2U)
#define CTRL_PID_LOW_REASON_BOTH          (3U)

#define CTRL_STEER_RAW_REJECT_NONE        (0U)
#define CTRL_STEER_RAW_REJECT_INVALID     (1U)
#define CTRL_STEER_RAW_REJECT_RANGE       (2U)
#define CTRL_STEER_RAW_REJECT_JUMP        (3U)

#define CTRL_CPU0_STAGE_BOOT             (0U)
#define CTRL_CPU0_STAGE_CAMERA_LINK      (1U)
#define CTRL_CPU0_STAGE_KEYS             (2U)
#define CTRL_CPU0_STAGE_GPS              (3U)
#define CTRL_CPU0_STAGE_VOICE_BOARD      (4U)
#define CTRL_CPU0_STAGE_VOICE_COMMAND    (5U)
#define CTRL_CPU0_STAGE_LORA             (6U)
#define CTRL_CPU0_STAGE_PEDAL            (7U)
#define CTRL_CPU0_STAGE_REMOTE           (8U)
#define CTRL_CPU0_STAGE_INS              (9U)
#define CTRL_CPU0_STAGE_CONTROL          (10U)
#define CTRL_CPU0_STAGE_LOGGER           (11U)
#define CTRL_CPU0_STAGE_UI               (12U)
#define CTRL_CPU0_STAGE_BEEP             (13U)
#define CTRL_CPU0_STAGE_DELAY            (14U)
#define CTRL_CPU0_STAGE_TASK3_LOAD       (20U)
#define CTRL_CPU0_STAGE_TASK3_STATE      (21U)
#define CTRL_CPU0_STAGE_TASK3_TERMINAL   (22U)
#define CTRL_CPU0_STAGE_TASK3_STEER      (23U)
#define CTRL_CPU0_STAGE_TASK3_SPEED      (24U)
#define CTRL_CPU0_STAGE_TASK3_COMMAND    (25U)
#define CTRL_CPU0_STAGE_FOLLOW_SAVE      (30U)

extern volatile float g_pid_fixed_target_speed_mps;
extern volatile float g_pid_pedal_target_speed_mps;
extern volatile float g_pid_used_target_speed_mps;
extern volatile uint8 g_pid_target_src;
extern volatile uint8 g_pid_target_use_pedal;

extern volatile float g_steer_fixed_target_angle_deg;
extern volatile float g_steer_target_angle_deg;
extern volatile float g_steer_feedback_angle_deg;
extern volatile float g_steer_feedback_speed_counts_s;
extern volatile uint8 g_steer_feedback_valid;
extern volatile uint32 g_steer_feedback_update_ms;
extern volatile uint8 g_steer_sensor_fault;
extern volatile uint8 g_steer_stall_fault;
extern volatile uint32 g_steer_stall_fault_time_ms;
extern volatile int16 g_steer_raw_read;
extern volatile int16 g_steer_raw_last_valid;
extern volatile int16 g_steer_raw_jump_counts;
extern volatile uint8 g_steer_raw_last_reject_reason;
extern volatile uint8 g_steer_raw_last_reject_streak;
extern volatile uint32 g_steer_raw_reject_count;
extern volatile uint32 g_steer_raw_last_reject_ms;
extern volatile int16 g_steer_raw_last_reject_raw;
extern volatile int16 g_steer_raw_last_reject_ref_raw;
extern volatile int16 g_steer_raw_last_reject_jump_counts;
extern volatile uint32 g_ctrl_main_loop_heartbeat_ms;
extern volatile uint32 g_ctrl_cpu0_heartbeat_ms;
extern volatile uint32 g_ctrl_cpu0_age_ms;
extern volatile uint8 g_ctrl_cpu0_stage;
extern volatile uint8 g_ctrl_cpu0_stall_stage;
extern volatile uint32 g_ctrl_cpu0_stall_count;
extern volatile uint32 g_ctrl_cpu0_stall_max_age_ms;
extern volatile uint32 g_ctrl_safety_pause_until_ms;
extern volatile uint8 g_ctrl_stop_reason;
extern volatile uint8 g_ctrl_safety_pause_active;
extern volatile uint32 g_ctrl_main_loop_age_ms;
extern volatile uint32 g_ctrl_cmd_publish_ms;
extern volatile uint32 g_ctrl_cmd_age_ms;
extern volatile uint8 g_ctrl_pid_reset_reason;
extern volatile uint32 g_ctrl_pid_reset_time_ms;
extern volatile uint8 g_ctrl_pid_low_reason;
extern volatile uint32 g_ctrl_pid_low_time_ms;
extern volatile float g_ctrl_pid_low_target_r;
extern volatile float g_ctrl_pid_low_feedback_r;
extern volatile float g_ctrl_pid_low_output_r;
extern volatile float g_ctrl_pid_low_target_l;
extern volatile float g_ctrl_pid_low_feedback_l;
extern volatile float g_ctrl_pid_low_output_l;
extern volatile uint32 g_imu_pit_count;
extern volatile uint32 g_imu_update_count;
extern volatile uint32 g_imu_update_ms;
extern volatile uint32 g_imu_read_fail_count;

extern volatile ctrl_cmd_t g_ctrl_cmd;
extern volatile uint32 g_ctrl_cmd_seq;

void ctrl_cmd_publish(const ctrl_cmd_t *cmd);
uint8 ctrl_cmd_get_snapshot(ctrl_cmd_t *cmd);
void ctrl_main_loop_heartbeat(void);
void ctrl_cpu0_progress(uint8 stage);
void ctrl_safety_pause_ms(uint32 pause_ms);
uint8 ctrl_safety_pause_is_active(void);
#endif
