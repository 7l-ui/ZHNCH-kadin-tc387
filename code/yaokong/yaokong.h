#ifndef _YAOKONG_H_
#define _YAOKONG_H_

#include "zf_common_headfile.h"

#define YAOKONG_UART_INDEX          UART_4
#define YAOKONG_UART_TX_PIN         UART4_TX_P00_9
#define YAOKONG_UART_RX_PIN         UART4_RX_P00_12
#define YAOKONG_UART_BAUDRATE       (100000)

#define YAOKONG_REMOTE_ENABLE       (1U)    //遥控使能

typedef enum
{
    TARGET_SRC_TASK = 0,
    TARGET_SRC_PEDAL = 1,
    TARGET_SRC_REMOTE = 2,
    TARGET_SRC_PWM_TEST = 3,
    TARGET_SRC_STEER_SCAN = 4
} target_src_t;

#define YAOKONG_CHANNEL_NUM         ( 6 )
#define YAOKONG_FRAME_LEN           ( 25 )
#define YAOKONG_FRAME_HEADER        ( 0x0F )
#define YAOKONG_FRAME_FOOTER        ( 0x00 )
#define YAOKONG_ABNORMAL_MASK       ( 0x04 )

#define YAOKONG_CHANNEL_THROTTLE    (1)
#define YAOKONG_CHANNEL_STEER       (0)
#define YAOKONG_CHANNEL_CH3_UNLOCK  (2)
#define YAOKONG_CHANNEL_CH5_BUTTON  (4)
#define YAOKONG_CHANNEL_CH6_BUTTON  (5)
#define YAOKONG_THROTTLE_MIN        (0)
#define YAOKONG_THROTTLE_MID        (1040)
#define YAOKONG_THROTTLE_NEG_RAW    (459)
#define YAOKONG_THROTTLE_POS_RAW    (1840)
#define YAOKONG_STEER_MID           (832)
#define YAOKONG_STEER_NEG_RAW       (158)
#define YAOKONG_STEER_POS_RAW       (1505)
#define YAOKONG_THROTTLE_MAX        (2047)
#define YAOKONG_TARGET_SPEED_MAX    (3.00f)
#define YAOKONG_TARGET_STEER_MAX_DEG (25.0f)
#define YAOKONG_STEER_DIR           (-1.0f)
#define YAOKONG_GUARD_ENABLE_DEFAULT (1U)   //baohu
#define YAOKONG_REMOTE_ARM_NEUTRAL_US (300000UL)
#define YAOKONG_REMOTE_ARM_SPEED_EPS   (0.05f)
#define YAOKONG_REMOTE_ARM_STEER_EPS_DEG (1.0f)

typedef struct {
    uint16 channel[YAOKONG_CHANNEL_NUM];
    uint8  state;
    uint8  finsh_flag;
    uint8  connected;
    uint8  guard_enable;
    uint8  motor_run_latch;
    uint8  ch3_pressed;
    uint8  ch5_pressed;
    uint8  ch6_pressed;
    float  target_speed_mps;
    float  target_steer_angle_deg;
} yaokong_status_t;

extern volatile yaokong_status_t g_yaokong;

void yaokong_init(void);
void yaokong_handler(void);
void yaokong_update_speed(void);
float yaokong_get_target_speed(void);
float yaokong_get_target_steer_angle(void);
uint8 yaokong_is_connected(void);
uint8 yaokong_is_motor_run_enabled(void);
void yaokong_remote_entry_reset(void);
uint8 yaokong_remote_is_armed(void);
uint8 yaokong_ch6_take_press_event(void);
uint32 yaokong_get_last_frame_age_ms(void);
uint32 yaokong_get_valid_frame_count(void);
uint32 yaokong_get_bad_frame_count(void);
void yaokong_guard_set_enable(uint8 enable);
uint8 yaokong_guard_get_enable(void);

#endif
