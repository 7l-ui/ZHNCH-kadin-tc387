#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"
#include "stdbool.h"
#include "xl9555_app.h"
#include "UI.h"

// Key logical IDs for xl9555_app_get_key()
#define KEY1_XL9555             1
#define KEY2_XL9555             2
#define KEY3_XL9555             3
#define KEY4_XL9555             4
#define KEY5_XL9555             5

// Toggle logical IDs for xl9555_app_get_switch()
#define TOGGLE1_XL9555          1
#define TOGGLE2_XL9555          2

// Buzzer output pin
#define BUZZER_PIN              (P33_10)
#define PROGRAM_NUM             (15)

typedef struct {
    uint8 toggle1;
    uint8 toggle2;
} Toggle_status;

typedef struct {
    uint8 key1;
    uint8 key2;
    uint8 key3;
    uint8 key4;
    uint8 key5;
} Key_status;

typedef struct {
    uint8 key1;
    uint8 key2;
    uint8 key3;
    uint8 key4;
    uint8 key5;
} Last_status;

typedef struct {
    uint8 key1;
    uint8 key2;
    uint8 key3;
    uint8 key4;
    uint8 key5;
} Key_flag;

extern uint8 Key_Num;
extern volatile uint8 g_key_last_event_num;
extern volatile uint32 g_key_last_event_time_ms;
extern volatile uint32 g_key_event_count;
extern uint8 test_flag;
extern uint16 BUZZER_NUM;
extern uint8 program_select;

// Read toggle inputs.
void Toggle_Active(void);
// 初始化按键子系统
// 1) 初始�?XL9555
// 2) 初始化蜂鸣器输出为低电平
void Key_Init(void);

// 轮询按键并更�?Key_Num
// Poll keys and update Key_Num.
void Key_scan(void);
// 根据 Key_Num 执行对应按键动作
// 函数结束会将 Key_Num 清零
void Key_Active(void);

void Key_KM4_Deal(void);


// �?BUZZER_NUM 倒计时驱动蜂鸣器
void Beep_Deal(void);

// 调试打印接口：扫描并打印一�?Key_Num
void Key_Test_Print(void);

#endif
