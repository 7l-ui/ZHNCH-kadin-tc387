/**
 ****************************************************************************************************
 * @file        es8388_test.h
 * @author      测试程序
 * @version     V1.0
 * @date        2026-04-06
 * @brief       ES8388功能测试程序头文件
 ****************************************************************************************************
 */

#ifndef __ES8388_TEST_H
#define __ES8388_TEST_H

#include "zf_common_typedef.h"

// 函数声明
uint8_t es8388_test_init(void);
void es8388_test_play_tone(uint32_t frequency, uint32_t duration_ms);
void es8388_test_i2c_communication(void);
void es8388_test_all(void);

#endif