/*********************************************************************************************************************
 * MT6701 磁编码器驱动程序 - 源文件
 * 适用 TC387 平台驱动 MT6701 I2C接口磁编码器传感器
 *
 * 注意：这是简化版本，仅适用通用场景。
 * 实际使用时需根据硬件配置I2C通信
 ********************************************************************************************************************/

#include "MT6701.h"
#include <stdlib.h>

// 内部全局变量
static int g_initialized = 0;

/**
 * @brief 初始化MT6701设备
 */
void MT6701_Init(MT6701_Handle *handle, uint8 address, int update_interval,
                 int rpm_threshold, int rpm_filter_size,
                 gpio_pin_enum scl_pin, gpio_pin_enum sda_pin, uint32 delay)
{
    // 简单初始化，分配滤波器内存
    if (handle == NULL) {
        return;
    }

    // 初始化结构体字段
    handle->address = address;
    handle->update_interval = update_interval;
    handle->rpm_threshold = rpm_threshold;
    if (rpm_filter_size < 0) {
        handle->rpm_filter_size = 0;
    } else if (rpm_filter_size > 0xFFFF) {
        handle->rpm_filter_size = 0xFFFF;
    } else {
        handle->rpm_filter_size = (uint16)rpm_filter_size;
    }
    handle->last_update_time = 0;
    handle->count = 0;
    handle->accumulator = 0;
    handle->rpm = 0.0f;
    handle->rpm_filter_index = 0;

    // 分配滤波器内存
    if (handle->rpm_filter_size > 0) {
        handle->rpm_filter = (float*)malloc(sizeof(float) * handle->rpm_filter_size);
        if (handle->rpm_filter != NULL) {
            for (uint16 i = 0; i < handle->rpm_filter_size; i++) {
                handle->rpm_filter[i] = 0.0f;
            }
        }
    } else {
        handle->rpm_filter = NULL;
    }

    // 初始化软件I2C对象 - 为MT6701创建独立的I2C实例
    handle->soft_iic_obj = (soft_iic_info_struct*)malloc(sizeof(soft_iic_info_struct));
    if (handle->soft_iic_obj == NULL) {
        printf("MT6701_Init: Failed to allocate memory for soft_iic_obj\r\n");
        return;
    }

    // 使用soft_iic_init来正确初始化I2C，它会自动配置GPIO（包括内部上拉）
    soft_iic_init(handle->soft_iic_obj, address, delay, scl_pin, sda_pin);

    g_initialized = 1;

    printf("MT6701_Init: Device initialized with I2C address 0x%02X\r\n", address);
    printf("MT6701_Init: Using dedicated I2C bus - SCL=P%d_%d, SDA=P%d_%d (Open-Drain mode)\r\n",
           scl_pin / 32, scl_pin % 32, sda_pin / 32, sda_pin % 32);

    // 测试软件I2C通信
    printf("Testing MT6701 I2C communication...\r\n");
    unsigned char test_buff[2];
    soft_iic_read_8bit_registers(handle->soft_iic_obj, 0x03, test_buff, 2);
    printf("MT6701 I2C Test Read: buff[0]=0x%02X, buff[1]=0x%02X\r\n", test_buff[0], test_buff[1]);

    if (test_buff[0] == 0xFF && test_buff[1] == 0xFF) {
        printf("MT6701 WARNING: I2C read returned 0xFF 0xFF - Check I2C address and connections\r\n");
        printf("Try using address 0x0C (8-bit write address) instead of 0x06 (7-bit)\r\n");
    }
}

/**
 * @brief 初始化MT6701设备（使用默认引脚和延迟）
 */
void MT6701_Init_Default(MT6701_Handle *handle, uint8 address, int update_interval,
                         int rpm_threshold, int rpm_filter_size)
{
    // 调用完整初始化函数，使用默认引脚和延迟
    MT6701_Init(handle, address, update_interval, rpm_threshold, rpm_filter_size,
                MT6701_DEFAULT_SCL_PIN, MT6701_DEFAULT_SDA_PIN, MT6701_DEFAULT_I2C_DELAY);
}

/**
 * @brief 释放MT6701设备资源
 */
void MT6701_Deinit(MT6701_Handle *handle)
{
    if (handle == NULL) {
        return;
    }

    // 释放滤波器内存
    if (handle->rpm_filter != NULL) {
        free(handle->rpm_filter);
        handle->rpm_filter = NULL;
    }

    // 释放I2C对象内存
    if (handle->soft_iic_obj != NULL) {
        free(handle->soft_iic_obj);
        handle->soft_iic_obj = NULL;
    }

    g_initialized = 0;
    printf("MT6701_Deinit: Resources released\r\n");
}

/**
 * @brief 更新MT6701角度和转速数据
 */
void MT6701_Update(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return;
    }

    // 此函数仅更新内部状态，不进行I2C读取
    // 实际读取应在MT6701_ReadCount中完成
    // 这里只更新内部时间戳和滤波器
    handle->last_update_time = handle->last_update_time + handle->update_interval;

    // 如果需要，更新转速滤波器
    if (handle->rpm_filter != NULL && handle->rpm_filter_size > 0) {
        handle->rpm_filter[handle->rpm_filter_index] = handle->rpm;
        handle->rpm_filter_index = (handle->rpm_filter_index + 1) % handle->rpm_filter_size;
    }
}

/**
 * @brief 获取当前角度（弧度）
 */
float MT6701_GetAngleRadians(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0.0f;
    }
    return (float)handle->count * MT6701_COUNTS_TO_RADIANS;
}

/**
 * @brief 获取当前角度（度）
 */
float MT6701_GetAngleDegrees(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0.0f;
    }
    return (float)handle->count * MT6701_COUNTS_TO_DEGREES;
}

/**
 * @brief 获取累计完整圈数
 */
int MT6701_GetFullTurns(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0;
    }
    return handle->accumulator / 16384;
}

/**
 * @brief 获取累计转数（浮点数）
 */
float MT6701_GetTurns(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0.0f;
    }
    return (float)handle->accumulator / 16384.0f;
}

/**
 * @brief 获取累计计数值
 */
int MT6701_GetAccumulator(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0;
    }
    return handle->accumulator;
}

/**
 * @brief 获取当前转速（RPM）
 */
float MT6701_GetRPM(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0.0f;
    }
    return handle->rpm;
}

/**
 * @brief 获取当前原始计数值
 */
int MT6701_GetCount(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return 0;
    }
    return handle->count;
}

/**
 * @brief 直接读取MT6701角度计数值
 * @note 在模拟模式下，这会进行实际I2C读取，可能被中断打断
 */
int MT6701_ReadCount(MT6701_Handle *handle)
{
    if (handle == NULL || !g_initialized) {
        return -1; // 错误
    }

    if (handle->soft_iic_obj == NULL) {
        printf("MT6701 Error: I2C object not initialized\r\n");
        return -1;
    }

    unsigned char buff[2];
    int angle_raw, angle_count;
    int retry_count = 0;
    const int max_retries = 3;
    int read_success = 0;

    // 保存当前中断状态并禁用中断（避免I2C时序）
    // 注意：这里需要根据TC387的具体中断API实现
    // 例如：disable_interrupts和restore_interrupts函数
    // 暂时使用占位符

    // 带重试机制的读取
    for (retry_count = 0; retry_count < max_retries; retry_count++) {
        // 从寄存器0x03开始读取2个字节（角度数据）
        soft_iic_read_8bit_registers(handle->soft_iic_obj, 0x03, buff, 2);

        // 检查读取结果是否有效
        // 0xFF 0xFF 通常表示I2C读取失败
        if (!(buff[0] == 0xFF && buff[1] == 0xFF)) {
            read_success = 1;
            break; // 读取成功
        }

        // 如果读取失败，添加延时后重试
        if (retry_count < max_retries - 1) {
            // 使用简单的循环延时
            for (volatile int i = 0; i < 500; i++);
        }
    }

    // 调试信息：打印原始寄存器值（降低打印频率）
    static int debug_counter = 0;
    if (debug_counter++ % 100 == 0) { // 每100次读取打印一次
        printf("MT6701 Raw[%d]: buff[0]=0x%02X, buff[1]=0x%02X (retry=%d)\r\n",
               debug_counter, buff[0], buff[1], retry_count);
    }

    // 检查是否所有重试都失败
    if (!read_success) {
        // I2C通信失败
        if (debug_counter % 100 == 1) {
            printf("MT6701 Error: I2C read failed after %d retries\r\n", max_retries);
        }
        return -2; // I2C通信错误
    }

    // I2C读取成功，解析数据
    // 合并14位角度数据
    angle_raw = ((int)buff[0] << 8) | (int)buff[1];

    // 右移2位得到14位角度值（去掉低2位状态位）
    angle_raw = angle_raw >> 2;

    // 确保在0-16383范围内
    angle_count = angle_raw & 0x3FFF;

    // 调试信息：打印解析结果
    if (debug_counter % 100 == 1) {
        printf("MT6701 Calc: raw=0x%04X (%d), count=%d, angle=%.2f°\r\n",
               angle_raw, angle_raw, angle_count, (float)angle_count * (360.0f / 16384.0f));
    }

    // 更新句柄中的计数值
    handle->count = angle_count;

    return angle_count;
}




