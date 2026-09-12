/*********************************************************************************************************************
 * MT6701 磁编码器驱动库 - 头文件
 * 基于 TC387 开发板的 MT6701 I2C接口磁编码器驱动
 * 
 * 功能：提供MT6701磁编码器的角度读取、转速计算、累计转数等功能
 * 特性：
 *     1. 14位分辨率（0-16383计数）
 *     2. 支持角度读取（弧度/度）
 *     3. 支持转速计算（RPM）
 *     4. 支持累计转数统计
 *     5. I2C接口，使用软件I2C驱动
 * 
 * 硬件连接：
 *     I2C数据线：MAG_SDA -> MAG_I2C3_SDA_P32_4 (P32_4)
 *     I2C时钟线：MAG_SCL -> MAG_I2C3_SCL_P33_8 (P33_8)
 * 
 * 寄存器说明：
 *     0x03: ANGLE_H - 角度高8位寄存器
 *     0x04: ANGLE_L - 角度低8位寄存器
 * 
 * 使用示例：
 *     MT6701_Handle mt6701;
 *     MT6701_Init(&mt6701, 0x36, 10, P32_4, P33_8);  // 初始化设备
 *     float angle_deg = MT6701_GetAngleDegrees(&mt6701);  // 读取角度（度）
 *     float rpm = MT6701_GetRPM(&mt6701);  // 读取转速
 *     int turns = MT6701_GetFullTurns(&mt6701);  // 读取完整转数
 ********************************************************************************************************************/

#ifndef _MT6701_H_
#define _MT6701_H_

#include "zf_common_headfile.h"
#include "zf_driver_soft_iic.h"

// MT6701 常量定义
#define MT6701_COUNTS_PER_REVOLUTION    16384.0f    /*!< 每转计数（14位分辨率） */
#define MT6701_COUNTS_TO_RADIANS        (2.0f * 3.141592653589793f / MT6701_COUNTS_PER_REVOLUTION)  /*!< 计数到弧度转换系数 */
#define MT6701_COUNTS_TO_DEGREES        (360.0f / MT6701_COUNTS_PER_REVOLUTION)  /*!< 计数到度转换系数 */
#define MT6701_SECONDS_PER_MINUTE       60.0f       /*!< 每分钟秒数 */

// 默认引脚定义（根据用户提供的硬件连接）
#define MT6701_DEFAULT_SCL_PIN          P33_8       /*!< 默认I2C时钟线引脚定义 */
#define MT6701_DEFAULT_SDA_PIN          P32_4       /*!< 默认I2C数据线引脚定义 */
#define MT6701_DEFAULT_ADDRESS          0x06        /*!< 默认I2C设备地址 (7-bit address) */
#define MT6701_DEFAULT_I2C_DELAY        100         /*!< 默认I2C通信延迟（共享总线时降低速率） */

// MT6701 寄存器地址（基于官方权威数据手册）
#define MT6701_REG_ANGLE_MSB            0x03        /*!< 角度高8位寄存器地址 (Data[13:6]) */
#define MT6701_REG_ANGLE_LSB            0x04        /*!< 角度低8位寄存器地址 (Data[5:0] + 2位状态位) */
#define MT6701_REG_EEPROM_KEY           0x09        /*!< EEPROM编程密钥寄存器地址 */
#define MT6701_REG_EEPROM_CMD           0x0A        /*!< EEPROM编程指令触发寄存器地址 */

// I2C地址定义（官方固定）
#define MT6701_I2C_ADDR_7BIT            0x06        /*!< 7位I2C设备地址（固定不可配置） */
#define MT6701_I2C_ADDR_WRITE           0x0C        /*!< 8位写地址（0x06 << 1 | 0） */
#define MT6701_I2C_ADDR_READ            0x0D        /*!< 8位读地址（0x06 << 1 | 1） */

/**
 * @brief MT6701设备句柄结构体
 * @note 包含MT6701设备的所有状态信息和I2C通信对象
 */
typedef struct {
    soft_iic_info_struct *soft_iic_obj;  /*!< 软件I2C对象指针 */
    uint8 address;                       /*!< I2C设备地址（7位） */
    
    int count;                           /*!< 当前角度计数值（0-16383） */
    int accumulator;                     /*!< 累计角度计数（可记录多圈） */
    float rpm;                           /*!< 当前转速（RPM） */
    
    // RPM滤波相关参数
    float *rpm_filter;                   /*!< RPM滤波数组指针 */
    uint16 rpm_filter_size;              /*!< RPM滤波数组大小 */
    uint16 rpm_filter_index;             /*!< RPM滤波当前索引 */
    int rpm_threshold;                   /*!< RPM滤波阈值，低于此值的RPM才进行滤波 */
    
    uint32 last_update_time;             /*!< 最后更新时间（毫秒） */
    int update_interval;                 /*!< 更新间隔（毫秒） */
} MT6701_Handle;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化MT6701设备
 * @param[in] handle MT6701设备句柄指针
 * @param[in] address I2C设备地址（7位）
 * @param[in] update_interval 更新间隔（毫秒）
 * @param[in] rpm_threshold RPM滤波阈值（RPM）
 * @param[in] rpm_filter_size RPM滤波数组大小
 * @param[in] scl_pin SCL引脚定义
 * @param[in] sda_pin SDA引脚定义
 * @param[in] delay I2C通信延迟时间
 * @note 此函数会初始化I2C总线并分配RPM滤波数组内存
 */
void MT6701_Init(MT6701_Handle *handle, uint8 address, int update_interval,
                 int rpm_threshold, int rpm_filter_size,
                 gpio_pin_enum scl_pin, gpio_pin_enum sda_pin, uint32 delay);

/**
 * @brief 初始化MT6701设备（使用默认引脚和延迟）
 * @param[in] handle MT6701设备句柄指针
 * @param[in] address I2C设备地址（7位）
 * @param[in] update_interval 更新间隔（毫秒）
 * @param[in] rpm_threshold RPM滤波阈值（RPM）
 * @param[in] rpm_filter_size RPM滤波数组大小
 * @note 使用默认引脚P33_8(SCL)和P32_4(SDA)，默认延迟10
 */
void MT6701_Init_Default(MT6701_Handle *handle, uint8 address, int update_interval,
                         int rpm_threshold, int rpm_filter_size);

/**
 * @brief 释放MT6701设备资源
 * @param[in] handle MT6701设备句柄指针
 * @note 释放RPM滤波数组内存
 */
void MT6701_Deinit(MT6701_Handle *handle);

/**
 * @brief 更新MT6701角度和转速数据
 * @param[in] handle MT6701设备句柄指针
 * @note 此函数会读取当前角度并计算转速，更新累计计数
 */
void MT6701_Update(MT6701_Handle *handle);

/**
 * @brief 获取当前角度（弧度）
 * @param[in] handle MT6701设备句柄指针
 * @return 角度值（弧度，范围[0, 2π)）
 */
float MT6701_GetAngleRadians(MT6701_Handle *handle);

/**
 * @brief 获取当前角度（度）
 * @param[in] handle MT6701设备句柄指针
 * @return 角度值（度，范围[0, 360)）
 */
float MT6701_GetAngleDegrees(MT6701_Handle *handle);

/**
 * @brief 获取累计完整转数
 * @param[in] handle MT6701设备句柄指针
 * @return 完整转数（整数）
 */
int MT6701_GetFullTurns(MT6701_Handle *handle);

/**
 * @brief 获取累计转数（浮点数）
 * @param[in] handle MT6701设备句柄指针
 * @return 转数（浮点数）
 */
float MT6701_GetTurns(MT6701_Handle *handle);

/**
 * @brief 获取累计计数值
 * @param[in] handle MT6701设备句柄指针
 * @return 累计计数值（整数）
 */
int MT6701_GetAccumulator(MT6701_Handle *handle);

/**
 * @brief 获取当前转速（RPM）
 * @param[in] handle MT6701设备句柄指针
 * @return 转速值（RPM）
 */
float MT6701_GetRPM(MT6701_Handle *handle);

/**
 * @brief 获取当前原始计数值
 * @param[in] handle MT6701设备句柄指针
 * @return 原始计数值（0-16383）
 */
int MT6701_GetCount(MT6701_Handle *handle);

/**
 * @brief 直接读取MT6701角度计数值
 * @param[in] handle MT6701设备句柄指针
 * @return 角度计数值（0-16383），错误返回-1
 * @note 此函数直接进行I2C读取，不更新内部状态
 */
int MT6701_ReadCount(MT6701_Handle *handle);

/**
 * @brief 诊断MT6701设备
 * @param[in] handle MT6701设备句柄指针
 * @note 此函数会测试I2C通信并打印诊断信息
 */
void MT6701_Diagnose(MT6701_Handle *handle);

#ifdef __cplusplus
}
#endif

#endif // _MT6701_H_
