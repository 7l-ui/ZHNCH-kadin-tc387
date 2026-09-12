/*********************************************************************************************************************
 * XL9555 16位I/O扩展芯片驱动 - 头文件
 * 适配 TC387 平台控制 XL9535/XL9555 I2C接口16位GPIO扩展芯片
 *
 * 功能：提供16位GPIO扩展芯片的输入/输出控制、极性配置和模式设置
 * 特性：
 *     1. 16个独立GPIO，分为PORT0（0-7）和PORT1（8-15）
 *     2. 支持输入/输出模式配置
 *     3. 支持输入极性反转（正逻辑/负逻辑）
 *     4. I2C接口，支持标准模式（100kHz）和快速模式（400kHz）
 *     5. 软件I2C接口，基于zf_driver_soft_iic库
 *
 * 寄存器说明：
 *     0x00: INPUT_PORT0    - PORT0输入状态寄存器
 *     0x01: INPUT_PORT1    - PORT1输入状态寄存器
 *     0x02: OUTPUT_PORT0   - PORT0输出状态寄存器
 *     0x03: OUTPUT_PORT1   - PORT1输出状态寄存器
 *     0x04: POLARITY_PORT0 - PORT0极性反转寄存器
 *     0x05: POLARITY_PORT1 - PORT1极性反转寄存器
 *     0x06: MODE_PORT0     - PORT0方向配置寄存器（0=输出，1=输入）
 *     0x07: MODE_PORT1     - PORT1方向配置寄存器（0=输出，1=输入）
 *
 * 使用示例：
 *     xl9555_init_desc(&xl9555_dev, 0x20, 10, P20_0, P20_1);  // 初始化设备
 *     xl9555_set_gpio_mode(&xl9555_dev, 0, XL9555_GPIO_OUTPUT);  // GPIO0设为输出
 *     xl9555_set_gpio_level(&xl9555_dev, 0, 1);  // GPIO0输出高电平
 *     uint8_t level;
 *     xl9555_get_gpio_level(&xl9555_dev, 8, &level);  // 读取GPIO8的输入电平
 ********************************************************************************************************************/

#ifndef _XL9555_H_
#define _XL9555_H_

#include "zf_common_headfile.h"

// XL9555 I2C地址定义
#define XL9555_I2C_ADDRESS_BASE     (0x20)  /*!< XL9555默认I2C基地址 */
#define XL9555_I2C_ADDRESS_MAX      (0x27)  /*!< XL9555最大I2C地址 */

// 寄存器地址定义
#define XL9555_REG_INPUT_PORT0      (0x00)  /*!< PORT0输入状态寄存器 */
#define XL9555_REG_INPUT_PORT1      (0x01)  /*!< PORT1输入状态寄存器 */
#define XL9555_REG_OUTPUT_PORT0     (0x02)  /*!< PORT0输出状态寄存器 */
#define XL9555_REG_OUTPUT_PORT1     (0x03)  /*!< PORT1输出状态寄存器 */
#define XL9555_REG_POLARITY_PORT0   (0x04)  /*!< PORT0极性反转寄存器 */
#define XL9555_REG_POLARITY_PORT1   (0x05)  /*!< PORT1极性反转寄存器 */
#define XL9555_REG_MODE_PORT0       (0x06)  /*!< PORT0方向配置寄存器 */
#define XL9555_REG_MODE_PORT1       (0x07)  /*!< PORT1方向配置寄存器 */

// 引脚定义（注意：根据用户反馈，P13_0作为SDA线电压只有1.6V，可能存在上拉电阻不足问题）
// 用户希望继续使用P13_1和P13_0引脚
#define XL9555_SCL_PIN              P13_1   /*!< I2C时钟线引脚定义 */
#define XL9555_SDA_PIN              P13_0   /*!< I2C数据线引脚定义 */
#define XL9555_INT_PIN              P14_3   /*!< 中断线引脚定义 */

// I2C通信延迟定义（单位：微秒级循环计数，根据实际时钟频率调整）
#define XL9555_I2C_DELAY            (1000)  /*!< 无外部上拉时使用更低速率，提高软I2C稳定性 */

/**
 * @brief GPIO模式枚举
 * @note 0: 输出模式，1: 输入模式
 */
typedef enum {
    XL9555_GPIO_OUTPUT = 0,  /*!< GPIO配置为输出模式 */
    XL9555_GPIO_INPUT,       /*!< GPIO配置为输入模式 */
} xl9555_gpio_mode_t;

/**
 * @brief GPIO极性枚举
 * @note 0: 不反转（正逻辑），1: 反转（负逻辑）
 */
typedef enum {
    XL9555_POLARITY_NOT_INVERTED = 0,  /*!< 输入极性不反转 */
    XL9555_POLARITY_INVERTED,          /*!< 输入极性反转 */
} xl9555_polarity_t;

/**
 * @brief XL9555设备描述符结构体
 * @note 包含I2C通信对象和设备信息
 */
typedef struct {
    soft_iic_info_struct *soft_iic_obj;  /*!< 软件I2C对象指针 */
    uint8 addr;                          /*!< I2C设备地址（7位） */
} xl9555_dev_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化XL9555设备描述符
 * @param[in] dev 设备描述符指针
 * @param[in] addr I2C设备地址（7位，范围0x20-0x27）
 * @param[in] delay I2C通信延迟时间（单位：微秒级循环计数）
 * @param[in] scl_pin SCL引脚定义
 * @param[in] sda_pin SDA引脚定义
 * @note 内部会创建soft_iic_info_struct对象并初始化I2C对象
 */
void xl9555_init_desc(xl9555_dev_t *dev, uint8 addr, uint32 delay,
                      gpio_pin_enum scl_pin, gpio_pin_enum sda_pin);

/**
 * @brief 释放XL9555设备描述符
 * @param[in] dev 设备描述符指针
 * @note 用于此函数释放分配的soft_iic_info_struct对象（如需要）
 */
void xl9555_free_desc(xl9555_dev_t *dev);

/**
 * @brief 读取所有GPIO的电平状态（16位）
 * @param[in] dev 设备描述符指针
 * @param[out] levels 电平状态值指针（16位，bit0对应GPIO0，bit15对应GPIO15）
 * @note 返回值：0=成功，非0=失败（实际实现中可根据需要添加返回值）
 */
void xl9555_get_full_gpio_level(xl9555_dev_t *dev, uint16 *levels);

/**
 * @brief 读取单个GPIO的电平状态
 * @param[in] dev 设备描述符指针
 * @param[in] gpio GPIO编号（0-15）
 * @param[out] level 电平状态指针（0=低电平，1=高电平）
 * @note 若GPIO配置为输出模式，读取输出寄存器的值
 */
void xl9555_get_gpio_level(xl9555_dev_t *dev, uint8 gpio, uint8 *level);

/**
 * @brief 设置所有GPIO的输出电平（16位）
 * @param[in] dev 设备描述符指针
 * @param[in] levels 电平状态值（16位，bit0对应GPIO0，bit15对应GPIO15）
 * @note 只对配置为输出模式的GPIO才会有效
 */
void xl9555_set_full_gpio_level(xl9555_dev_t *dev, uint16 levels);

/**
 * @brief 设置单个GPIO的输出电平
 * @param[in] dev 设备描述符指针
 * @param[in] gpio GPIO编号（0-15）
 * @param[in] level 电平状态（0=低电平，1=高电平）
 * @note 只对配置为输出模式的GPIO才会有效
 */
void xl9555_set_gpio_level(xl9555_dev_t *dev, uint8 gpio, uint8 level);

/**
 * @brief 读取所有GPIO的极性设置（16位）
 * @param[in] dev 设备描述符指针
 * @param[out] polarity 极性配置值指针（16位，0=不反转，1=反转）
 * @note 极性设置仅对输入模式的GPIO有效
 */
void xl9555_get_full_gpio_polarity(xl9555_dev_t *dev, uint16 *polarity);

/**
 * @brief 读取单个GPIO的极性配置
 * @param[in] dev 设备描述符指针
 * @param[in] gpio GPIO编号（0-15）
 * @param[out] polarity 极性配置指针
 * @note 极性设置仅对输入模式的GPIO有效
 */
void xl9555_get_gpio_polarity(xl9555_dev_t *dev, uint8 gpio, xl9555_polarity_t *polarity);

/**
 * @brief 设置所有GPIO的极性设置（16位）
 * @param[in] dev 设备描述符指针
 * @param[in] polarity 极性配置值（16位，0=不反转，1=反转）
 * @note 极性设置仅对输入模式的GPIO有效
 */
void xl9555_set_full_gpio_polarity(xl9555_dev_t *dev, uint16 polarity);

/**
 * @brief 设置单个GPIO的极性配置
 * @param[in] dev 设备描述符指针
 * @param[in] gpio GPIO编号（0-15）
 * @param[in] polarity 极性配置
 * @note 极性设置仅对输入模式的GPIO有效
 */
void xl9555_set_gpio_polarity(xl9555_dev_t *dev, uint8 gpio, xl9555_polarity_t polarity);

/**
 * @brief 读取所有GPIO的方向模式（16位）
 * @param[in] dev 设备描述符指针
 * @param[out] mode 方向模式值指针（16位，0=输出，1=输入）
 * @note 复位后所有GPIO默认为输入模式
 */
void xl9555_get_full_gpio_mode(xl9555_dev_t *dev, uint16 *mode);

/**
 * @brief 读取单个GPIO的方向模式
 * @param[in] dev 设备描述符指针
 * @param[in] gpio GPIO编号（0-15）
 * @param[out] mode 方向模式指针
 * @note 复位后所有GPIO默认为输入模式
 */
void xl9555_get_gpio_mode(xl9555_dev_t *dev, uint8 gpio, xl9555_gpio_mode_t *mode);

/**
 * @brief 设置所有GPIO的方向模式（16位）
 * @param[in] dev 设备描述符指针
 * @param[in] mode 方向模式值（16位，0=输出，1=输入）
 * @note 复位后所有GPIO默认为输入模式，需要相应配置输出引脚
 */
void xl9555_set_full_gpio_mode(xl9555_dev_t *dev, uint16 mode);

/**
 * @brief 设置单个GPIO的方向模式
 * @param[in] dev 设备描述符指针
 * @param[in] gpio GPIO编号（0-15）
 * @param[in] mode 方向模式
 * @note 复位后所有GPIO默认为输入模式，需要相应配置输出引脚
 */
void xl9555_set_gpio_mode(xl9555_dev_t *dev, uint8 gpio, xl9555_gpio_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // _XL9555_H_
