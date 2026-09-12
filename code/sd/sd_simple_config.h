/*********************************************************************************************************************
* SD卡控制器引脚配置
* 文件位置：code/sd/sd_simple_config.h
* 说明：在此文件中配置SPI引脚和SD卡参数
* 注意：需要包含zf_driver_gpio.h与zf_driver_spi.h以使用引脚定义
********************************************************************************************************************/

#ifndef _SD_SIMPLE_CONFIG_H_
#define _SD_SIMPLE_CONFIG_H_

// 包含GPIO引脚定义头文件
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

// ==================== SPI引脚配置 ====================
// 请根据实际硬件连接修改以下引脚定义
// 当前默认使用 QSPI3：P02_7(SCK), P02_6(MOSI), P02_5(MISO), P02_4(CS)

// SPI 模块号
#define SD_SPI_INDEX  SPI_3

// CS (片选) 引脚 
#define SD_CS_PIN     P02_4

// SCK (时钟) 引脚  
#define SD_SCK_PIN    SPI3_SCLK_P02_7

// MOSI (主出从入) 引脚 
#define SD_MOSI_PIN   SPI3_MOSI_P02_6

// MISO (主入从出) 引脚 
#define SD_MISO_PIN   SPI3_MISO_P02_5

// ==================== SD卡参数配置 ====================

// SPI时钟频率（Hz）
#define SD_SPI_CLOCK_FREQ  400000   // 400kHz初始化频率
#define SD_SPI_CLOCK_HIGH  25000000 // 25MHz高速模式

// SD卡命令重试次数
#define SD_CMD_RETRY_COUNT  100

// SD卡初始化重试次数
#define SD_INIT_RETRY_COUNT  10

// 超时时间（毫秒）
#define SD_TIMEOUT_MS        5000

// ==================== SD卡命令定义 ====================

#define SD_CMD0     0   // 复位SD卡
#define SD_CMD8     8   // 发送接口条件
#define SD_CMD17    17  // 读单块
#define SD_CMD24    24  // 写单块
#define SD_CMD55    55  // 应用特定命令
#define SD_CMD58    58  // 读OCR寄存器
#define SD_ACMD41   41  // 发送主机容量支持信息

// 响应类型
#define SD_R1_IDLE_STATE        0x01
#define SD_R1_ERASE_RESET       0x02
#define SD_R1_ILLEGAL_COMMAND   0x04
#define SD_R1_CRC_ERROR         0x08
#define SD_R1_ERASE_SEQ_ERROR   0x10
#define SD_R1_ADDRESS_ERROR     0x20
#define SD_R1_PARAMETER_ERROR   0x40

// 数据令牌
#define SD_DATA_TOKEN_START_BLOCK  0xFE
#define SD_DATA_TOKEN_STOP_TRAN    0xFD

#endif



