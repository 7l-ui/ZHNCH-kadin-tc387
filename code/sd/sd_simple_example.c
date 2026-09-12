/*********************************************************************************************************************
* SD卡简单使用示例
* 文件位置：code/sd/sd_simple_example.c
* 说明：演示如何使用sd_simple库进行SD卡操作
********************************************************************************************************************/

#include "sd_simple.h"
#include "zf_driver_uart.h"
#include <stdio.h>

// 测试数据缓冲区（1个扇区 = 512字节）
static uint8_t test_buffer[512];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     填充测试数据缓冲区
// 使用示例     fill_test_buffer();
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static void fill_test_buffer(void)
{
    for (int i = 0; i < 512; i++) {
        test_buffer[i] = i & 0xFF;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     验证读取的数据缓冲区
// 参数说明     *buffer             要验证的数据缓冲区指针
// 返回参数     uint8_t 1-验证通过，0-验证失败
// 使用示例     if(verify_buffer(buffer)) { ... }
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static uint8_t verify_buffer(uint8_t *buffer)
{
    for (int i = 0; i < 512; i++) {
        if (buffer[i] != (i & 0xFF)) {
            printf("Data error at position %d: expected 0x%02X, got 0x%02X\n", 
                   i, i & 0xFF, buffer[i]);
            return 0;
        }
    }
    return 1;
}

// 主测试函数
void sd_simple_test(void)
{
    uint8_t result;
    
    printf("\n========================================\n");
    printf("SD Card Simple Library Test\n");
    printf("========================================\n\n");
    
    // 1. 初始化SD卡
    printf("1. Initializing SD card... ");
    result = sd_simple_init();
    if (result == 0) {
        printf("OK\n");
    } else {
        printf("FAILED (Error code: %d)\n", result);
        return;
    }
    
    // 2. 打印SD卡信息
    printf("2. SD card information:\n");
    sd_simple_print_info();
    
    // 3. 检查SD卡是否就绪
    printf("3. Checking SD card readiness... ");
    if (sd_simple_is_ready()) {
        printf("Ready\n");
    } else {
        printf("Not ready\n");
        return;
    }
    
    // 4. 获取容量
    printf("4. SD card capacity: %u MB\n", sd_simple_get_capacity_mb());
    
    // 5. 写入测试数据（扇区100）
    printf("5. Writing test data to sector 100... ");
    fill_test_buffer();
    result = sd_simple_write(test_buffer, 100, 1);
    if (result == 0) {
        printf("OK\n");
    } else {
        printf("FAILED (Error code: %d)\n", result);
        return;
    }
    
    // 6. 读取测试数据（扇区100）
    printf("6. Reading data from sector 100... ");
    uint8_t read_buffer[512];
    result = sd_simple_read(read_buffer, 100, 1);
    if (result == 0) {
        printf("OK\n");
    } else {
        printf("FAILED (Error code: %d)\n", result);
        return;
    }
    
    // 7. 验证数据
    printf("7. Verifying data... ");
    if (verify_buffer(read_buffer)) {
        printf("OK\n");
    } else {
        printf("FAILED (Data mismatch)\n");
        return;
    }
    
    // 8. 批量读写测试（扇区200-204）
    printf("8. Testing batch read/write (sectors 200-204)...\n");
    
    // 准备批量测试数据
    uint8_t batch_write[5 * 512];
    uint8_t batch_read[5 * 512];
    
    for (int i = 0; i < 5 * 512; i++) {
        batch_write[i] = 0xAA + (i % 10);
    }
    
    // 批量写入
    printf("   Writing 5 sectors... ");
    result = sd_simple_write(batch_write, 200, 5);
    if (result == 0) {
        printf("OK\n");
    } else {
        printf("FAILED (Error code: %d)\n", result);
        return;
    }
    
    // 批量读取
    printf("   Reading 5 sectors... ");
    result = sd_simple_read(batch_read, 200, 5);
    if (result == 0) {
        printf("OK\n");
    } else {
        printf("FAILED (Error code: %d)\n", result);
        return;
    }
    
    // 批量验证
    printf("   Verifying batch data... ");
    uint8_t batch_ok = 1;
    for (int i = 0; i < 5 * 512; i++) {
        if (batch_read[i] != batch_write[i]) {
            batch_ok = 0;
            printf("FAILED at position %d\n", i);
            break;
        }
    }
    if (batch_ok) {
        printf("OK\n");
    }
    
    printf("\n========================================\n");
    printf("SD Card Test COMPLETED SUCCESSFULLY\n");
    printf("========================================\n");
}

// 快速使用示例
void sd_simple_quick_example(void)
{
    uint8_t buffer[512];
    
    // 示例1：基本读写
    printf("=== SD Card Quick Example ===\n");
    
    // 初始化SD卡
    if (sd_simple_init() == 0) {
        printf("SD card initialized successfully.\n");
        
        // 读取扇区0的数据（MBR/引导扇区）
        if (sd_simple_read(buffer, 0, 1) == 0) {
            printf("Successfully read sector 0.\n");
            
            // 打印前16字节
            printf("First 16 bytes of sector 0:\n");
            for (int i = 0; i < 16; i++) {
                printf("%02X ", buffer[i]);
                if ((i + 1) % 8 == 0) printf(" ");
            }
            printf("\n");
        }
        
        // 获取SD卡信息
        sd_simple_info_struct info;
        sd_simple_get_info(&info);
        printf("SD card type: ");
        switch (info.type) {
            case SD_TYPE_SDV1: printf("SD V1.x\n"); break;
            case SD_TYPE_SDV2: printf("SD V2.0\n"); break;
            case SD_TYPE_SDHC: printf("SDHC\n"); break;
            case SD_TYPE_SDXC: printf("SDXC\n"); break;
            default: printf("Unknown\n"); break;
        }
        printf("Capacity: %u MB\n", info.capacity_mb);
    } else {
        printf("Failed to initialize SD card.\n");
    }
}

// 在主函数中使用
int sd_simple_main_example(void)
{
    // 初始化串口用于输出 - 使用UART0，波特率115200，TX引脚P14_0，RX引脚P14_1
    uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);
    
    printf("SD Simple Library Example\n");
    
    // 运行快速示例
    sd_simple_quick_example();
    
    // 运行完整测试（可选）
    // sd_simple_test();
    
    return 0;
}

