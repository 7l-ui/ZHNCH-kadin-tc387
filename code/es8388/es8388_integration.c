/**
 ****************************************************************************************************
 * @file        es8388_integration.c
 * @author      逐飞科技 / 正点原子团队
 * @version     V1.0
 * @date        2026-03-23
 * @brief       ES8388与鸣笛系统集成实现
 * 
 * 功能说明：
 * 1. 集成ES8388驱动和鸣笛系统
 * 2. 通过ES8388 DAC播放鸣笛声音
 * 3. 兼容现有horn_simple接口
 ****************************************************************************************************
 * @attention
 *
 * 硬件连接要求：
 * 1. I2C连接：SCL -> ES8388_SCL_PIN, SDA -> ES8388_SDA_PIN
 * 2. I2S连接：BCK -> ES8388_I2S_BCK_PIN, WS -> ES8388_I2S_WS_PIN, SD -> ES8388_I2S_SD_PIN
 * 3. 注意：horn_config.h中的引脚配置应与ES8388的I2S输入引脚一致
 *
 ****************************************************************************************************
 */

#include "es8388_integration.h"
#include "zf_driver_soft_iis.h"
#include "es8388_config.h"

// 全局状态
static uint8_t es8388_horn_busy = 0;
static uint8_t es8388_horn_stop_request = 0;

// IIS对象
static soft_iis_info_struct es8388_iis_obj;

// 音频缓冲区
#define ES8388_AUDIO_BUFFER_SIZE 512
static int16 es8388_audio_buffer[ES8388_AUDIO_BUFFER_SIZE];

// 正弦波表（与horn_simple.c中的相同，确保兼容）
static const int16 es8388_sine_1000hz[] = {
    0, 4339, 8192, 11039, 12539, 12539, 11039, 8192, 4339, 0, -4339, -8192, -11039, -12539, 
    -12539, -11039, -8192, -4339, 0, 4339, 8192, 11039, 12539, 12539, 11039, 8192, 4339, 0,
    -4339, -8192, -11039, -12539, -12539, -11039, -8192, -4339, 0, 4339, 8192, 11039, 12539, 12539, 11039, 8192
};

static const int16 es8388_sine_500hz[] = {
    0, 2169, 4339, 6508, 8192, 9512, 10499, 11039, 11039, 10499, 9512, 8192, 6508, 4339, 
    2169, 0, -2169, -4339, -6508, -8192, -9512, -10499, -11039, -11039, -10499, -9512, 
    -8192, -6508, -4339, -2169, 0, 2169, 4339, 6508, 8192, 9512, 10499, 11039, 11039, 
    10499, 9512, 8192, 6508, 4339, 2169, 0, -2169, -4339, -6508, -8192, -9512, -10499, 
    -11039, -11039, -10499, -9512, -8192, -6508, -4339, -2169, 0, 2169, 4339, 6508, 8192, 
    9512, 10499, 11039, 11039, 10499, 9512, 8192, 6508, 4339, 2169, 0, -2169, -4339, -6508, 
    -8192, -9512, -10499, -11039, -11039, -10499, -9512, -8192, -6508, -4339, -2169
};

//-------------------------------------------------------------------------------------------------------------------
// 内部函数：填充音频缓冲区并通过I2S发送
//-------------------------------------------------------------------------------------------------------------------
static void es8388_fill_audio_buffer(uint32_t duration_ms, uint32_t freq_hz)
{
    const int16 *wave_table;
    uint32_t table_size;
    uint32_t samples_needed;
    
    // 选择波形表
    if (freq_hz == 500) {
        wave_table = es8388_sine_500hz;
        table_size = 88;
    } else { // 1000Hz
        wave_table = es8388_sine_1000hz;
        table_size = 44;
    }
    
    // 计算需要多少样本
    samples_needed = duration_ms * HORN_SAMPLE_RATE / 1000;
    
    // 分批填充和发送音频数据
    uint32_t samples_sent = 0;
    uint32_t table_index = 0;
    
    while (samples_sent < samples_needed && !es8388_horn_stop_request)
    {
        // 计算本次发送的样本数
        uint32_t batch_size = ES8388_AUDIO_BUFFER_SIZE;
        if (samples_sent + batch_size > samples_needed) {
            batch_size = samples_needed - samples_sent;
        }
        
        // 填充缓冲区
        for (uint32_t i = 0; i < batch_size; i++)
        {
            es8388_audio_buffer[i] = wave_table[table_index];
            table_index++;
            if (table_index >= table_size) {
                table_index = 0;
            }
        }
        
        // 通过I2S发送音频数据到ES8388
        soft_iis_send_16bit_array(&es8388_iis_obj, es8388_audio_buffer, batch_size, 0);
        
        samples_sent += batch_size;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 内部函数：播放静音（间隔）
//-------------------------------------------------------------------------------------------------------------------
static void es8388_play_silence(uint32_t duration_ms)
{
    uint32_t samples_needed = duration_ms * HORN_SAMPLE_RATE / 1000;
    
    // 清零缓冲区
    for (uint32_t i = 0; i < ES8388_AUDIO_BUFFER_SIZE; i++) {
        es8388_audio_buffer[i] = 0;
    }
    
    uint32_t samples_sent = 0;
    
    while (samples_sent < samples_needed && !es8388_horn_stop_request)
    {
        uint32_t batch_size = ES8388_AUDIO_BUFFER_SIZE;
        if (samples_sent + batch_size > samples_needed) {
            batch_size = samples_needed - samples_sent;
        }
        
        // 发送静音数据
        soft_iis_send_16bit_array(&es8388_iis_obj, es8388_audio_buffer, batch_size, 0);
        
        samples_sent += batch_size;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// ES8388鸣笛系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8_t es8388_horn_init(void)
{
    uint8_t ret;
    
    // 1. 初始化ES8388（通过I2C配置）
    ret = es8388_init();
    if (ret != 0) {
        return ret;  // ES8388初始化失败
    }
    
    // 2. 配置ES8388的I2S格式
    // 注意：这里的格式需要与软件IIS配置一致
    es8388_i2s_cfg(ES8388_DEFAULT_I2S_FORMAT, ES8388_DEFAULT_DATA_LEN);
    
    // 3. 初始化IIS为主发送模式
    // 使用horn_config.h中配置的引脚（这些引脚应连接到ES8388的I2S输入）
    soft_iis_init_master_tx(&es8388_iis_obj, 
                            HORN_SAMPLE_RATE, 
                            SOFT_IIS_BITS_16, 
                            SOFT_IIS_FORMAT_I2S,
                            HORN_BCK_PIN,   // BCK引脚 -> ES8388 BCLK
                            HORN_WS_PIN,    // WS引脚  -> ES8388 LRCK
                            HORN_SD_PIN,    // SD引脚  -> ES8388 DIN
                            HORN_MCLK_PIN); // MCLK引脚（如不使用设为0xFFFFFFFF）
    
    // 4. 开始IIS传输
    soft_iis_start(&es8388_iis_obj);
    
    // 5. 初始化状态
    es8388_horn_busy = 0;
    es8388_horn_stop_request = 0;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// ES8388鸣笛控制函数
//-------------------------------------------------------------------------------------------------------------------
void es8388_horn_beep(es8388_horn_mode_enum mode)
{
    // 设置忙状态
    es8388_horn_busy = 1;
    es8388_horn_stop_request = 0;
    
    switch (mode) {
        case ES8388_HORN_BEEP_1S:
            es8388_fill_audio_buffer(1000, 1000);
            break;
            
        case ES8388_HORN_BEEP_2S:
            es8388_fill_audio_buffer(2000, 1000);
            break;
            
        case ES8388_HORN_BEEP_3S:
            es8388_fill_audio_buffer(3000, 1000);
            break;
            
        case ES8388_HORN_BEEP_2_TIMES:
            es8388_fill_audio_buffer(1000, 1000);
            es8388_play_silence(1000);
            es8388_fill_audio_buffer(1000, 1000);
            break;
            
        case ES8388_HORN_BEEP_3_TIMES:
            for (int i = 0; i < 3; i++) {
                es8388_fill_audio_buffer(1000, 1000);
                if (i < 2 && !es8388_horn_stop_request) es8388_play_silence(1000);
                if (es8388_horn_stop_request) break;
            }
            break;
            
        case ES8388_HORN_BEEP_4_TIMES:
            for (int i = 0; i < 4; i++) {
                es8388_fill_audio_buffer(1000, 1000);
                if (i < 3 && !es8388_horn_stop_request) es8388_play_silence(1000);
                if (es8388_horn_stop_request) break;
            }
            break;
            
        case ES8388_HORN_LONG_SHORT:
            es8388_fill_audio_buffer(1000, 1000);
            es8388_play_silence(1000);
            es8388_fill_audio_buffer(3000, 1000);
            break;
            
        case ES8388_HORN_RAPID:
            for (int i = 0; i < 6; i++) {
                es8388_fill_audio_buffer(500, 1000);
                if (i < 5 && !es8388_horn_stop_request) es8388_play_silence(500);
                if (es8388_horn_stop_request) break;
            }
            break;
            
        case ES8388_HORN_ALARM:
            for (int i = 0; i < 6; i++) {
                if (i % 2 == 0) {
                    es8388_fill_audio_buffer(1000, 500);  // 500Hz
                } else {
                    es8388_fill_audio_buffer(1000, 1000); // 1000Hz
                }
                if (es8388_horn_stop_request) break;
            }
            break;
            
        default:
            es8388_fill_audio_buffer(1000, 1000);
            break;
    }
    
    // 清除忙状态
    es8388_horn_busy = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 停止当前鸣笛
//-------------------------------------------------------------------------------------------------------------------
void es8388_horn_stop(void)
{
    es8388_horn_stop_request = 1;
    
    // 等待鸣笛停止
    while (es8388_horn_busy) {
        // 短暂延时
        for (volatile int i = 0; i < 1000; i++);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 检查鸣笛是否正在进行
//-------------------------------------------------------------------------------------------------------------------
uint8_t es8388_horn_is_busy(void)
{
    return es8388_horn_busy;
}

//-------------------------------------------------------------------------------------------------------------------
// 设置ES8388音量
//-------------------------------------------------------------------------------------------------------------------
void es8388_set_volume(uint8_t volume, uint8_t is_headphone)
{
    if (is_headphone) {
        es8388_hpvol_set(volume);
    } else {
        es8388_spkvol_set(volume);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// ES8388鸣笛示例演示
//-------------------------------------------------------------------------------------------------------------------
void es8388_horn_demo(void)
{
    // 演示所有鸣笛模式
    es8388_horn_beep(ES8388_HORN_BEEP_1S);
    while(es8388_horn_is_busy()); // 等待完成
    
    // 间隔2秒
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_BEEP_2S);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_BEEP_3S);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_BEEP_2_TIMES);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_BEEP_3_TIMES);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_BEEP_4_TIMES);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_LONG_SHORT);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_RAPID);
    while(es8388_horn_is_busy());
    
    for(volatile int i = 0; i < 2000000; i++);
    
    es8388_horn_beep(ES8388_HORN_ALARM);
    while(es8388_horn_is_busy());
}
