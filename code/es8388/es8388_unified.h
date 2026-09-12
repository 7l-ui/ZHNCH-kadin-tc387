/**
 ****************************************************************************************************
 * @file        es8388_unified.h
 * @author      ����ԭ���Ŷ�(ALIENTEK) / ��ɿƼ�
 * @version     V2.0
 * @date        2026-03-23
 * @brief       ES8388����������TC387��ֲ�棩- ���ɲ��ź�¼�����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 * 
 * ����˵����
 * 1. ES8388����������I2C�Ĵ�����д��
 * 2. ES8388��Ƶ���Ź��ܣ�DAC�����
 * 3. ES8388¼����ܣ�ADC���룩
 * 4. ����I2S�ӿڿ���
 * 
 * ��ֲ˵����
 * 1. ��ԭSTM32F407������ֲ��Infineon TC387ƽ̨
 * 2. ʹ����ɿƼ�TC387��Դ������IIC����
 * 3. ����TC387��GPIO��ϵͳ�ӿ�
 * 4. ���ɲ��ź�¼����ܵ������ļ�
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407�����壨ԭ�棩
 *          ��ɿƼ� TC387�����壨��ֲ�棩
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211116 ��һ�η�����ԭ�棩
 * V2.0 20260323 TC387��ֲ�棨����������
 * V3.0 20260323 ���ɲ��ź�¼�����
 *
 ****************************************************************************************************
 */
 
#ifndef __ES8388_UNIFIED_H
#define __ES8388_UNIFIED_H

#include "../../libraries/zf_common/zf_common_typedef.h"
#include "../../libraries/zf_driver/zf_driver_gpio.h"
#include "es8388_config.h"

#define ES8388_ADDR     ES8388_IIC_ADDR    /* ES8388��������ַ,��CE(AD0)���� */

// ==================== ������������ ====================

/**
 * @brief       ES8388��ʼ��
 * @param       ��
 * @retval      0,��ʼ������
 *              ����,�������
 */
uint8_t es8388_init(void);

/**
 * @brief       ES8388д�Ĵ���
 * @param       reg : �Ĵ�����ַ
 * @param       val : Ҫд��Ĵ�����ֵ
 * @retval      0,�ɹ�
 *              ����,�������
 */
uint8_t es8388_write_reg(uint8_t reg, uint8_t val);

/**
 * @brief       ES8388���Ĵ���
 * @param       reg : �Ĵ�����ַ
 * @retval      ��ȡ��������
 */
uint8_t es8388_read_reg(uint8_t reg);

/**
 * @brief       ��ȡ��ǰ����ʵ��ʹ�õ�I2C����
 * @param       addr    : ����7λ��ַ(��ΪNULL)
 * @param       scl_pin : ����SCL����(��ΪNULL)
 * @param       sda_pin : ����SDA����(��ΪNULL)
 * @retval      0
 */
uint8_t es8388_get_runtime_i2c(uint8_t *addr, gpio_pin_enum *scl_pin, gpio_pin_enum *sda_pin);
uint8_t es8388_is_ready(void);

/**
 * @brief       ����ES8388����ģʽ
 * @param       fmt : ����ģʽ
 *    @arg      0, �����ֱ�׼I2S;
 *    @arg      1, MSB(�����);
 *    @arg      2, LSB(�Ҷ���);
 *    @arg      3, PCM/DSP
 * @param       len : ���ݳ���
 *    @arg      0, 24bit
 *    @arg      1, 20bit 
 *    @arg      2, 18bit 
 *    @arg      3, 16bit 
 *    @arg      4, 32bit 
 * @retval      ��
 */
void es8388_i2s_cfg(uint8_t fmt, uint8_t len);

/**
 * @brief       ���ö�������
 * @param       volume : ������С(0 ~ 33)
 * @retval      ��
 */
void es8388_hpvol_set(uint8_t volume);

/**
 * @brief       ������������
 * @param       volume : ������С(0 ~ 33)
 * @retval      ��
 */
void es8388_spkvol_set(uint8_t volume);

/**
 * @brief       ����3D������
 * @param       depth : 0 ~ 7(3Dǿ��,0�ر�,7��ǿ)
 * @retval      ��
 */
void es8388_3d_set(uint8_t depth);

/**
 * @brief       ES8388 DAC/ADC����
 * @param       dacen : dacʹ��(1)/�ر�(0)
 * @param       adcen : adcʹ��(1)/�ر�(0)
 * @retval      ��
 */
void es8388_adda_cfg(uint8_t dacen, uint8_t adcen);

/**
 * @brief       ES8388 DAC���ͨ������
 * @param       o1en : ͨ��1ʹ��(1)/��ֹ(0)
 * @param       o2en : ͨ��2ʹ��(1)/��ֹ(0)
 * @retval      ��
 */
void es8388_output_cfg(uint8_t o1en, uint8_t o2en);

/**
 * @brief       ES8388 MIC��������(MIC PGA����)
 * @param       gain : 0~8, ��Ӧ0~24dB  3dB/Step
 * @retval      ��
 */
void es8388_mic_gain(uint8_t gain);

/**
 * @brief       ES8388 ALC����
 * @param       sel
 *   @arg       0,�ر�ALC
 *   @arg       1,��ͨ��ALC
 *   @arg       2,��ͨ��ALC
 *   @arg       3,������ALC
 * @param       maxgain : 0~7,��Ӧ-6.5~+35.5dB
 * @param       mingain : 0~7,��Ӧ-12~+30dB 6dB/STEP
 * @retval      ��
 */
void es8388_alc_ctrl(uint8_t sel, uint8_t maxgain, uint8_t mingain);

/**
 * @brief       ES8388 ADC���ͨ������
 * @param       in : ����ͨ��
 *    @arg      0, ͨ��1����
 *    @arg      1, ͨ��2����
 * @retval      ��
 */
void es8388_input_cfg(uint8_t in);

// ==================== ��Ƶ���Ź��� ====================

/**
 * @brief       ��ʼ��ES8388��Ƶ����ϵͳ
 * @param       ��
 * @retval      0:�ɹ�������:ʧ��
 * @note       �˺������ʼ��ES8388��I2S���ͽӿ�
 */
uint8_t es8388_play_init(void);

/**
 * @brief       ֹͣ��ǰ����
 * @param       ��
 * @retval      ��
 * @note       ���ֹͣ���ڲ��ŵ���Ƶ
 */
void es8388_play_stop(void);

/**
 * @brief       ��鲥���Ƿ����ڽ���
 * @param       ��
 * @retval      1:���ڽ��У�0:����
 */
uint8_t es8388_play_is_busy(void);

/**
 * @brief       ���ò�������
 * @param       volume : ������С��0~33��
 * @param       is_headphone : 1:����������0:��������
 * @retval      ��
 */
void es8388_set_play_volume(uint8_t volume, uint8_t is_headphone);

// ==================== �����Խӿڣ�������horn_simple���ݣ� ====================

// ����ģʽ���壨��horn_simple.h�е�horn_simple_mode_enum��Ӧ��
typedef enum {
    ES8388_PLAY_BEEP_1S = 0,      // ����1�������
    ES8388_PLAY_BEEP_2S,          // ����2�������
    ES8388_PLAY_BEEP_3S,          // ����3�������
    ES8388_PLAY_BEEP_2_TIMES,     // ����2�η�����
    ES8388_PLAY_BEEP_3_TIMES,     // ����3�η�����
    ES8388_PLAY_BEEP_4_TIMES,     // ����4�η�����
    ES8388_PLAY_LONG_SHORT,       // ������
    ES8388_PLAY_RAPID,            // ���ٷ�����
    ES8388_PLAY_ALARM             // ������
} es8388_play_mode_enum;

/**
 * @brief       ������Ƶ�����horn_simple_beep��
 * @param       mode : ����ģʽ
 * @retval      ��
 * @note       ����ʽ������������ɺ󷵻�
 */
void es8388_play_beep(es8388_play_mode_enum mode);

/**
 * @brief       ES8388������ʾ
 * @param       ��
 * @retval      ��
 * @note       ������ʾ���в���ģʽ�����ڲ���
 */
void es8388_play_demo(void);

// ==================== ¼����� ====================

// ¼��״̬ö��
typedef enum {
    ES8388_RECORD_STOPPED = 0,    // ֹͣ״̬
    ES8388_RECORD_RUNNING,        // ¼����
    ES8388_RECORD_PAUSED,         // ��ͣ״̬
    ES8388_RECORD_ERROR           // ����״̬
} es8388_record_state_enum;

// ¼������ṹ��
typedef struct {
    uint32_t sample_rate;         // �����ʣ�Hz��
    uint8_t bits_per_sample;      // ����λ����16/24/32��
    uint8_t channels;             // ͨ������1:��������2:��������
    uint8_t input_source;         // ����Դ��0:��·���룬1:��˷����룩
} es8388_record_config_struct;

/**
 * @brief       ��ʼ��ES8388¼��ϵͳ
 * @param       config : ¼�����ò�������ΪNULLʹ��Ĭ��ֵ��
 * @retval      0:�ɹ�������:ʧ��
 * @note       �˺�������ES8388��ADC��I2S����ģʽ
 */
uint8_t es8388_record_init(es8388_record_config_struct *config);

/**
 * @brief       ��ʼ¼��
 * @param       ��
 * @retval      0:�ɹ�������:ʧ��
 * @note       ��ʼ��ES8388������Ƶ����
 */
uint8_t es8388_record_start(void);
uint8_t es8388_record_relock_clock(void);
uint8_t es8388_record_recover_clock(void);

/**
 * @brief       ֹͣ¼��
 * @param       ��
 * @retval      0:�ɹ�������:ʧ��
 * @note       ֹͣ¼����ͷ���Դ
 */
uint8_t es8388_record_stop(void);
uint8_t es8388_record_stop_keep_clock(void);

/**
 * @brief       ��ͣ¼��
 * @param       ��
 * @retval      0:�ɹ�������:ʧ��
 * @note       ��ͣ¼������Լ���
 */
uint8_t es8388_record_pause(void);

/**
 * @brief       ����¼��
 * @param       ��
 * @retval      0:�ɹ�������:ʧ��
 * @note       ����ͣ״̬����¼��
 */
uint8_t es8388_record_resume(void);

/**
 * @brief       ��ȡ��Ƶ���ݣ�����ʽ��
 * @param       buffer : ��Ƶ���ݻ�����
 * @param       samples : ��Ҫ��ȡ����������ÿ��ͨ����
 * @retval      ʵ�ʶ�ȡ��������
 * @note       ����ֱ����ȡ��ָ������������
 */
uint32_t es8388_record_read(int16_t *buffer, uint32_t samples);

/**
 * @brief       ��ȡ��Ƶ���ݣ�������ʽ��
 * @param       buffer : ��Ƶ���ݻ�����
 * @param       samples : ��Ҫ��ȡ����������ÿ��ͨ����
 * @retval      ʵ�ʶ�ȡ��������
 * @note       �������أ���ȡ��ǰ���õ�����
 */
uint32_t es8388_record_try_read(int16_t *buffer, uint32_t samples);

/**
 * @brief       ��ȡ¼��״̬
 * @param       ��
 * @retval      ¼��״̬
 */
es8388_record_state_enum es8388_record_get_state(void);

/**
 * @brief       ��ȡ��¼�Ƶ���������
 * @param       ��
 * @retval      ��������
 */
uint32_t es8388_record_get_total_samples(void);

/**
 * @brief       ��ȡ¼��ʱ�����룩
 * @param       ��
 * @retval      ¼��ʱ�����룩
 * @note       ���ݲ����ʺ�������������
 */
uint32_t es8388_record_get_duration_seconds(void);

/**
 * @brief       ������˷�����
 * @param       gain : ����ֵ��0~8����Ӧ0~24dB��
 * @retval      ��
 */
void es8388_record_set_mic_gain(uint8_t gain);

/**
 * @brief       ��������Դ
 * @param       source : ����Դ��0:��·���룬1:��˷����룩
 * @retval      ��
 */
void es8388_record_set_input_source(uint8_t source);

// Recording IIS format: 0=I2S, 1=Left-Justified, 2=Right-Justified
void es8388_record_set_iis_format(uint8_t fmt);
uint8_t es8388_record_get_iis_format(void);
uint8_t es8388_record_override_ws_pin(gpio_pin_enum ws_pin);

typedef void (*es8388_record_callback_t)(int16_t *data, uint32_t samples, void *user_data);

/**
 * @brief       ����¼��ص�����
 * @param       callback : �ص�����ָ��
 * @param       user_data : �û�����
 * @retval      ��
 * @note       ���ú���Զ����ûص�����������Ƶ����
 */
void es8388_record_set_callback(es8388_record_callback_t callback, void *user_data);

/**
 * @brief       ¼����ʾ����
 * @param       ��
 * @retval      ��
 * @note       ¼��5����Ƶ��ͨ�����ڴ�ӡ��Ϣ
 */
void es8388_record_demo(void);

// ==================== �߼����� ====================

/**
 * @brief       �л�����ģʽ
 * @param       mode : 0:����ģʽ��1:¼��ģʽ
 * @retval      0:�ɹ�������:ʧ��
 * @note       �����ڲ��ź�¼��֮���л���������Դ��ͻ
 */
uint8_t es8388_switch_mode(uint8_t mode);

/**
 * @brief       ����������ʾ
 * @param       ��
 * @retval      ��
 * @note       ��ʾ���й��ܣ����š�¼���ģʽ�л���
 */
void es8388_full_demo(void);

#endif /* __ES8388_UNIFIED_H */
