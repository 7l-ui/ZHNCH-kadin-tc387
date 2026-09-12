#ifndef GUIMAI_ADC_AUDIO_H
#define GUIMAI_ADC_AUDIO_H

#include "guimai_voice_config.h"
#include "zf_common_headfile.h"

uint8 guimai_adc_audio_init(void);
uint8 guimai_adc_audio_start(void);
void guimai_adc_audio_stop(void);
uint32 guimai_adc_audio_read(int16 *dst, uint32 capacity);
/* 0=OK, 1=channel, 2=clock, 3=ADC init, 4=DMA busy/reset,
 * 5=ring overrun/poll gap, 6=DMA request lost, 7=no samples. */
uint8 guimai_adc_audio_error(void);

#endif
