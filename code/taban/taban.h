#ifndef _TABAN_H_
#define _TABAN_H_

#include "zf_common_headfile.h"

typedef enum
{
    TABAN_ID_0 = 0,
    TABAN_ID_1,
    TABAN_ID_MAX
} taban_id_t;

typedef struct
{
    uint16 raw_adc[TABAN_ID_MAX];
    uint16 filt_adc[TABAN_ID_MAX];
    uint16 throttle_permille[TABAN_ID_MAX];
    uint8 released_once[TABAN_ID_MAX];
    uint8 fault_active[TABAN_ID_MAX];
    float target_speed_mps;
} taban_status_t;

void taban_init(void);
void taban_task(void);
uint16 taban_get_raw_adc(taban_id_t pedal_id);
uint16 taban_get_filt_adc(taban_id_t pedal_id);
uint16 taban_get_throttle_permille(taban_id_t pedal_id);
uint8 taban_is_released_once(taban_id_t pedal_id);
uint8 taban_is_fault_active(taban_id_t pedal_id);
float taban_get_target_speed_mps(void);
void taban_get_status(taban_status_t *status);

#endif
