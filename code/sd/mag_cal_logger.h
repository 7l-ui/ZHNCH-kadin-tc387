#ifndef _MAG_CAL_LOGGER_H_
#define _MAG_CAL_LOGGER_H_

#include "zf_common_headfile.h"

uint8 mag_cal_logger_start(void);
void mag_cal_logger_set_auto_stop_ms(uint32 duration_ms);
void mag_cal_logger_stop(void);
void mag_cal_logger_task(uint32 now_ms);
void mag_cal_logger_worker_task(uint32 now_ms);
uint8 mag_cal_logger_is_running(void);
uint8 mag_cal_logger_get_state(void);
uint8 mag_cal_logger_get_last_error(void);
const char *mag_cal_logger_get_filename(void);

#endif
