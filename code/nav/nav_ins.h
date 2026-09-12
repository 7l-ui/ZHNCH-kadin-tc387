#ifndef _NAV_INS_H_
#define _NAV_INS_H_

#include "nav_types.h"

void nav_ins_init(void);
void nav_ins_set_count_to_meter(float count_to_meter);
void nav_ins_set_speed_filter_alpha(float alpha);
void nav_ins_set_accel_speed_gain(float gain);
void nav_ins_reset_position(float x_m, float y_m);
void nav_ins_correct_position(float x_m, float y_m, float blend_gain, float snap_threshold_m);
void nav_ins_reset_encoder(int16 encoder_r_count, int16 encoder_l_count, uint32 timestamp_ms);
void nav_ins_update_with_accel(uint32 timestamp_ms,
                               int16 encoder_r_count,
                               int16 encoder_l_count,
                               float yaw_deg,
                               float yaw_rate_dps,
                               float forward_accel_mps2);
void nav_ins_update(uint32 timestamp_ms,
                    int16 encoder_r_count,
                    int16 encoder_l_count,
                    float yaw_deg,
                    float yaw_rate_dps);
void nav_ins_get_state(nav_ins_state_t *out_state);

#endif
