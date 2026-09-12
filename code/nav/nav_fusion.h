#ifndef _NAV_FUSION_H_
#define _NAV_FUSION_H_

#include "nav_types.h"

void nav_fusion_init(void);
void nav_fusion_set_gps_timeout_ms(uint32 timeout_ms);
void nav_fusion_update_from_gps(const nav_gps_sample_t *gps_sample);
void nav_fusion_update_from_ins(const nav_ins_state_t *ins_state);
void nav_fusion_get_state(nav_state_t *out_state);
uint8 nav_fusion_is_valid(void);

#endif
