#ifndef _NAV_CONTROL_H_
#define _NAV_CONTROL_H_

#include "nav_types.h"

/* Top-level navigation interface used by task/control code. */
void nav_control_init(void);
void nav_control_set_encoder_count_to_meter(float count_to_meter);
void nav_control_set_gps_timeout_ms(uint32 timeout_ms);
void nav_control_set_gps_ins_correction(uint8 enable, float blend_gain, float snap_threshold_m);
void nav_control_set_gps_origin(double latitude_deg, double longitude_deg);
uint8 nav_control_set_gps_origin_from_current(void);
/* Use this when a future GPS driver provides data without zf_device_gnss. */
uint8 nav_control_update_gps_sample(const nav_gps_sample_t *gps_sample);
/* Current project path: poll zf_device_gnss, parse, then feed fusion. */
uint8 nav_control_poll_gps(uint32 timestamp_ms);
void nav_control_update_ins_with_accel(uint32 timestamp_ms,
                                       int16 encoder_r_count,
                                       int16 encoder_l_count,
                                       float yaw_deg,
                                       float yaw_rate_dps,
                                       float forward_accel_mps2);
uint8 nav_control_update_ins_from_imu(uint32 timestamp_ms,
                                      int16 encoder_r_count,
                                      int16 encoder_l_count);
void nav_control_update_ins(uint32 timestamp_ms,
                            int16 encoder_r_count,
                            int16 encoder_l_count,
                            float yaw_deg,
                            float yaw_rate_dps);
void nav_control_get_state(nav_state_t *out_state);
uint8 nav_control_is_valid(void);

uint8 nav_control_path_start_record(float sample_interval_m);
uint8 nav_control_path_start_replay(void);
uint8 nav_control_path_start_replay_reverse(void);
uint8 nav_control_path_start_replay_reverse_fixed_yaw(void);
uint8 nav_control_path_start_replay_backward(void);
uint8 nav_control_path_start_replay_backward_fixed_yaw(void);
uint8 nav_control_path_start_replay_reverse_fixed_yaw_from_step(uint16 start_step);
uint8 nav_control_path_start_replay_reverse_fixed_ins_from_step(uint16 start_step);
void nav_control_path_set_replay_progress_gate(uint8 enable);
void nav_control_path_set_replay_rescue_project(uint8 enable);
uint8 nav_control_path_find_reverse_fixed_ins_match(float x_m,
                                                    float y_m,
                                                    float yaw_deg,
                                                    uint16 max_step,
                                                    nav_path_replay_match_t *out_match);
void nav_control_path_stop(void);
void nav_control_path_clear(void);
uint8 nav_control_path_get_replay_local_position(float x_m,
                                                 float y_m,
                                                 float *local_x_m,
                                                 float *local_y_m);
uint8 nav_control_path_get_replay_point(uint16 index,
                                        float *x_m,
                                        float *y_m);
uint8 nav_control_path_save_to_sd(const char *filename);
uint8 nav_control_path_request_save_to_sd(uint8 task_id,
                                          const char *filename,
                                          uint8 flash_result,
                                          uint8 flash_load_result,
                                          uint8 ref_ready,
                                          uint8 ins_valid,
                                          const nav_ins_state_t *ins_state);
uint8 nav_control_path_sd_save_is_busy(void);
uint8 nav_control_path_force_last_sample_yaw(float yaw_deg);
uint8 nav_control_path_append_status_to_sd(uint8 task_id,
                                           const char *event,
                                           uint8 sd_result,
                                           uint8 flash_result,
                                           uint8 flash_load_result,
                                           uint8 ref_ready,
                                           uint8 ins_valid,
                                           uint8 logger_state,
                                           uint8 logger_error,
                                           uint8 logger_task,
                                           uint8 logger_profile,
                                           uint8 logger_target_src);
uint8 nav_control_path_append_record_diag_to_sd(uint8 task_id,
                                                const char *event,
                                                const nav_ins_state_t *ins_state);
uint8 nav_control_path_load_from_sd(const char *filename);
uint8 nav_control_path_save_to_flash(void);
uint8 nav_control_path_load_from_flash(void);
uint8 nav_control_path_save_to_flash_task(uint8 task_id);
uint8 nav_control_path_load_from_flash_task(uint8 task_id);
void nav_control_path_get_state(nav_path_state_t *out_state);

#endif
