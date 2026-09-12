#ifndef _NAV_PATH_H_
#define _NAV_PATH_H_

#include "nav_types.h"

#define NAV_PATH_ROOT_DIR            "0:/NAV"
#define NAV_PATH_TASK1_DIR           "0:/TASK1"
#define NAV_PATH_DEFAULT_FILE        "0:/NAV/PATH.CSV"
#define NAV_PATH_TASK1_FILE          "0:/TASK1/PATH_T1.CSV"
#define NAV_PATH_TASK2_FILE          "0:/NAV/PATH_T2.CSV"
#define NAV_PATH_TASK3_FILE          "0:/NAV/PATH_T3.CSV"
#define NAV_PATH_TASK3_FOLLOW_FILE   "0:/NAV/PATH_T3_FOLLOW.CSV"
#define NAV_PATH_RECORD_LOG_FILE     "0:/NAV/PATH_RECORD_LOG.CSV"
#define NAV_PATH_TASK1_RECORD_LOG_FILE "0:/TASK1/PATH_RECORD_LOG.CSV"
#define NAV_PATH_RECORD_DIAG_FILE    "0:/NAV/PATH_RECORD_DIAG.CSV"
#define NAV_PATH_TASK1_RECORD_DIAG_FILE "0:/TASK1/PATH_RECORD_DIAG.CSV"
#define NAV_PATH_DEFAULT_INTERVAL_M  (0.04f)
#define NAV_PATH_MAX_SAMPLES         (2900U) /* 17-page follow-path capacity */
#define NAV_PATH_STORAGE_TASK3_FOLLOW_ID (4U)

void nav_path_init(void);
void nav_path_clear(void);
uint8 nav_path_start_record(float sample_interval_m);
uint8 nav_path_start_replay(void);
uint8 nav_path_start_replay_reverse(void);
uint8 nav_path_start_replay_reverse_fixed_yaw(void);
uint8 nav_path_start_replay_backward(void);
uint8 nav_path_start_replay_backward_fixed_yaw(void);
uint8 nav_path_start_replay_reverse_fixed_yaw_from_step(uint16 start_step);
uint8 nav_path_start_replay_reverse_fixed_ins_from_step(uint16 start_step);
void nav_path_set_replay_progress_gate(uint8 enable);
void nav_path_set_replay_rescue_project(uint8 enable);
uint8 nav_path_find_reverse_fixed_ins_match(float x_m,
                                            float y_m,
                                            float yaw_deg,
                                            uint16 max_step,
                                            nav_path_replay_match_t *out_match);
void nav_path_stop(void);
void nav_path_update(float distance_m, float x_m, float y_m, float yaw_deg, float speed_mps);
uint8 nav_path_save_to_sd(const char *filename);
uint8 nav_path_request_save_to_sd(uint8 task_id,
                                  const char *filename,
                                  uint8 flash_result,
                                  uint8 flash_load_result,
                                  uint8 ref_ready,
                                  uint8 ins_valid,
                                  const nav_ins_state_t *ins_state);
uint8 nav_path_sd_save_is_busy(void);
uint8 nav_path_force_last_sample_yaw(float yaw_deg);
uint8 nav_path_ensure_task1_sd_dir(void);
void nav_path_request_task1_sd_dir(void);
void nav_path_worker_task(uint32 now_ms);
uint8 nav_path_append_status_to_sd(uint8 task_id,
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
uint8 nav_path_append_record_diag_to_sd(uint8 task_id,
                                        const char *event,
                                        const nav_ins_state_t *ins_state);
uint8 nav_path_load_from_sd(const char *filename);
uint8 nav_path_save_to_flash(void);
uint8 nav_path_load_from_flash(void);
uint8 nav_path_save_to_flash_task(uint8 task_id);
uint8 nav_path_load_from_flash_task(uint8 task_id);
void nav_path_get_state(nav_path_state_t *out_state);
uint16 nav_path_get_sample_count(void);
float nav_path_get_sample_interval_m(void);
uint8 nav_path_get_replay_local_position(float x_m,
                                         float y_m,
                                         float *local_x_m,
                                         float *local_y_m);
uint8 nav_path_get_replay_point(uint16 index,
                                float *x_m,
                                float *y_m);
uint8 nav_path_get_yaw_deg(uint16 index, float *yaw_deg);
uint8 nav_path_get_sample(uint16 index,
                          float *x_m,
                          float *y_m,
                          float *yaw_deg,
                          float *kappa_1pm,
                          float *speed_mps);

#endif
