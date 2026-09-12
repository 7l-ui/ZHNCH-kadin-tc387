#ifndef _DATA_LOGGER_H_
#define _DATA_LOGGER_H_

#include "zf_common_headfile.h"
#include "nav_types.h"

typedef enum
{
    SD_APP_MODE_NONE = 0,
    SD_APP_MODE_LOGGER = 2,
    SD_APP_MODE_MAG_CAL = 3
} sd_app_mode_t;

typedef enum
{
    DATA_LOGGER_PROFILE_DRIVE_DEBUG = 0,
    DATA_LOGGER_PROFILE_FULL_NAV = 1,
    DATA_LOGGER_PROFILE_GPS_TEST = 2,
    DATA_LOGGER_PROFILE_IMU_TEST = 3,
    DATA_LOGGER_PROFILE_TASK2_INS_DEBUG = 4
} data_logger_profile_t;

typedef enum
{
    DATA_LOGGER_CAMERA_SNAPSHOT_FULL = 0,
    DATA_LOGGER_CAMERA_SNAPSHOT_HALF,
    DATA_LOGGER_CAMERA_SNAPSHOT_QUARTER,
    DATA_LOGGER_CAMERA_SNAPSHOT_SIZE_COUNT
} data_logger_camera_snapshot_size_t;

typedef enum
{
    DATA_LOGGER_CAMERA_SNAPSHOT_IDLE = 0,
    DATA_LOGGER_CAMERA_SNAPSHOT_PENDING,
    DATA_LOGGER_CAMERA_SNAPSHOT_WRITING,
    DATA_LOGGER_CAMERA_SNAPSHOT_DONE,
    DATA_LOGGER_CAMERA_SNAPSHOT_FAILED
} data_logger_camera_snapshot_state_t;

typedef enum
{
    DATA_LOGGER_CAMERA_RECORD_IDLE = 0,
    DATA_LOGGER_CAMERA_RECORD_STARTING,
    DATA_LOGGER_CAMERA_RECORD_RUNNING,
    DATA_LOGGER_CAMERA_RECORD_STOPPING,
    DATA_LOGGER_CAMERA_RECORD_DONE,
    DATA_LOGGER_CAMERA_RECORD_FAILED
} data_logger_camera_record_state_t;

typedef enum
{
    DATA_LOGGER_CAMERA_TEST_IDLE = 0,
    DATA_LOGGER_CAMERA_TEST_STARTING,
    DATA_LOGGER_CAMERA_TEST_RUNNING,
    DATA_LOGGER_CAMERA_TEST_STOPPING,
    DATA_LOGGER_CAMERA_TEST_DONE,
    DATA_LOGGER_CAMERA_TEST_FAILED
} data_logger_camera_test_state_t;

#define DATA_LOGGER_CAMERA_TEST_MODE_STATIC  (1U)
#define DATA_LOGGER_CAMERA_TEST_MODE_MOTION  (2U)

#define DATA_LOGGER_CAMERA_TEST_REASON_MANUAL      (1U)
#define DATA_LOGGER_CAMERA_TEST_REASON_LOST        (2U)
#define DATA_LOGGER_CAMERA_TEST_REASON_CONFIDENCE  (3U)
#define DATA_LOGGER_CAMERA_TEST_REASON_OFFSET_JUMP (4U)
#define DATA_LOGGER_CAMERA_TEST_REASON_DISTANCE_JUMP (5U)
#define DATA_LOGGER_CAMERA_TEST_REASON_BOX_JUMP    (6U)

typedef struct
{
    uint32 time_ms;
    uint8 target_valid;
    uint8 confidence_pct;
    uint8 fill_pct;
    uint8 distance_valid;
    uint16 center_x;
    uint16 center_y;
    int16 offset_x;
    int16 bearing_deg_x10;
    uint16 bbox_x;
    uint16 bbox_y;
    uint16 bbox_width;
    uint16 bbox_height;
    uint32 area_px;
    uint16 distance_cm;
    uint16 process_us;
    uint16 camera_fps;
    uint16 vision_load_pct;
} data_logger_camera_frame_meta_t;

uint8 data_logger_init(void);
uint8 data_logger_start(void);
uint8 data_logger_start_task(uint8 task_id, data_logger_profile_t profile);
uint8 data_logger_start_task_src(uint8 task_id, data_logger_profile_t profile, uint8 target_src);
uint8 data_logger_restart_task_src(uint8 task_id, data_logger_profile_t profile, uint8 target_src);
uint8 data_logger_force_restart_task_src(uint8 task_id, data_logger_profile_t profile, uint8 target_src);
void data_logger_stop(void);
void data_logger_stop_cancel_restart(void);
void data_logger_mark(uint16 marker);
void data_logger_mark_now(uint16 marker);
void data_logger_gps_test_mark_point(uint16 point_id);
void data_logger_gps_test_mark_point_sample(uint16 point_id, const nav_gps_sample_t *gps_sample);
void data_logger_task(uint32 now_ms);
void data_logger_worker_task(uint32 now_ms);
uint8 data_logger_is_running(void);
uint8 data_logger_is_busy(void);
uint8 data_logger_get_state(void);
uint8 data_logger_get_last_error(void);
uint8 data_logger_get_profile(void);
uint8 data_logger_get_task_id(void);
uint8 data_logger_get_target_src(void);
uint32 data_logger_get_dropped_count(void);
const char *data_logger_get_filename(void);
uint8 data_logger_camera_snapshot_request(data_logger_camera_snapshot_size_t size,
                                          const uint16 *image);
uint8 data_logger_camera_snapshot_request_with_meta(
    data_logger_camera_snapshot_size_t size,
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta);
void data_logger_camera_snapshot_worker_task(uint32 now_ms);
uint8 data_logger_camera_snapshot_is_busy(void);
uint8 data_logger_camera_snapshot_get_state(void);
uint8 data_logger_camera_snapshot_get_size(void);
uint8 data_logger_camera_snapshot_get_last_error(void);
uint32 data_logger_camera_snapshot_get_bytes(void);
const char *data_logger_camera_snapshot_get_filename(void);
uint8 data_logger_camera_record_start(data_logger_camera_snapshot_size_t size,
                                      uint8 target_fps);
void data_logger_camera_record_stop(void);
uint8 data_logger_camera_record_frame_request(
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta);
void data_logger_camera_record_worker_task(uint32 now_ms);
uint8 data_logger_camera_record_is_active(void);
uint8 data_logger_camera_record_is_frame_busy(void);
uint8 data_logger_camera_record_get_state(void);
uint8 data_logger_camera_record_get_last_error(void);
uint8 data_logger_camera_record_get_target_fps(void);
uint32 data_logger_camera_record_get_frame_count(void);
uint32 data_logger_camera_record_get_dropped_count(void);
uint32 data_logger_camera_record_get_bytes(void);
const char *data_logger_camera_record_get_directory(void);
uint8 data_logger_camera_test_start(uint8 mode);
uint8 data_logger_camera_test_start_parallel(uint8 mode);
void data_logger_camera_test_stop(void);
uint8 data_logger_camera_test_frame_request(
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta,
    uint8 reason);
void data_logger_camera_test_worker_task(uint32 now_ms);
uint8 data_logger_camera_test_is_active(void);
uint8 data_logger_camera_test_get_state(void);
uint8 data_logger_camera_test_get_mode(void);
uint8 data_logger_camera_test_get_last_error(void);
uint8 data_logger_camera_test_get_last_reason(void);
uint32 data_logger_camera_test_get_event_count(void);
uint32 data_logger_camera_test_get_dropped_count(void);
uint32 data_logger_camera_test_get_bytes(void);
const char *data_logger_camera_test_get_directory(void);

#endif
