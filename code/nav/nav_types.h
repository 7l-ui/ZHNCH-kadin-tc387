#ifndef _NAV_TYPES_H_
#define _NAV_TYPES_H_

#include "zf_common_headfile.h"

#define NAV_PI  (3.14159265358979323846f)

typedef struct
{
    /* Local frame: x points east, y points north, unit is meter. */
    float x_m;
    float y_m;
} nav_point_t;

typedef struct
{
    nav_point_t start;
    nav_point_t end;
    uint8 valid;
    /* Standard line form: a*x + b*y + c = 0. */
    float a;
    float b;
    float c;
} nav_line_t;

typedef enum
{
    NAV_ARC_DIR_CW = 0,
    NAV_ARC_DIR_CCW = 1
} nav_arc_dir_t;

typedef struct
{
    nav_point_t center;
    float start_deg;
    float end_deg;
    float radius_m;
    nav_arc_dir_t dir;
    uint8 valid;
} nav_arc_t;

typedef struct
{
    uint8 valid;
    double origin_latitude_deg;
    double origin_longitude_deg;
    double meters_per_degree_lat;
    double meters_per_degree_lon;
} nav_local_frame_t;

typedef struct
{
    uint8 valid;
    uint8 satellite_used;
    uint32 timestamp_ms;
    double latitude_deg;
    double longitude_deg;
    float x_m;
    float y_m;
    float speed_mps;
    float course_deg;
} nav_gps_sample_t;

typedef struct
{
    uint8 valid;
    uint16 sample_count;
    uint16 required_count;
    uint32 first_timestamp_ms;
    uint32 last_timestamp_ms;
    double latitude_sum_deg;
    double longitude_sum_deg;
    float speed_sum_mps;
    float course_sin_sum;
    float course_cos_sum;
    uint32 satellite_sum;
} nav_gps_average_t;

typedef struct
{
    uint8 valid;
    uint32 timestamp_ms;
    /* INS position is dead-reckoned from forward motion and yaw. */
    float x_m;
    float y_m;
    float yaw_deg;
    float yaw_rate_dps;
    float forward_accel_mps2;
    float speed_mps;
    float encoder_speed_mps;
    float distance_m;
    int16 encoder_r_count;
    int16 encoder_l_count;
    uint32 update_dt_ms;
    int32 encoder_delta_r;
    int32 encoder_delta_l;
    float encoder_delta_avg;
    float delta_distance_m;
    float delta_x_m;
    float delta_y_m;
    float yaw_used_deg;
    float count_to_meter;
} nav_ins_state_t;

typedef enum
{
    NAV_PATH_MODE_IDLE = 0,
    NAV_PATH_MODE_RECORD = 1,
    NAV_PATH_MODE_REPLAY = 2
} nav_path_mode_t;

typedef struct
{
    uint8 valid;
    uint8 active;
    uint8 finished;
    uint8 last_sd_result;
    uint8 last_flash_result;
    uint8 last_flash_load_result;
    nav_path_mode_t mode;
    uint16 sample_count;
    uint16 replay_index;
    float sample_interval_m;
    float progress_m;
    float ref_x_m;
    float ref_y_m;
    float ref_yaw_deg;
    float ref_kappa_1pm;
    float ref_kappa_ahead_1pm;
    float ref_kappa_yaw_1pm;
    float ref_speed_mps;
    float ref_speed_ahead_mps;
    int8 ref_motion_dir;
    int8 ref_motion_dir_ahead;
    float ref_dir_change_distance_m;
    uint16 ref_dir_change_samples;
    float yaw_error_deg;
    float lateral_error_m;
} nav_path_state_t;

typedef struct
{
    uint8 valid;
    uint16 start_step;
    uint16 replay_index;
    float ref_x_m;
    float ref_y_m;
    float ref_yaw_deg;
    float distance_m;
    float lateral_error_m;
    float longitudinal_error_m;
    float yaw_error_deg;
    float score;
} nav_path_replay_match_t;

typedef struct
{
    uint8 valid;
    uint8 gps_valid;
    uint8 ins_valid;
    uint8 gps_ins_correction_enabled;
    uint32 timestamp_ms;
    uint32 last_gps_ms;
    float x_m;
    float y_m;
    float yaw_deg;
    float yaw_rate_dps;
    float forward_accel_mps2;
    float speed_mps;
    float encoder_speed_mps;
    float distance_m;
    float gps_ins_error_m;
    float gps_ins_correction_gain;
    float gps_ins_snap_threshold_m;
    uint8 path_valid;
    uint8 path_active;
    uint8 path_finished;
    uint8 path_last_sd_result;
    uint8 path_last_flash_result;
    uint8 path_last_flash_load_result;
    nav_path_mode_t path_mode;
    uint16 path_sample_count;
    uint16 path_replay_index;
    float path_sample_interval_m;
    float path_progress_m;
    float path_ref_x_m;
    float path_ref_y_m;
    float path_ref_yaw_deg;
    float path_ref_kappa_1pm;
    float path_ref_kappa_ahead_1pm;
    float path_ref_kappa_yaw_1pm;
    float path_ref_speed_mps;
    float path_ref_speed_ahead_mps;
    int8 path_ref_motion_dir;
    int8 path_ref_motion_dir_ahead;
    float path_yaw_error_deg;
    float path_lateral_error_m;
} nav_state_t;

#endif
