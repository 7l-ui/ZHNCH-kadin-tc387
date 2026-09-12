#include "data_logger.h"

#include "ff.h"
#include "sd_simple.h"
#include "pid.h"
#include "pid_config.h"
#include "dianji.h"
#include "isr.h"
#include "cpu0_main.h"
#include "dici.h"
#include "nav_control.h"
#include "nav_gps.h"
#include "nav_ins.h"
#include "task_ctrl.h"
#include "taban.h"
#include "yaokong.h"
#include "guimai/guimai_board.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

#define DATA_LOGGER_DRIVE_SAMPLE_PERIOD_MS (10U)
#define DATA_LOGGER_FULL_SAMPLE_PERIOD_MS  (50U)
#define DATA_LOGGER_GPS_TEST_SAMPLE_PERIOD_MS (50U)
#define DATA_LOGGER_IMU_TEST_SAMPLE_PERIOD_MS (50U)
#define DATA_LOGGER_TASK2_INS_SAMPLE_PERIOD_MS (20U)
#define DATA_LOGGER_SYNC_PERIOD_MS     (1000U)
#define DATA_LOGGER_BUFFER_SIZE        (4096U)
#define DATA_LOGGER_LINE_SIZE          (2048U)
#define DATA_LOGGER_QUEUE_SIZE         (64U)
#define DATA_LOGGER_MAX_FILES          (1000U)
#define DATA_LOGGER_TASK_COUNT         (5U)
#define DATA_LOGGER_TARGET_SRC_COUNT   (5U)
#define DATA_LOGGER_TASK2_FREEZE_SPEED_MPS (0.08f)
#define DATA_LOGGER_TASK2_FREEZE_STEER_DEG (10.0f)
#define DATA_LOGGER_TASK2_FREEZE_YAW_EPS_DEG (0.05f)
#define DATA_LOGGER_TASK2_FREEZE_GYRO_EPS_DPS (0.20f)
#define DATA_LOGGER_TASK2_FREEZE_SUSPECT_MS (500U)
#define DATA_LOGGER_CAMERA_SNAPSHOT_MAX_FILES       (1000U)
#define DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE (54U)
#define DATA_LOGGER_CAMERA_RECORD_MAX_DIRS           (1000U)
#define DATA_LOGGER_CAMERA_RECORD_MAX_FRAMES         (100000U)
#define DATA_LOGGER_CAMERA_RECORD_LINE_SIZE          (512U)
#define DATA_LOGGER_CAMERA_TEST_MAX_DIRS             (1000U)
#define DATA_LOGGER_CAMERA_TEST_MAX_EVENTS           (30U)
#define DATA_LOGGER_CAMERA_TEST_LINE_SIZE            (512U)
#define DATA_LOGGER_CAMERA_TEST_ROOT_DIR             "0:/LOG/TASK3_CAMERA_TEST"

#define DATA_LOGGER_STATE_IDLE         (0U)
#define DATA_LOGGER_STATE_STARTING     (1U)
#define DATA_LOGGER_STATE_RUNNING      (2U)
#define DATA_LOGGER_STATE_STOPPING     (3U)
#define DATA_LOGGER_STATE_FAILED       (4U)

extern volatile int16 g_ecod1_count;
extern volatile int16 g_ecod2_count;
extern volatile int16 g_ecod1_speed;
extern volatile int16 g_ecod2_speed;
extern volatile int16 g_ecod1_speed_filt;
extern volatile int16 g_ecod2_speed_filt;
extern volatile int16 g_abs_encoder_raw;
extern uint8 Key_Num;
extern volatile uint8 g_key_last_event_num;
extern volatile uint32 g_key_last_event_time_ms;
extern volatile uint32 g_key_event_count;
extern void sd_simple_reset(void);

typedef struct
{
    uint32 time_ms;
    uint8 task_id;
    uint8 task_state;
    uint8 profile;
    int16 abs_raw;
    int16 steer_center_raw;
    int16 steer_delta_counts;
    float steer_speed_counts_s;
    float steer_target_deg;
    float steer_feedback_deg;
    uint8 steer_valid;
    uint8 steer_sensor_fault;
    uint8 steer_stall_fault;
    uint8 steer_scan_phase;
    uint8 steer_scan_success;
    int16 steer_scan_left_raw;
    int16 steer_scan_right_raw;
    int16 steer_scan_center_raw;
    int16 steer_scan_span_counts;
    int16 ecod1_count;
    int16 ecod2_count;
    int16 ecod1_speed;
    int16 ecod2_speed;
    int16 ecod1_speed_filt;
    int16 ecod2_speed_filt;
    float target_speed_mps;
    float pid_out_r;
    float pid_out_l;
    float pid_out_steer;
    float pid_ff_r;
    float pid_p_r;
    float pid_i_r;
    float pid_d_r;
    uint16 pid_boost_r;
    uint8 pid_dir_guard_r;
    float pid_ff_l;
    float pid_p_l;
    float pid_i_l;
    float pid_d_l;
    uint16 pid_boost_l;
    uint8 pid_dir_guard_l;
    float pid_target_r;
    float pid_feedback_r;
    float pid_error_r;
    float pid_target_l;
    float pid_feedback_l;
    float pid_error_l;
    int16 duty_m;
    int16 duty_r;
    int16 duty_l;
    float imu_yaw;
    float gyro_z;
    float imu_rel_pitch;
    float imu_rel_roll;
    float imu_abs_pitch;
    float imu_abs_roll;
    float imu_abs_yaw;
    float imu_mag_raw_x;
    float imu_mag_raw_y;
    float imu_mag_raw_z;
    uint8 imu_init_ret;
    uint32 imu_pit_count;
    uint32 imu_update_count;
    uint32 imu_update_age_ms;
    uint32 imu_read_fail_count;
    imu_ahrs_debug_t imu_dbg;
    uint8 target_src;
    uint8 gnss_state;
    uint8 remote_connected;
    uint8 remote_armed;
    uint8 remote_state;
    uint8 remote_guard;
    uint8 remote_motor_run;
    uint8 remote_ch5_pressed;
    uint8 remote_ch6_pressed;
    uint16 remote_raw_steer;
    uint16 remote_raw_throttle;
    uint16 remote_raw_ch5;
    uint16 remote_raw_ch[YAOKONG_CHANNEL_NUM];
    float remote_target_speed_mps;
    float remote_target_steer_deg;
    uint32 remote_last_frame_age_ms;
    uint32 remote_valid_frames;
    uint32 remote_bad_frames;
    uint16 pedal_raw_adc[TABAN_ID_MAX];
    uint16 pedal_filt_adc[TABAN_ID_MAX];
    uint16 pedal_permille[TABAN_ID_MAX];
    uint8 pedal_released_once[TABAN_ID_MAX];
    uint8 pedal_fault_active[TABAN_ID_MAX];
    float pedal_target_speed_mps;
    nav_state_t nav_state;
    nav_ins_state_t ins_state;
    float imu_acc_x_g;
    float imu_acc_y_g;
    float imu_acc_z_g;
    nav_gps_sample_t gps_sample;
    float gps_test_origin_x_m;
    float gps_test_origin_y_m;
    float gps_test_drift_m;
    ctrl_cmd_t ctrl_cmd;
    uint16 marker;
    uint16 gps_test_point_id;
    uint8 gps_test_event;
    uint8 key_num;
    uint8 key_event_num;
    uint32 key_event_count;
    uint32 key_event_age_ms;
    uint8 ctrl_stop_reason;
    uint8 ctrl_safety_pause_active;
    uint32 ctrl_main_loop_age_ms;
    uint32 ctrl_cpu0_age_ms;
    uint8 ctrl_cpu0_stage;
    uint8 ctrl_cpu0_stall_stage;
    uint32 ctrl_cpu0_stall_count;
    uint32 ctrl_cpu0_stall_max_age_ms;
    uint32 ctrl_cmd_age_ms;
    uint32 steer_feedback_age_ms;
    uint32 ctrl_safety_pause_until_ms;
    uint8 ctrl_pid_reset_reason;
    uint32 ctrl_pid_reset_age_ms;
    uint8 ctrl_pid_low_reason;
    uint32 ctrl_pid_low_age_ms;
    float ctrl_pid_low_target_r;
    float ctrl_pid_low_feedback_r;
    float ctrl_pid_low_output_r;
    float ctrl_pid_low_target_l;
    float ctrl_pid_low_feedback_l;
    float ctrl_pid_low_output_l;
    uint32 imu_yaw_freeze_ms;
    uint8 imu_yaw_freeze_suspect;
    ctrl_task2_path_debug_t task2_path_dbg;
    ctrl_task3_follow_status_t task3_follow_status;
    uint32 logger_dropped;
    int16 steer_raw_read;
    int16 steer_raw_last_valid;
    int16 steer_raw_jump_counts;
    uint8 steer_raw_last_reject_reason;
    uint8 steer_raw_last_reject_streak;
    uint32 steer_raw_reject_count;
    uint32 steer_raw_last_reject_ms;
    int16 steer_raw_last_reject_raw;
    int16 steer_raw_last_reject_ref_raw;
    int16 steer_raw_last_reject_jump_counts;
} data_logger_sample_t;

static const char *DATA_LOGGER_FULL_HEADER =
    "time_ms,marker,task_id,task_state,profile,abs_raw,steer_center_raw,steer_delta_counts,steer_speed_counts_s,steer_target_deg,steer_feedback_deg,steer_valid,steer_sensor_fault,steer_stall_fault,"
    "steer_scan_phase,steer_scan_success,steer_scan_left_raw,steer_scan_right_raw,steer_scan_center_raw,steer_scan_span_counts,"
    "ecod1_count,ecod2_count,ecod1_speed,ecod2_speed,ecod1_speed_filt,ecod2_speed_filt,target_speed_mps,pid_out_r,pid_out_l,pid_dir_guard_r,pid_dir_guard_l,pid_out_steer,"
    "duty_m,duty_r,duty_l,imu_yaw,gyro_z,target_src,"
    "remote_connected,remote_armed,remote_state,remote_guard,remote_motor_run,remote_ch5_pressed,remote_raw_steer,remote_raw_throttle,remote_raw_ch5,remote_target_speed_mps,remote_target_steer_deg,remote_last_frame_age_ms,remote_valid_frames,remote_bad_frames,"
    "nav_valid,gps_valid,ins_valid,nav_timestamp_ms,ins_x_m,ins_y_m,ins_yaw_deg,ins_yaw_rate_dps,"
    "ins_speed_mps,encoder_speed_mps,distance_m,forward_accel_mps2,imu_acc_x_g,imu_acc_y_g,imu_acc_z_g,cmp_en,cmp_ok,shadow_algo,shadow_yaw,shadow_dyaw,"
    "path_valid,path_active,path_finished,path_mode,path_sample_count,path_replay_index,path_sample_interval_m,"
    "path_progress_m,path_ref_x_m,path_ref_y_m,path_ref_yaw_deg,path_ref_kappa_1pm,path_ref_kappa_ahead_1pm,path_ref_kappa_yaw_1pm,path_ref_speed_mps,"
    "path_ref_speed_ahead_mps,path_ref_motion_dir,path_ref_motion_dir_ahead,path_yaw_error_deg,path_lateral_error_m,path_sd,path_fsave,path_fload,logger_dropped,"
    "pid_target_r,pid_feedback_r,pid_error_r,pid_target_l,pid_feedback_l,pid_error_l,"
    "ctrl_use_pid,ctrl_direct_pwm,ctrl_steer_closed_loop,ctrl_brake_to_stop,ctrl_cmd_target_speed_mps,ctrl_cmd_target_angle_deg,"
    "ctrl_stop_reason,ctrl_safety_pause_active,ctrl_main_loop_age_ms,ctrl_cpu0_age_ms,ctrl_cpu0_stage,ctrl_cpu0_stall_stage,ctrl_cpu0_stall_count,ctrl_cpu0_stall_max_age_ms,ctrl_cmd_age_ms,steer_feedback_age_ms,ctrl_safety_pause_until_ms,"
    "ctrl_pid_reset_reason,ctrl_pid_reset_age_ms,ctrl_pid_low_reason,ctrl_pid_low_age_ms,"
    "ctrl_pid_low_target_r,ctrl_pid_low_feedback_r,ctrl_pid_low_output_r,ctrl_pid_low_target_l,ctrl_pid_low_feedback_l,ctrl_pid_low_output_l,remote_ch1_raw,remote_ch2_raw,remote_ch3_raw,remote_ch4_raw,remote_ch5_raw,remote_ch6_raw,remote_ch6_pressed,"
    "follow_active,follow_phase,follow_error,follow_target_valid,follow_target_stable,follow_confidence_pct,follow_distance_valid,follow_bearing_deg_x10,follow_bbox_height,follow_distance_cm,follow_path_samples,follow_frame_age_ms,follow_target_speed_mps,follow_target_angle_deg,"
    "steer_raw_read,steer_raw_last_valid,steer_raw_jump_counts,steer_raw_last_reject_reason,steer_raw_last_reject_streak,steer_raw_reject_count,steer_raw_last_reject_ms,steer_raw_last_reject_raw,steer_raw_last_reject_ref_raw,steer_raw_last_reject_jump_counts\r\n";

static const char *DATA_LOGGER_DRIVE_HEADER =
    "time_ms,marker,task_id,task_state,profile,target_src,"
    "remote_connected,remote_armed,remote_state,remote_guard,remote_motor_run,remote_ch5_pressed,remote_raw_steer,remote_raw_throttle,remote_raw_ch5,remote_target_speed_mps,remote_target_steer_deg,remote_last_frame_age_ms,remote_valid_frames,remote_bad_frames,"
    "target_speed_mps,drive_target_ecod,pedal_raw0,pedal_raw1,pedal_filt0,pedal_filt1,pedal_perm0,pedal_perm1,pedal_rel0,pedal_rel1,pedal_fault0,pedal_fault1,pedal_target_mps,ecod1_speed,ecod2_speed,ecod1_speed_filt,ecod2_speed_filt,"
    "pid_target_r,pid_feedback_r,pid_error_r,pid_ff_r,pid_p_r,pid_i_r,pid_d_r,pid_boost_r,pid_dir_guard_r,pid_out_r,"
    "pid_target_l,pid_feedback_l,pid_error_l,pid_ff_l,pid_p_l,pid_i_l,pid_d_l,pid_boost_l,pid_dir_guard_l,pid_out_l,"
    "duty_r,duty_l,duty_m,abs_raw,steer_center_raw,steer_delta_counts,steer_speed_counts_s,steer_target_deg,steer_feedback_deg,steer_valid,steer_sensor_fault,steer_stall_fault,"
    "steer_scan_phase,steer_scan_success,steer_scan_left_raw,steer_scan_right_raw,steer_scan_center_raw,steer_scan_span_counts,pid_out_steer,"
    "ctrl_use_pid,ctrl_direct_pwm,ctrl_steer_closed_loop,ctrl_brake_to_stop,ctrl_target_angle_deg,"
    "key_num,key_event_num,key_event_count,key_event_age_ms,ctrl_stop_reason,ctrl_safety_pause_active,ctrl_main_loop_age_ms,ctrl_safety_pause_until_ms,logger_dropped,"
    "ctrl_cmd_target_speed_mps,ctrl_cmd_age_ms,steer_feedback_age_ms,"
    "ctrl_pid_reset_reason,ctrl_pid_reset_age_ms,ctrl_pid_low_reason,ctrl_pid_low_age_ms,"
    "ctrl_pid_low_target_r,ctrl_pid_low_feedback_r,ctrl_pid_low_output_r,ctrl_pid_low_target_l,ctrl_pid_low_feedback_l,ctrl_pid_low_output_l,remote_ch1_raw,remote_ch2_raw,remote_ch3_raw,remote_ch4_raw,remote_ch5_raw,remote_ch6_raw,remote_ch6_pressed\r\n";

static const char *DATA_LOGGER_GPS_TEST_HEADER =
    "time_ms,marker,event,point_id,task_id,task_state,profile,gps_valid,gps_timestamp_ms,sat,gnss_state,"
    "lat,lon,gps_x_m,gps_y_m,gps_speed_mps,gps_course_deg,"
    "origin_x_m,origin_y_m,drift_m,"
    "nav_valid,gps_fusion_valid,ins_valid,ins_x_m,ins_y_m,ins_yaw_deg,ins_speed_mps,gps_ins_error_m,"
    "corr_enabled,corr_gain,corr_snap_m,logger_dropped\r\n";

static const char *DATA_LOGGER_IMU_TEST_HEADER =
    "time_ms,marker,task_id,task_state,profile,fusion_algo,cmp_enabled,cmp_valid,shadow_algo,imu_init_ret,pit_count,update_count,update_age_ms,read_fail_count,"
    "rel_pitch,rel_roll,rel_yaw,abs_pitch,abs_roll,abs_yaw,imu_gyro_z,"
    "rx,flt,dt_init,att,ref,mag,req,bias_ready,bias_count,bias_target,sample_dt,cfg_dt,"
    "gyro_x,gyro_y,gyro_z,gyro_norm,bias_x,bias_y,bias_z,"
    "acc_x,acc_y,acc_z,acc_norm,acc_conf,mag_x,mag_y,mag_z,mag_norm,mag_conf,raw_mag_x,raw_mag_y,raw_mag_z,mag_norm_ref,"
    "cmp_main_pitch,cmp_main_roll,cmp_main_yaw,cmp_shadow_pitch,cmp_shadow_roll,cmp_shadow_yaw,"
    "shadow_abs_pitch,shadow_abs_roll,shadow_abs_yaw,cmp_delta_pitch,cmp_delta_roll,cmp_delta_yaw,logger_dropped\r\n";

static const char *DATA_LOGGER_TASK2_INS_HEADER =
    "time_ms,marker,task_id,task_state,profile,target_src,"
    "ecod1_count,ecod2_count,ecod1_speed,ecod2_speed,ecod1_speed_filt,ecod2_speed_filt,"
    "nav_valid,gps_valid,ins_valid,ins_timestamp_ms,ins_dt_ms,"
    "enc_delta_r,enc_delta_l,enc_delta_avg,delta_distance_m,count_to_meter,"
    "imu_yaw,ins_yaw_deg,yaw_used_deg,gyro_z,ins_yaw_rate_dps,"
    "imu_abs_yaw,imu_pit_count,imu_update_count,imu_update_age_ms,imu_read_fail_count,"
    "imu_rx,imu_bias_ready,imu_bias_count,imu_sample_dt_s,imu_dbg_gyro_z_dps,imu_yaw_freeze_ms,imu_yaw_freeze_suspect,"
    "delta_x_m,delta_y_m,ins_x_m,ins_y_m,distance_m,"
    "ins_speed_mps,encoder_speed_mps,forward_accel_mps2,imu_acc_x_g,imu_acc_y_g,imu_acc_z_g,"
    "path_valid,path_active,path_finished,path_mode,path_sample_count,path_replay_index,path_sample_interval_m,path_progress_m,"
    "path_ref_x_m,path_ref_y_m,path_ref_yaw_deg,path_ref_kappa_1pm,path_ref_kappa_yaw_1pm,path_ref_speed_mps,path_yaw_error_deg,path_lateral_error_m,path_sd,path_fsave,path_fload,"
    "task2_dbg_valid,task2_dbg_motion,task2_dbg_route_cmd,task2_dbg_route_step_index,task2_dbg_route_step_count,task2_dbg_path_index,task2_dbg_path_count,task2_dbg_final,task2_dbg_return,"
    "task2_dbg_target_x_m,task2_dbg_target_y_m,task2_dbg_preview_x_m,task2_dbg_preview_y_m,task2_dbg_desired_yaw_deg,task2_dbg_yaw_error_deg,task2_dbg_lateral_error_m,task2_dbg_projection_m,task2_dbg_dist_target_m,task2_dbg_dist_park_m,task2_dbg_target_angle_deg,task2_dbg_target_speed_mps,"
    "steer_target_deg,steer_feedback_deg,steer_valid,steer_sensor_fault,steer_stall_fault,target_speed_mps,pid_target_r,pid_feedback_r,pid_target_l,pid_feedback_l,pid_dir_guard_r,pid_dir_guard_l,duty_r,duty_l,duty_m,"
    "key_num,key_event_num,key_event_count,key_event_age_ms,ctrl_stop_reason,ctrl_safety_pause_active,ctrl_main_loop_age_ms,ctrl_safety_pause_until_ms,logger_dropped,"
    "pid_error_r,pid_error_l,ctrl_use_pid,ctrl_direct_pwm,ctrl_steer_closed_loop,ctrl_brake_to_stop,ctrl_cmd_target_speed_mps,ctrl_cmd_target_angle_deg,ctrl_cmd_age_ms,steer_feedback_age_ms,"
    "ctrl_pid_reset_reason,ctrl_pid_reset_age_ms,ctrl_pid_low_reason,ctrl_pid_low_age_ms,"
    "ctrl_pid_low_target_r,ctrl_pid_low_feedback_r,ctrl_pid_low_output_r,ctrl_pid_low_target_l,ctrl_pid_low_feedback_l,ctrl_pid_low_output_l\r\n";

static const char *g_logger_task_dir[DATA_LOGGER_TASK_COUNT] =
{
    "0:/LOG/TASK0_DRIVE",
    "0:/TASK1",
    "0:/LOG/TASK2_PATH_REPLAY",
    "0:/LOG/TASK3_NAV",
    "0:/LOG/TASK4_DEBUG"
};

static const char *g_logger_target_src_dir[DATA_LOGGER_TARGET_SRC_COUNT] =
{
    "TASK",
    "PEDAL",
    "REMOTE",
    "PWM_TEST",
    "SCAN"
};

static const char *DATA_LOGGER_TASK1_GPS_TEST_DIR = "0:/TASK1/GPS_TEST";
static const char *DATA_LOGGER_TASK1_INS_NAV_DIR = "0:/TASK1/INS_NAV";
static const char *DATA_LOGGER_TASK1_IMU_TEST_DIR = "0:/TASK1/IMU_TEST";

static volatile data_logger_sample_t g_logger_queue[DATA_LOGGER_QUEUE_SIZE];
static volatile uint16 g_logger_queue_head;
static volatile uint16 g_logger_queue_tail;
static volatile uint32 g_logger_dropped_count;
static volatile uint32 g_logger_last_sample_ms;
static volatile uint8 g_logger_state;
static volatile uint8 g_logger_last_error;
static volatile uint8 g_logger_task_id;
static volatile uint8 g_logger_profile;
static volatile uint8 g_logger_target_src;
static volatile uint16 g_logger_marker;
static volatile uint8 g_logger_restart_pending;
static volatile uint8 g_logger_restart_task_id;
static volatile uint8 g_logger_restart_profile;
static volatile uint8 g_logger_restart_target_src;
static char g_logger_filename[64];
static uint8 g_logger_gps_test_origin_valid;
static float g_logger_gps_test_origin_x_m;
static float g_logger_gps_test_origin_y_m;
static uint8 g_logger_task2_freeze_valid;
static uint32 g_logger_task2_freeze_start_ms;
static float g_logger_task2_freeze_last_yaw_deg;
static volatile uint8 g_camera_snapshot_state;
static volatile uint8 g_camera_snapshot_size;
static volatile uint8 g_camera_snapshot_last_error;
static volatile uint32 g_camera_snapshot_bytes;
static volatile uint16 g_camera_snapshot_width;
static volatile uint16 g_camera_snapshot_height;
static volatile uint8 g_camera_record_state;
static volatile uint8 g_camera_record_size;
static volatile uint8 g_camera_record_target_fps;
static volatile uint8 g_camera_record_frame_busy;
static volatile uint8 g_camera_record_last_error;
static volatile uint32 g_camera_record_frame_count;
static volatile uint32 g_camera_record_dropped_count;
static volatile uint32 g_camera_record_bytes;
static volatile data_logger_camera_frame_meta_t g_camera_record_meta;
static volatile uint8 g_camera_test_state;
static volatile uint8 g_camera_test_mode;
static volatile uint8 g_camera_test_allow_logger;
static volatile uint8 g_camera_test_frame_busy;
static volatile uint8 g_camera_test_last_error;
static volatile uint8 g_camera_test_last_reason;
static volatile uint32 g_camera_test_event_count;
static volatile uint32 g_camera_test_dropped_count;
static volatile uint32 g_camera_test_bytes;
static volatile data_logger_camera_frame_meta_t g_camera_test_meta;

#pragma section all "cpu1_dsram"
static FATFS g_logger_fatfs;
static FIL g_logger_file;
static uint8 g_logger_mounted;
static uint8 g_logger_file_open;
static uint32 g_logger_last_sync_ms;
static char g_logger_buffer[DATA_LOGGER_BUFFER_SIZE];
static char g_logger_line[DATA_LOGGER_LINE_SIZE];
static uint32 g_logger_buffer_used;
static volatile uint16 g_camera_snapshot_image[SCC8660_H][SCC8660_W];
static char g_camera_snapshot_filename[64];
static FIL g_camera_record_csv_file;
static uint8 g_camera_record_csv_open;
static uint32 g_camera_record_last_sync_ms;
static char g_camera_record_directory[64];
static char g_camera_record_frame_filename[96];
static char g_camera_record_line[DATA_LOGGER_CAMERA_RECORD_LINE_SIZE];
static FIL g_camera_test_csv_file;
static uint8 g_camera_test_csv_open;
static char g_camera_test_directory[80];
static char g_camera_test_image_directory[96];
static char g_camera_test_frame_filename[112];
static char g_camera_test_line[DATA_LOGGER_CAMERA_TEST_LINE_SIZE];
#pragma section all restore

static void data_logger_memory_sync(void)
{
    __dsync();
}

static uint16 data_logger_next_index(uint16 index)
{
    index++;
    if (index >= DATA_LOGGER_QUEUE_SIZE)
    {
        index = 0U;
    }
    return index;
}

static float data_logger_abs_f32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float data_logger_wrap180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void data_logger_update_task2_freeze_diag(data_logger_sample_t *sample,
                                                 uint32 now_ms)
{
    uint8 active;
    float yaw_delta_deg;

    if (sample == NULL)
    {
        return;
    }

    sample->imu_yaw_freeze_ms = 0U;
    sample->imu_yaw_freeze_suspect = 0U;

    active = ((sample->profile == (uint8)DATA_LOGGER_PROFILE_TASK2_INS_DEBUG) &&
              (data_logger_abs_f32(sample->target_speed_mps) >= DATA_LOGGER_TASK2_FREEZE_SPEED_MPS) &&
              (data_logger_abs_f32(sample->steer_target_deg) >= DATA_LOGGER_TASK2_FREEZE_STEER_DEG)) ? 1U : 0U;
    if (active == 0U)
    {
        g_logger_task2_freeze_valid = 0U;
        g_logger_task2_freeze_start_ms = now_ms;
        g_logger_task2_freeze_last_yaw_deg = sample->imu_yaw;
        return;
    }

    if (g_logger_task2_freeze_valid == 0U)
    {
        g_logger_task2_freeze_valid = 1U;
        g_logger_task2_freeze_start_ms = now_ms;
        g_logger_task2_freeze_last_yaw_deg = sample->imu_yaw;
        return;
    }

    yaw_delta_deg = data_logger_abs_f32(data_logger_wrap180(sample->imu_yaw -
                                                            g_logger_task2_freeze_last_yaw_deg));
    if ((yaw_delta_deg <= DATA_LOGGER_TASK2_FREEZE_YAW_EPS_DEG) &&
        (data_logger_abs_f32(sample->gyro_z) <= DATA_LOGGER_TASK2_FREEZE_GYRO_EPS_DPS))
    {
        sample->imu_yaw_freeze_ms = now_ms - g_logger_task2_freeze_start_ms;
        if (sample->imu_yaw_freeze_ms >= DATA_LOGGER_TASK2_FREEZE_SUSPECT_MS)
        {
            sample->imu_yaw_freeze_suspect = 1U;
        }
    }
    else
    {
        g_logger_task2_freeze_start_ms = now_ms;
        sample->imu_yaw_freeze_ms = 0U;
    }

    g_logger_task2_freeze_last_yaw_deg = sample->imu_yaw;
}

static void data_logger_queue_reset(void)
{
    g_logger_queue_head = 0U;
    g_logger_queue_tail = 0U;
    g_logger_dropped_count = 0U;
    g_logger_last_sample_ms = 0U;
    g_logger_gps_test_origin_valid = 0U;
    g_logger_gps_test_origin_x_m = 0.0f;
    g_logger_gps_test_origin_y_m = 0.0f;
    g_logger_task2_freeze_valid = 0U;
    g_logger_task2_freeze_start_ms = 0U;
    g_logger_task2_freeze_last_yaw_deg = 0.0f;
    data_logger_memory_sync();
}

static uint32 data_logger_extend_time_ms(uint32 raw_ms)
{
    return raw_ms;
}

static uint8 data_logger_queue_push(const data_logger_sample_t *sample)
{
    uint16 head;
    uint16 next;

    if (sample == NULL)
    {
        return 1U;
    }

    head = g_logger_queue_head;
    next = data_logger_next_index(head);
    if (next == g_logger_queue_tail)
    {
        g_logger_dropped_count++;
        return 2U;
    }

    g_logger_queue[head] = *sample;
    data_logger_memory_sync();
    g_logger_queue_head = next;
    return 0U;
}

static uint8 data_logger_queue_pop(data_logger_sample_t *sample)
{
    uint16 tail;

    if (sample == NULL)
    {
        return 1U;
    }

    tail = g_logger_queue_tail;
    if (tail == g_logger_queue_head)
    {
        return 1U;
    }

    *sample = g_logger_queue[tail];
    data_logger_memory_sync();
    g_logger_queue_tail = data_logger_next_index(tail);
    return 0U;
}

static uint8 data_logger_flush(void)
{
    UINT written = 0U;
    FRESULT fr;

    if ((g_logger_file_open == 0U) || (g_logger_buffer_used == 0U))
    {
        return 0U;
    }

    fr = f_write(&g_logger_file, g_logger_buffer, (UINT)g_logger_buffer_used, &written);
    if ((fr != FR_OK) || (written != (UINT)g_logger_buffer_used))
    {
        printf("[LOGGER] write failed, fr=%d written=%u/%u\r\n",
               (int)fr,
               (unsigned)written,
               (unsigned)g_logger_buffer_used);
        return 1U;
    }

    g_logger_buffer_used = 0U;
    return 0U;
}

static uint8 data_logger_append(const char *text, uint32 len)
{
    if ((text == NULL) || (len == 0U))
    {
        return 0U;
    }

    if (len > DATA_LOGGER_BUFFER_SIZE)
    {
        return 1U;
    }

    if ((g_logger_buffer_used + len) > DATA_LOGGER_BUFFER_SIZE)
    {
        if (data_logger_flush() != 0U)
        {
            return 2U;
        }
    }

    memcpy(&g_logger_buffer[g_logger_buffer_used], text, len);
    g_logger_buffer_used += len;
    return 0U;
}

static const char *data_logger_get_task_dir(uint8 task_id)
{
    if (task_id >= DATA_LOGGER_TASK_COUNT)
    {
        task_id = 0U;
    }
    return g_logger_task_dir[task_id];
}

static const char *data_logger_get_target_src_dir(uint8 target_src)
{
    if (target_src >= DATA_LOGGER_TARGET_SRC_COUNT)
    {
        target_src = TARGET_SRC_TASK;
    }
    return g_logger_target_src_dir[target_src];
}

static void data_logger_get_active_dir(char *dir)
{
    const char *task_dir = data_logger_get_task_dir(g_logger_task_id);

    if (dir == NULL)
    {
        return;
    }

    if ((g_logger_task_id == 1U) &&
        (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_GPS_TEST))
    {
        sprintf(dir, "%s", DATA_LOGGER_TASK1_GPS_TEST_DIR);
    }
    else if ((g_logger_task_id == 1U) &&
             (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_FULL_NAV))
    {
        sprintf(dir, "%s", DATA_LOGGER_TASK1_INS_NAV_DIR);
    }
    else if ((g_logger_task_id == 1U) &&
             (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_IMU_TEST))
    {
        sprintf(dir, "%s", DATA_LOGGER_TASK1_IMU_TEST_DIR);
    }
    else if (g_logger_task_id == 0U)
    {
        sprintf(dir, "%s/%s", task_dir, data_logger_get_target_src_dir(g_logger_target_src));
    }
    else
    {
        sprintf(dir, "%s", task_dir);
    }
}

static uint8 data_logger_make_filename(void)
{
    uint32 i;
    char dir[64];

    data_logger_get_active_dir(dir);

    for (i = 0U; i < DATA_LOGGER_MAX_FILES; i++)
    {
        FILINFO info;
        FRESULT fr;

        sprintf(g_logger_filename, "%s/LOG%03u.CSV", dir, (unsigned)i);
        fr = f_stat(g_logger_filename, &info);
        if (fr == FR_NO_FILE)
        {
            data_logger_memory_sync();
            return 0U;
        }
        if (fr != FR_OK)
        {
            return 1U;
        }
    }

    return 2U;
}

static uint8 data_logger_backend_init(void)
{
    FRESULT fr;
    uint8 ret;
    uint8 i;
    char dir[64];

    if (g_logger_mounted != 0U)
    {
        return 0U;
    }

    ret = sd_simple_init();
    if (ret != 0U)
    {
        printf("[LOGGER] sd init failed on CPU1, ret=%u\r\n", ret);
        return 1U;
    }

    fr = f_mount(&g_logger_fatfs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("[LOGGER] f_mount failed on CPU1, fr=%d\r\n", (int)fr);
        return 2U;
    }

    fr = f_mkdir("0:/LOG");
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[LOGGER] f_mkdir failed, fr=%d\r\n", (int)fr);
        return 3U;
    }

    for (i = 0U; i < DATA_LOGGER_TASK_COUNT; i++)
    {
        fr = f_mkdir(g_logger_task_dir[i]);
        if ((fr != FR_OK) && (fr != FR_EXIST))
        {
            printf("[LOGGER] f_mkdir failed, fr=%d dir=%s\r\n",
                   (int)fr,
                   g_logger_task_dir[i]);
            return 4U;
        }
    }

    for (i = 0U; i < DATA_LOGGER_TARGET_SRC_COUNT; i++)
    {
        sprintf(dir, "%s/%s", g_logger_task_dir[0], g_logger_target_src_dir[i]);
        fr = f_mkdir(dir);
        if ((fr != FR_OK) && (fr != FR_EXIST))
        {
            printf("[LOGGER] f_mkdir failed, fr=%d dir=%s\r\n",
                   (int)fr,
                   dir);
            return 4U;
        }
    }

    fr = f_mkdir(DATA_LOGGER_TASK1_GPS_TEST_DIR);
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[LOGGER] f_mkdir failed, fr=%d dir=%s\r\n",
               (int)fr,
               DATA_LOGGER_TASK1_GPS_TEST_DIR);
        return 4U;
    }

    fr = f_mkdir(DATA_LOGGER_TASK1_INS_NAV_DIR);
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[LOGGER] f_mkdir failed, fr=%d dir=%s\r\n",
               (int)fr,
               DATA_LOGGER_TASK1_INS_NAV_DIR);
        return 4U;
    }

    fr = f_mkdir(DATA_LOGGER_TASK1_IMU_TEST_DIR);
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[LOGGER] f_mkdir failed, fr=%d dir=%s\r\n",
               (int)fr,
               DATA_LOGGER_TASK1_IMU_TEST_DIR);
        return 4U;
    }

    g_logger_mounted = 1U;
    return 0U;
}

static uint8 data_logger_camera_snapshot_scale(uint8 size)
{
    if (size == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_HALF)
    {
        return 2U;
    }
    if (size == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_QUARTER)
    {
        return 4U;
    }
    return 1U;
}

static uint16 data_logger_camera_snapshot_display_rgb565(uint16 pixel)
{
    return (uint16)((pixel << 8) | (pixel >> 8));
}

static void data_logger_camera_stage_pixel(uint16 x, uint16 y, uint16 display_color)
{
    volatile uint16 *snapshot = (volatile uint16 *)g_camera_snapshot_image;

    if ((x < g_camera_snapshot_width) && (y < g_camera_snapshot_height))
    {
        snapshot[(uint32)y * g_camera_snapshot_width + x] =
            data_logger_camera_snapshot_display_rgb565(display_color);
    }
}

static void data_logger_camera_stage_overlay(
    const data_logger_camera_frame_meta_t *meta,
    uint8 scale)
{
    uint16 x0;
    uint16 y0;
    uint16 x1;
    uint16 y1;
    uint16 cx;
    uint16 cy;
    uint16 x;
    uint16 y;
    uint16 cross_radius;

    if ((meta == NULL) || (meta->target_valid == 0U) ||
        (meta->bbox_width == 0U) || (meta->bbox_height == 0U))
    {
        return;
    }

    x0 = (uint16)(meta->bbox_x / scale);
    y0 = (uint16)(meta->bbox_y / scale);
    x1 = (uint16)((meta->bbox_x + meta->bbox_width - 1U) / scale);
    y1 = (uint16)((meta->bbox_y + meta->bbox_height - 1U) / scale);
    if (x1 >= g_camera_snapshot_width) x1 = (uint16)(g_camera_snapshot_width - 1U);
    if (y1 >= g_camera_snapshot_height) y1 = (uint16)(g_camera_snapshot_height - 1U);
    if ((x0 > x1) || (y0 > y1))
    {
        return;
    }

    for (x = x0; x <= x1; x++)
    {
        data_logger_camera_stage_pixel(x, y0, RGB565_YELLOW);
        data_logger_camera_stage_pixel(x, y1, RGB565_YELLOW);
    }
    for (y = y0; y <= y1; y++)
    {
        data_logger_camera_stage_pixel(x0, y, RGB565_YELLOW);
        data_logger_camera_stage_pixel(x1, y, RGB565_YELLOW);
    }

    cx = (uint16)(meta->center_x / scale);
    cy = (uint16)(meta->center_y / scale);
    cross_radius = (uint16)(3U / scale);
    if (cross_radius == 0U) cross_radius = 1U;
    for (x = (cx > cross_radius) ? (uint16)(cx - cross_radius) : 0U;
         (x <= (uint16)(cx + cross_radius)) && (x < g_camera_snapshot_width);
         x++)
    {
        data_logger_camera_stage_pixel(x, cy, RGB565_RED);
    }
    for (y = (cy > cross_radius) ? (uint16)(cy - cross_radius) : 0U;
         (y <= (uint16)(cy + cross_radius)) && (y < g_camera_snapshot_height);
         y++)
    {
        data_logger_camera_stage_pixel(cx, y, RGB565_RED);
    }
}

static void data_logger_camera_stage_frame(
    data_logger_camera_snapshot_size_t size,
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta)
{
    uint8 scale = data_logger_camera_snapshot_scale((uint8)size);
    uint16 width = (uint16)(SCC8660_W / scale);
    uint16 height = (uint16)(SCC8660_H / scale);
    uint16 y;
    uint16 x;
    volatile uint16 *snapshot = (volatile uint16 *)g_camera_snapshot_image;

    for (y = 0U; y < height; y++)
    {
        for (x = 0U; x < width; x++)
        {
            snapshot[(uint32)y * width + x] =
                image[(uint32)(y * scale) * SCC8660_W + (x * scale)];
        }
    }

    g_camera_snapshot_width = width;
    g_camera_snapshot_height = height;
    data_logger_camera_stage_overlay(meta, scale);
}

static void data_logger_camera_snapshot_put_u16(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
}

static void data_logger_camera_snapshot_put_u32(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint8 data_logger_camera_snapshot_make_filename(void)
{
    uint32 i;

    for (i = 0U; i < DATA_LOGGER_CAMERA_SNAPSHOT_MAX_FILES; i++)
    {
        FILINFO info;
        FRESULT fr;

        sprintf(g_camera_snapshot_filename, "0:/CAM/T3C%03u.BMP", (unsigned)i);
        fr = f_stat(g_camera_snapshot_filename, &info);
        if (fr == FR_NO_FILE)
        {
            return 0U;
        }
        if (fr != FR_OK)
        {
            return 1U;
        }
    }

    return 2U;
}

static uint8 data_logger_camera_write_staged_bmp(const char *filename)
{
    FIL file;
    FRESULT fr;
    UINT written;
    uint8 header[DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE];
    uint8 file_open = 0U;
    uint16 width = g_camera_snapshot_width;
    uint16 height = g_camera_snapshot_height;
    uint16 row_bytes = (uint16)(width * 3U);
    uint32 image_bytes = (uint32)row_bytes * height;
    uint32 block_used = 0U;
    uint16 y;
    uint16 x;
    uint16 pixel;
    uint8 *write_buffer = (uint8 *)g_logger_buffer;
    uint8 ret = 0U;

    if ((filename == NULL) || (width == 0U) || (height == 0U) ||
        (row_bytes > DATA_LOGGER_BUFFER_SIZE))
    {
        return 9U;
    }

    /* The BMP encoder reuses the logger's 4 KB staging buffer. Preserve any
       queued CSV text before filling that buffer with pixel bytes. */
    if (g_logger_buffer_used != 0U)
    {
        if ((g_logger_file_open == 0U) || (data_logger_flush() != 0U))
        {
            return 23U;
        }
    }

    fr = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK)
    {
        return 24U;
    }
    file_open = 1U;

    memset(header, 0, sizeof(header));
    header[0] = 'B';
    header[1] = 'M';
    data_logger_camera_snapshot_put_u32(&header[2],
                                        DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE + image_bytes);
    data_logger_camera_snapshot_put_u32(&header[10],
                                        DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE);
    data_logger_camera_snapshot_put_u32(&header[14], 40U);
    data_logger_camera_snapshot_put_u32(&header[18], width);
    data_logger_camera_snapshot_put_u32(&header[22], height);
    data_logger_camera_snapshot_put_u16(&header[26], 1U);
    data_logger_camera_snapshot_put_u16(&header[28], 24U);
    data_logger_camera_snapshot_put_u32(&header[34], image_bytes);

    fr = f_write(&file, header, sizeof(header), &written);
    if ((fr != FR_OK) || (written != sizeof(header)))
    {
        ret = 25U;
    }

    for (y = height; (ret == 0U) && (y > 0U); y--)
    {
        if (guimai_voice_is_recording() != 0U)
        {
            ret = 4U;
            break;
        }

        if ((block_used + row_bytes) > DATA_LOGGER_BUFFER_SIZE)
        {
            fr = f_write(&file, write_buffer, (UINT)block_used, &written);
            if ((fr != FR_OK) || (written != (UINT)block_used))
            {
                ret = 26U;
                break;
            }
            block_used = 0U;
        }

        for (x = 0U; x < width; x++)
        {
            pixel = data_logger_camera_snapshot_display_rgb565(
                ((const volatile uint16 *)g_camera_snapshot_image)[((uint32)(y - 1U) * width) + x]);
            write_buffer[block_used + x * 3U + 0U] = (uint8)((pixel & 0x1FU) << 3);
            write_buffer[block_used + x * 3U + 1U] = (uint8)(((pixel >> 5) & 0x3FU) << 2);
            write_buffer[block_used + x * 3U + 2U] = (uint8)(((pixel >> 11) & 0x1FU) << 3);
        }
        block_used += row_bytes;
    }

    if ((ret == 0U) && (block_used > 0U))
    {
        fr = f_write(&file, write_buffer, (UINT)block_used, &written);
        if ((fr != FR_OK) || (written != (UINT)block_used))
        {
            ret = 26U;
        }
    }
    if ((file_open != 0U) && (f_close(&file) != FR_OK) && (ret == 0U))
    {
        ret = 28U;
    }
    if (ret != 0U)
    {
        (void)f_unlink(filename);
    }
    return ret;
}

static uint8 data_logger_camera_snapshot_write(void)
{
    FRESULT fr;
    uint8 ret;

    ret = data_logger_backend_init();
    if (ret != 0U)
    {
        return (uint8)(10U + ret);
    }

    fr = f_mkdir("0:/CAM");
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        return 20U;
    }

    ret = data_logger_camera_snapshot_make_filename();
    if (ret != 0U)
    {
        return (uint8)(20U + ret);
    }

    ret = data_logger_camera_write_staged_bmp(g_camera_snapshot_filename);
    if (ret == 4U)
    {
        g_camera_snapshot_filename[0] = '\0';
    }
    if (ret == 0U)
    {
        g_camera_snapshot_bytes = DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE +
                                  ((uint32)g_camera_snapshot_width *
                                   g_camera_snapshot_height * 3U);
    }
    return ret;
}

uint8 data_logger_camera_snapshot_request(data_logger_camera_snapshot_size_t size,
                                          const uint16 *image)
{
    return data_logger_camera_snapshot_request_with_meta(size, image, NULL);
}

uint8 data_logger_camera_snapshot_request_with_meta(
    data_logger_camera_snapshot_size_t size,
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta)
{

    if ((image == NULL) || (size >= DATA_LOGGER_CAMERA_SNAPSHOT_SIZE_COUNT))
    {
        return 1U;
    }
    if ((g_logger_state != DATA_LOGGER_STATE_IDLE) &&
        (g_logger_state != DATA_LOGGER_STATE_FAILED))
    {
        return 2U;
    }
    if (data_logger_camera_snapshot_is_busy() != 0U)
    {
        return 3U;
    }
    if (data_logger_camera_record_is_active() != 0U)
    {
        return 5U;
    }
    if (data_logger_camera_test_is_active() != 0U)
    {
        return 6U;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        return 4U;
    }

    data_logger_camera_stage_frame(size, image, meta);
    g_camera_snapshot_size = (uint8)size;
    g_camera_snapshot_last_error = 0U;
    g_camera_snapshot_bytes =
        DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE +
        ((uint32)g_camera_snapshot_width * g_camera_snapshot_height * 3U);
    g_camera_snapshot_filename[0] = '\0';
    data_logger_memory_sync();
    g_camera_snapshot_state = (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_PENDING;
    data_logger_memory_sync();
    return 0U;
}

void data_logger_camera_snapshot_worker_task(uint32 now_ms)
{
    uint8 ret;

    (void)now_ms;
    if (g_camera_snapshot_state != (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_PENDING)
    {
        return;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        g_camera_snapshot_last_error = 4U;
        g_camera_snapshot_filename[0] = '\0';
        data_logger_memory_sync();
        g_camera_snapshot_state = (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_FAILED;
        data_logger_memory_sync();
        printf("[CAMERA] snapshot canceled: voice recording.\r\n");
        return;
    }

    g_camera_snapshot_state = (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_WRITING;
    data_logger_memory_sync();
    ret = data_logger_camera_snapshot_write();
    g_camera_snapshot_last_error = ret;
    g_camera_snapshot_state = (ret == 0U) ?
                              (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_DONE :
                              (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_FAILED;
    data_logger_memory_sync();
    printf("[CAMERA] snapshot state=%u ret=%u file=%s bytes=%lu\r\n",
           (unsigned)g_camera_snapshot_state,
           (unsigned)ret,
           g_camera_snapshot_filename,
           (unsigned long)g_camera_snapshot_bytes);
}

uint8 data_logger_camera_snapshot_is_busy(void)
{
    return ((g_camera_snapshot_state == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_PENDING) ||
            (g_camera_snapshot_state == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_WRITING)) ? 1U : 0U;
}

uint8 data_logger_camera_snapshot_get_state(void)
{
    return g_camera_snapshot_state;
}

uint8 data_logger_camera_snapshot_get_size(void)
{
    return g_camera_snapshot_size;
}

uint8 data_logger_camera_snapshot_get_last_error(void)
{
    return g_camera_snapshot_last_error;
}

uint32 data_logger_camera_snapshot_get_bytes(void)
{
    return g_camera_snapshot_bytes;
}

const char *data_logger_camera_snapshot_get_filename(void)
{
    return g_camera_snapshot_filename;
}

static void data_logger_camera_record_close(void)
{
    if (g_camera_record_csv_open != 0U)
    {
        (void)f_sync(&g_camera_record_csv_file);
        (void)f_close(&g_camera_record_csv_file);
    }
    g_camera_record_csv_open = 0U;
}

static uint8 data_logger_camera_record_start_files(uint32 now_ms)
{
    static const char header[] =
        "frame_id,time_ms,file,target_valid,confidence_pct,fill_pct,center_x,center_y,offset_x,bearing_deg_x10,"
        "bbox_x,bbox_y,bbox_width,bbox_height,area_px,distance_valid,distance_cm,process_us,camera_fps,vision_load_pct,"
        "save_width,save_height,dropped_frames\r\n";
    char csv_filename[96];
    FRESULT fr;
    FILINFO info;
    UINT written;
    uint32 i;
    uint8 ret;

    ret = data_logger_backend_init();
    if (ret != 0U)
    {
        return (uint8)(10U + ret);
    }

    fr = f_mkdir("0:/CAM");
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        return 20U;
    }

    g_camera_record_directory[0] = '\0';
    for (i = 0U; i < DATA_LOGGER_CAMERA_RECORD_MAX_DIRS; i++)
    {
        sprintf(g_camera_record_directory, "0:/CAM/REC%03u", (unsigned)i);
        fr = f_stat(g_camera_record_directory, &info);
        if (fr == FR_NO_FILE)
        {
            break;
        }
        if (fr != FR_OK)
        {
            return 21U;
        }
    }
    if (i >= DATA_LOGGER_CAMERA_RECORD_MAX_DIRS)
    {
        return 22U;
    }

    fr = f_mkdir(g_camera_record_directory);
    if (fr != FR_OK)
    {
        return 23U;
    }
    sprintf(csv_filename, "%s/VISION.CSV", g_camera_record_directory);
    fr = f_open(&g_camera_record_csv_file, csv_filename, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK)
    {
        return 24U;
    }
    g_camera_record_csv_open = 1U;

    fr = f_write(&g_camera_record_csv_file, header, sizeof(header) - 1U, &written);
    if ((fr != FR_OK) || (written != (sizeof(header) - 1U)))
    {
        data_logger_camera_record_close();
        return 25U;
    }
    if (f_sync(&g_camera_record_csv_file) != FR_OK)
    {
        data_logger_camera_record_close();
        return 26U;
    }

    g_camera_record_bytes = written;
    g_camera_record_last_sync_ms = now_ms;
    return 0U;
}

static uint8 data_logger_camera_record_append_meta(uint32 frame_id)
{
    FRESULT fr;
    UINT written;
    int len;

    len = sprintf(g_camera_record_line,
                  "%lu,%lu,F%05lu.BMP,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u,%u,%lu\r\n",
                  (unsigned long)frame_id,
                  (unsigned long)g_camera_record_meta.time_ms,
                  (unsigned long)frame_id,
                  (unsigned)g_camera_record_meta.target_valid,
                  (unsigned)g_camera_record_meta.confidence_pct,
                  (unsigned)g_camera_record_meta.fill_pct,
                  (unsigned)g_camera_record_meta.center_x,
                  (unsigned)g_camera_record_meta.center_y,
                  (int)g_camera_record_meta.offset_x,
                  (int)g_camera_record_meta.bearing_deg_x10,
                  (unsigned)g_camera_record_meta.bbox_x,
                  (unsigned)g_camera_record_meta.bbox_y,
                  (unsigned)g_camera_record_meta.bbox_width,
                  (unsigned)g_camera_record_meta.bbox_height,
                  (unsigned long)g_camera_record_meta.area_px,
                  (unsigned)g_camera_record_meta.distance_valid,
                  (unsigned)g_camera_record_meta.distance_cm,
                  (unsigned)g_camera_record_meta.process_us,
                  (unsigned)g_camera_record_meta.camera_fps,
                  (unsigned)g_camera_record_meta.vision_load_pct,
                  (unsigned)g_camera_snapshot_width,
                  (unsigned)g_camera_snapshot_height,
                  (unsigned long)g_camera_record_dropped_count);
    if ((len <= 0) || (len >= (int)sizeof(g_camera_record_line)))
    {
        return 31U;
    }

    fr = f_write(&g_camera_record_csv_file, g_camera_record_line, (UINT)len, &written);
    if ((fr != FR_OK) || (written != (UINT)len))
    {
        return 32U;
    }
    g_camera_record_bytes += written;
    return 0U;
}

static uint8 data_logger_camera_record_write_frame(void)
{
    uint32 frame_id = g_camera_record_frame_count;
    uint32 bmp_bytes;
    uint8 ret;

    if (frame_id >= DATA_LOGGER_CAMERA_RECORD_MAX_FRAMES)
    {
        return 30U;
    }

    sprintf(g_camera_record_frame_filename,
            "%s/F%05lu.BMP",
            g_camera_record_directory,
            (unsigned long)frame_id);
    ret = data_logger_camera_write_staged_bmp(g_camera_record_frame_filename);
    if (ret != 0U)
    {
        return ret;
    }

    ret = data_logger_camera_record_append_meta(frame_id);
    if (ret != 0U)
    {
        (void)f_unlink(g_camera_record_frame_filename);
        return ret;
    }

    bmp_bytes = DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE +
                ((uint32)g_camera_snapshot_width * g_camera_snapshot_height * 3U);
    g_camera_record_bytes += bmp_bytes;
    g_camera_record_frame_count++;
    return 0U;
}

static void data_logger_camera_record_fail(uint8 error)
{
    data_logger_camera_record_close();
    g_camera_record_last_error = error;
    g_camera_record_frame_busy = 0U;
    data_logger_memory_sync();
    g_camera_record_state = (uint8)DATA_LOGGER_CAMERA_RECORD_FAILED;
    printf("[CAMERA] record failed err=%u dir=%s frame=%lu drop=%lu\r\n",
           (unsigned)error,
           g_camera_record_directory,
           (unsigned long)g_camera_record_frame_count,
           (unsigned long)g_camera_record_dropped_count);
}

uint8 data_logger_camera_record_start(data_logger_camera_snapshot_size_t size,
                                      uint8 target_fps)
{
    if ((size >= DATA_LOGGER_CAMERA_SNAPSHOT_SIZE_COUNT) || (target_fps == 0U))
    {
        return 1U;
    }
    if ((g_logger_state != DATA_LOGGER_STATE_IDLE) &&
        (g_logger_state != DATA_LOGGER_STATE_FAILED))
    {
        return 2U;
    }
    if ((data_logger_camera_snapshot_is_busy() != 0U) ||
        (data_logger_camera_record_is_active() != 0U) ||
        (data_logger_camera_test_is_active() != 0U))
    {
        return 3U;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        return 4U;
    }

    g_camera_record_size = (uint8)size;
    g_camera_record_target_fps = target_fps;
    g_camera_record_frame_busy = 0U;
    g_camera_record_last_error = 0U;
    g_camera_record_frame_count = 0U;
    g_camera_record_dropped_count = 0U;
    g_camera_record_bytes = 0U;
    g_camera_record_directory[0] = '\0';
    g_camera_record_frame_filename[0] = '\0';
    data_logger_memory_sync();
    g_camera_record_state = (uint8)DATA_LOGGER_CAMERA_RECORD_STARTING;
    return 0U;
}

void data_logger_camera_record_stop(void)
{
    if ((g_camera_record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_STARTING) ||
        (g_camera_record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_RUNNING))
    {
        data_logger_memory_sync();
        g_camera_record_state = (uint8)DATA_LOGGER_CAMERA_RECORD_STOPPING;
    }
}

uint8 data_logger_camera_record_frame_request(
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta)
{
    data_logger_camera_frame_meta_t empty_meta;

    if (image == NULL)
    {
        return 1U;
    }
    if (g_camera_record_state != (uint8)DATA_LOGGER_CAMERA_RECORD_RUNNING)
    {
        return 5U;
    }
    if (g_camera_record_frame_busy != 0U)
    {
        g_camera_record_dropped_count++;
        return 6U;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        data_logger_camera_record_stop();
        return 4U;
    }

    data_logger_camera_stage_frame(
        (data_logger_camera_snapshot_size_t)g_camera_record_size,
        image,
        meta);
    if (meta != NULL)
    {
        g_camera_record_meta = *meta;
    }
    else
    {
        memset(&empty_meta, 0, sizeof(empty_meta));
        g_camera_record_meta = empty_meta;
    }
    data_logger_memory_sync();
    g_camera_record_frame_busy = 1U;
    data_logger_memory_sync();
    return 0U;
}

void data_logger_camera_record_worker_task(uint32 now_ms)
{
    uint8 state = g_camera_record_state;
    uint8 ret;

    data_logger_camera_test_worker_task(now_ms);

    if (state == (uint8)DATA_LOGGER_CAMERA_RECORD_STARTING)
    {
        if (guimai_voice_is_recording() != 0U)
        {
            data_logger_camera_record_fail(4U);
            return;
        }
        ret = data_logger_camera_record_start_files(now_ms);
        if (ret != 0U)
        {
            data_logger_camera_record_fail(ret);
            return;
        }
        data_logger_memory_sync();
        g_camera_record_state = (uint8)DATA_LOGGER_CAMERA_RECORD_RUNNING;
        printf("[CAMERA] record start dir=%s size=%u fps=%u\r\n",
               g_camera_record_directory,
               (unsigned)g_camera_record_size,
               (unsigned)g_camera_record_target_fps);
        return;
    }

    if ((state != (uint8)DATA_LOGGER_CAMERA_RECORD_RUNNING) &&
        (state != (uint8)DATA_LOGGER_CAMERA_RECORD_STOPPING))
    {
        return;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        data_logger_camera_record_fail(4U);
        return;
    }

    if (g_camera_record_frame_busy != 0U)
    {
        ret = data_logger_camera_record_write_frame();
        if (ret != 0U)
        {
            data_logger_camera_record_fail(ret);
            return;
        }
        data_logger_memory_sync();
        g_camera_record_frame_busy = 0U;
        data_logger_memory_sync();
    }

    if ((g_camera_record_csv_open != 0U) &&
        ((now_ms - g_camera_record_last_sync_ms) >= DATA_LOGGER_SYNC_PERIOD_MS))
    {
        g_camera_record_last_sync_ms = now_ms;
        if (f_sync(&g_camera_record_csv_file) != FR_OK)
        {
            data_logger_camera_record_fail(33U);
            return;
        }
    }

    if ((g_camera_record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_STOPPING) &&
        (g_camera_record_frame_busy == 0U))
    {
        data_logger_camera_record_close();
        data_logger_memory_sync();
        g_camera_record_state = (uint8)DATA_LOGGER_CAMERA_RECORD_DONE;
        printf("[CAMERA] record stop dir=%s frames=%lu drop=%lu bytes=%lu\r\n",
               g_camera_record_directory,
               (unsigned long)g_camera_record_frame_count,
               (unsigned long)g_camera_record_dropped_count,
               (unsigned long)g_camera_record_bytes);
    }
}

uint8 data_logger_camera_record_is_active(void)
{
    return ((g_camera_record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_STARTING) ||
            (g_camera_record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_RUNNING) ||
            (g_camera_record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_STOPPING)) ? 1U : 0U;
}

uint8 data_logger_camera_record_is_frame_busy(void)
{
    return g_camera_record_frame_busy;
}

uint8 data_logger_camera_record_get_state(void)
{
    return g_camera_record_state;
}

uint8 data_logger_camera_record_get_last_error(void)
{
    return g_camera_record_last_error;
}

uint8 data_logger_camera_record_get_target_fps(void)
{
    return g_camera_record_target_fps;
}

uint32 data_logger_camera_record_get_frame_count(void)
{
    return g_camera_record_frame_count;
}

uint32 data_logger_camera_record_get_dropped_count(void)
{
    return g_camera_record_dropped_count;
}

uint32 data_logger_camera_record_get_bytes(void)
{
    return g_camera_record_bytes;
}

const char *data_logger_camera_record_get_directory(void)
{
    return g_camera_record_directory;
}

static const char *data_logger_camera_test_mode_name(uint8 mode)
{
    return (mode == DATA_LOGGER_CAMERA_TEST_MODE_MOTION) ? "MOTION" : "STATIC";
}

static void data_logger_camera_test_close(void)
{
    if (g_camera_test_csv_open != 0U)
    {
        (void)f_sync(&g_camera_test_csv_file);
        (void)f_close(&g_camera_test_csv_file);
    }
    g_camera_test_csv_open = 0U;
}

static uint8 data_logger_camera_test_start_files(uint32 now_ms)
{
    static const char header[] =
        "event_id,time_ms,reason,file,target_valid,confidence_pct,fill_pct,"
        "center_x,center_y,offset_x,bearing_deg_x10,bbox_x,bbox_y,bbox_width,"
        "bbox_height,area_px,distance_valid,distance_cm,process_us,camera_fps,"
        "vision_load_pct,dropped_events\r\n";
    char static_dir[80];
    char motion_dir[80];
    FILINFO info;
    FRESULT fr;
    UINT written;
    uint32 index;

    if (data_logger_backend_init() != 0U)
    {
        return 10U;
    }

    fr = f_mkdir(DATA_LOGGER_CAMERA_TEST_ROOT_DIR);
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        return 20U;
    }

    g_camera_test_directory[0] = '\0';
    for (index = 0U; index < DATA_LOGGER_CAMERA_TEST_MAX_DIRS; index++)
    {
        sprintf(static_dir, "%s/TEST%03lu_STATIC",
                DATA_LOGGER_CAMERA_TEST_ROOT_DIR,
                (unsigned long)index);
        sprintf(motion_dir, "%s/TEST%03lu_MOTION",
                DATA_LOGGER_CAMERA_TEST_ROOT_DIR,
                (unsigned long)index);
        fr = f_stat(static_dir, &info);
        if ((fr != FR_NO_FILE) && (fr != FR_OK))
        {
            return 21U;
        }
        if (fr == FR_OK)
        {
            continue;
        }
        fr = f_stat(motion_dir, &info);
        if ((fr != FR_NO_FILE) && (fr != FR_OK))
        {
            return 22U;
        }
        if (fr == FR_OK)
        {
            continue;
        }
        sprintf(g_camera_test_directory, "%s/TEST%03lu_%s",
                DATA_LOGGER_CAMERA_TEST_ROOT_DIR,
                (unsigned long)index,
                data_logger_camera_test_mode_name(g_camera_test_mode));
        break;
    }
    if (g_camera_test_directory[0] == '\0')
    {
        return 23U;
    }

    fr = f_mkdir(g_camera_test_directory);
    if (fr != FR_OK)
    {
        return 24U;
    }
    sprintf(g_camera_test_image_directory, "%s/IMG", g_camera_test_directory);
    fr = f_mkdir(g_camera_test_image_directory);
    if (fr != FR_OK)
    {
        return 25U;
    }

    sprintf(g_camera_test_frame_filename, "%s/EVENTS.CSV", g_camera_test_directory);
    fr = f_open(&g_camera_test_csv_file,
                g_camera_test_frame_filename,
                FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK)
    {
        return 26U;
    }
    g_camera_test_csv_open = 1U;
    fr = f_write(&g_camera_test_csv_file,
                 header,
                 sizeof(header) - 1U,
                 &written);
    if ((fr != FR_OK) || (written != (sizeof(header) - 1U)) ||
        (f_sync(&g_camera_test_csv_file) != FR_OK))
    {
        data_logger_camera_test_close();
        return 27U;
    }
    g_camera_test_bytes = written;
    (void)now_ms;
    return 0U;
}

static uint8 data_logger_camera_test_append_event(uint32 event_id,
                                                  const char *image_name)
{
    FRESULT fr;
    UINT written;
    int len;

    len = sprintf(g_camera_test_line,
                  "%lu,%lu,%u,%s,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%lu\r\n",
                  (unsigned long)event_id,
                  (unsigned long)g_camera_test_meta.time_ms,
                  (unsigned)g_camera_test_last_reason,
                  image_name,
                  (unsigned)g_camera_test_meta.target_valid,
                  (unsigned)g_camera_test_meta.confidence_pct,
                  (unsigned)g_camera_test_meta.fill_pct,
                  (unsigned)g_camera_test_meta.center_x,
                  (unsigned)g_camera_test_meta.center_y,
                  (int)g_camera_test_meta.offset_x,
                  (int)g_camera_test_meta.bearing_deg_x10,
                  (unsigned)g_camera_test_meta.bbox_x,
                  (unsigned)g_camera_test_meta.bbox_y,
                  (unsigned)g_camera_test_meta.bbox_width,
                  (unsigned)g_camera_test_meta.bbox_height,
                  (unsigned long)g_camera_test_meta.area_px,
                  (unsigned)g_camera_test_meta.distance_valid,
                  (unsigned)g_camera_test_meta.distance_cm,
                  (unsigned)g_camera_test_meta.process_us,
                  (unsigned)g_camera_test_meta.camera_fps,
                  (unsigned)g_camera_test_meta.vision_load_pct,
                  (unsigned long)g_camera_test_dropped_count);
    if ((len <= 0) || (len >= (int)sizeof(g_camera_test_line)))
    {
        return 31U;
    }

    fr = f_write(&g_camera_test_csv_file,
                 g_camera_test_line,
                 (UINT)len,
                 &written);
    if ((fr != FR_OK) || (written != (UINT)len) ||
        (f_sync(&g_camera_test_csv_file) != FR_OK))
    {
        return 32U;
    }
    g_camera_test_bytes += written;
    return 0U;
}

static uint8 data_logger_camera_test_write_event(void)
{
    uint32 event_id = g_camera_test_event_count;
    uint32 bmp_bytes;
    uint8 ret;

    if (event_id >= DATA_LOGGER_CAMERA_TEST_MAX_EVENTS)
    {
        return 33U;
    }
    sprintf(g_camera_test_frame_filename,
            "%s/E%05lu.BMP",
            g_camera_test_image_directory,
            (unsigned long)event_id);
    ret = data_logger_camera_write_staged_bmp(g_camera_test_frame_filename);
    if (ret != 0U)
    {
        return ret;
    }
    ret = data_logger_camera_test_append_event(
        event_id,
        g_camera_test_frame_filename +
        strlen(g_camera_test_directory) + 1U);
    if (ret != 0U)
    {
        (void)f_unlink(g_camera_test_frame_filename);
        return ret;
    }

    bmp_bytes = DATA_LOGGER_CAMERA_SNAPSHOT_BMP_HEADER_SIZE +
                ((uint32)g_camera_snapshot_width * g_camera_snapshot_height * 3U);
    g_camera_test_bytes += bmp_bytes;
    g_camera_test_event_count++;
    return 0U;
}

static void data_logger_camera_test_fail(uint8 error)
{
    data_logger_camera_test_close();
    g_camera_test_last_error = error;
    g_camera_test_frame_busy = 0U;
    data_logger_memory_sync();
    g_camera_test_state = (uint8)DATA_LOGGER_CAMERA_TEST_FAILED;
    printf("[CAMERA_TEST] failed err=%u dir=%s events=%lu drop=%lu\r\n",
           (unsigned)error,
           g_camera_test_directory,
           (unsigned long)g_camera_test_event_count,
           (unsigned long)g_camera_test_dropped_count);
}

static uint8 data_logger_camera_test_start_mode(uint8 mode,
                                                uint8 allow_logger)
{
    if ((mode != DATA_LOGGER_CAMERA_TEST_MODE_STATIC) &&
        (mode != DATA_LOGGER_CAMERA_TEST_MODE_MOTION))
    {
        return 1U;
    }
    allow_logger = (allow_logger != 0U) ? 1U : 0U;
    if ((allow_logger == 0U) &&
        (g_logger_state != DATA_LOGGER_STATE_IDLE) &&
        (g_logger_state != DATA_LOGGER_STATE_FAILED))
    {
        return 2U;
    }
    if ((data_logger_camera_snapshot_is_busy() != 0U) ||
        (data_logger_camera_record_is_active() != 0U) ||
        (data_logger_camera_test_is_active() != 0U))
    {
        return 3U;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        return 4U;
    }

    g_camera_test_mode = mode;
    g_camera_test_allow_logger = allow_logger;
    g_camera_test_frame_busy = 0U;
    g_camera_test_last_error = 0U;
    g_camera_test_last_reason = 0U;
    g_camera_test_event_count = 0U;
    g_camera_test_dropped_count = 0U;
    g_camera_test_bytes = 0U;
    g_camera_test_directory[0] = '\0';
    g_camera_test_image_directory[0] = '\0';
    g_camera_test_frame_filename[0] = '\0';
    data_logger_memory_sync();
    g_camera_test_state = (uint8)DATA_LOGGER_CAMERA_TEST_STARTING;
    return 0U;
}

uint8 data_logger_camera_test_start(uint8 mode)
{
    return data_logger_camera_test_start_mode(mode, 0U);
}

uint8 data_logger_camera_test_start_parallel(uint8 mode)
{
    return data_logger_camera_test_start_mode(mode, 1U);
}

void data_logger_camera_test_stop(void)
{
    if ((g_camera_test_state == (uint8)DATA_LOGGER_CAMERA_TEST_STARTING) ||
        (g_camera_test_state == (uint8)DATA_LOGGER_CAMERA_TEST_RUNNING))
    {
        data_logger_memory_sync();
        g_camera_test_state = (uint8)DATA_LOGGER_CAMERA_TEST_STOPPING;
    }
}

uint8 data_logger_camera_test_frame_request(
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta,
    uint8 reason)
{
    data_logger_camera_frame_meta_t empty_meta;

    if (image == NULL)
    {
        return 1U;
    }
    if (g_camera_test_state != (uint8)DATA_LOGGER_CAMERA_TEST_RUNNING)
    {
        return 5U;
    }
    if (g_camera_test_event_count >= DATA_LOGGER_CAMERA_TEST_MAX_EVENTS)
    {
        g_camera_test_dropped_count++;
        return 7U;
    }
    if (g_camera_test_frame_busy != 0U)
    {
        g_camera_test_dropped_count++;
        return 6U;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        return 4U;
    }

    data_logger_camera_stage_frame(DATA_LOGGER_CAMERA_SNAPSHOT_FULL,
                                   image,
                                   meta);
    if (meta != NULL)
    {
        g_camera_test_meta = *meta;
    }
    else
    {
        memset(&empty_meta, 0, sizeof(empty_meta));
        g_camera_test_meta = empty_meta;
    }
    g_camera_test_last_reason = (reason == 0U) ?
                                DATA_LOGGER_CAMERA_TEST_REASON_MANUAL : reason;
    data_logger_memory_sync();
    g_camera_test_frame_busy = 1U;
    data_logger_memory_sync();
    return 0U;
}

void data_logger_camera_test_worker_task(uint32 now_ms)
{
    uint8 state = g_camera_test_state;
    uint8 ret;

    if (state == (uint8)DATA_LOGGER_CAMERA_TEST_STARTING)
    {
        if (guimai_voice_is_recording() != 0U)
        {
            data_logger_camera_test_fail(4U);
            return;
        }
        ret = data_logger_camera_test_start_files(now_ms);
        if (ret != 0U)
        {
            data_logger_camera_test_fail(ret);
            return;
        }
        data_logger_memory_sync();
        g_camera_test_state = (uint8)DATA_LOGGER_CAMERA_TEST_RUNNING;
        printf("[CAMERA_TEST] start mode=%s dir=%s\r\n",
               data_logger_camera_test_mode_name(g_camera_test_mode),
               g_camera_test_directory);
        return;
    }

    if ((state != (uint8)DATA_LOGGER_CAMERA_TEST_RUNNING) &&
        (state != (uint8)DATA_LOGGER_CAMERA_TEST_STOPPING))
    {
        return;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        data_logger_camera_test_fail(4U);
        return;
    }
    if (g_camera_test_frame_busy != 0U)
    {
        ret = data_logger_camera_test_write_event();
        if (ret != 0U)
        {
            data_logger_camera_test_fail(ret);
            return;
        }
        data_logger_memory_sync();
        g_camera_test_frame_busy = 0U;
        data_logger_memory_sync();
    }
    if ((g_camera_test_state == (uint8)DATA_LOGGER_CAMERA_TEST_STOPPING) &&
        (g_camera_test_frame_busy == 0U))
    {
        data_logger_camera_test_close();
        data_logger_memory_sync();
        g_camera_test_state = (uint8)DATA_LOGGER_CAMERA_TEST_DONE;
        printf("[CAMERA_TEST] stop dir=%s events=%lu drop=%lu bytes=%lu\r\n",
               g_camera_test_directory,
               (unsigned long)g_camera_test_event_count,
               (unsigned long)g_camera_test_dropped_count,
               (unsigned long)g_camera_test_bytes);
    }
}

uint8 data_logger_camera_test_is_active(void)
{
    return ((g_camera_test_state == (uint8)DATA_LOGGER_CAMERA_TEST_STARTING) ||
            (g_camera_test_state == (uint8)DATA_LOGGER_CAMERA_TEST_RUNNING) ||
            (g_camera_test_state == (uint8)DATA_LOGGER_CAMERA_TEST_STOPPING)) ? 1U : 0U;
}

uint8 data_logger_camera_test_get_state(void)
{
    return g_camera_test_state;
}

uint8 data_logger_camera_test_get_mode(void)
{
    return g_camera_test_mode;
}

uint8 data_logger_camera_test_get_last_error(void)
{
    return g_camera_test_last_error;
}

uint8 data_logger_camera_test_get_last_reason(void)
{
    return g_camera_test_last_reason;
}

uint32 data_logger_camera_test_get_event_count(void)
{
    return g_camera_test_event_count;
}

uint32 data_logger_camera_test_get_dropped_count(void)
{
    return g_camera_test_dropped_count;
}

uint32 data_logger_camera_test_get_bytes(void)
{
    return g_camera_test_bytes;
}

const char *data_logger_camera_test_get_directory(void)
{
    return g_camera_test_directory;
}

static void data_logger_backend_close(void)
{
    if (g_logger_file_open != 0U)
    {
        (void)data_logger_flush();
        (void)f_sync(&g_logger_file);
        (void)f_close(&g_logger_file);
    }

    g_logger_file_open = 0U;
    g_logger_buffer_used = 0U;
}

static void data_logger_backend_reset_mount(void)
{
    g_logger_mounted = 0U;
    sd_simple_reset();
}

static void data_logger_backend_fail(uint8 error)
{
    data_logger_backend_close();
    data_logger_backend_reset_mount();
    g_logger_last_error = error;
    data_logger_memory_sync();
    g_logger_state = DATA_LOGGER_STATE_FAILED;
    printf("[LOGGER] failed on CPU1, err=%u\r\n", (unsigned)error);
}

static uint8 data_logger_backend_start(uint32 now_ms)
{
    FRESULT fr;
    uint8 ret;

    if (g_logger_file_open != 0U)
    {
        return 0U;
    }

    ret = data_logger_backend_init();
    if (ret != 0U)
    {
        return ret;
    }

    ret = data_logger_make_filename();
    if (ret != 0U)
    {
        printf("[LOGGER] filename alloc failed, ret=%u\r\n", ret);
        return 5U;
    }

    fr = f_open(&g_logger_file, g_logger_filename, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK)
    {
        printf("[LOGGER] f_open failed, fr=%d file=%s\r\n", (int)fr, g_logger_filename);
        return 6U;
    }

    g_logger_buffer_used = 0U;
    g_logger_last_sync_ms = now_ms;
    g_logger_file_open = 1U;

    if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_GPS_TEST)
    {
        ret = data_logger_append(DATA_LOGGER_GPS_TEST_HEADER, (uint32)strlen(DATA_LOGGER_GPS_TEST_HEADER));
    }
    else if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_IMU_TEST)
    {
        ret = data_logger_append(DATA_LOGGER_IMU_TEST_HEADER, (uint32)strlen(DATA_LOGGER_IMU_TEST_HEADER));
    }
    else if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_TASK2_INS_DEBUG)
    {
        ret = data_logger_append(DATA_LOGGER_TASK2_INS_HEADER, (uint32)strlen(DATA_LOGGER_TASK2_INS_HEADER));
    }
    else if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_DRIVE_DEBUG)
    {
        ret = data_logger_append(DATA_LOGGER_DRIVE_HEADER, (uint32)strlen(DATA_LOGGER_DRIVE_HEADER));
    }
    else
    {
        ret = data_logger_append(DATA_LOGGER_FULL_HEADER, (uint32)strlen(DATA_LOGGER_FULL_HEADER));
    }
    if (ret != 0U)
    {
        return 7U;
    }
    if (data_logger_flush() != 0U)
    {
        return 8U;
    }
    if (f_sync(&g_logger_file) != FR_OK)
    {
        return 9U;
    }

    printf("[LOGGER] start on CPU1: %s profile=%u task=%u src=%u\r\n",
           g_logger_filename,
           (unsigned)g_logger_profile,
           (unsigned)g_logger_task_id,
           (unsigned)g_logger_target_src);
    return 0U;
}

static void data_logger_capture_sample(uint32 now_ms,
                                       data_logger_sample_t *sample,
                                       const nav_gps_sample_t *gps_override)
{
    pid_status_t pid_status;
    dianji_status_t motor_status;
    taban_status_t pedal_status;

    memset(sample, 0, sizeof(*sample));
    pid_get_status(&pid_status);
    dianji_get_status(&motor_status);
    taban_get_status(&pedal_status);
    nav_control_get_state(&sample->nav_state);
    nav_ins_get_state(&sample->ins_state);
    nav_gps_get_last_sample(&sample->gps_sample);
    ctrl_task2_get_path_debug(&sample->task2_path_dbg);
    ctrl_task3_follow_get_status(&sample->task3_follow_status);

    sample->time_ms = data_logger_extend_time_ms(now_ms);
    sample->task_id = g_logger_task_id;
    sample->task_state = g_task_ctrl_state;
    sample->profile = g_logger_profile;
    sample->abs_raw = g_abs_encoder_raw;
    sample->steer_center_raw = g_steer_center_raw;
    sample->steer_delta_counts = g_steer_delta_counts;
    sample->steer_speed_counts_s = g_steer_feedback_speed_counts_s;
    sample->steer_target_deg = g_steer_target_angle_deg;
    sample->steer_feedback_deg = g_steer_feedback_angle_deg;
    sample->steer_valid = g_steer_feedback_valid;
    sample->steer_sensor_fault = g_steer_sensor_fault;
    sample->steer_stall_fault = g_steer_stall_fault;
    sample->steer_raw_read = g_steer_raw_read;
    sample->steer_raw_last_valid = g_steer_raw_last_valid;
    sample->steer_raw_jump_counts = g_steer_raw_jump_counts;
    sample->steer_raw_last_reject_reason =
        g_steer_raw_last_reject_reason;
    sample->steer_raw_last_reject_streak =
        g_steer_raw_last_reject_streak;
    sample->steer_raw_reject_count = g_steer_raw_reject_count;
    sample->steer_raw_last_reject_ms = g_steer_raw_last_reject_ms;
    sample->steer_raw_last_reject_raw = g_steer_raw_last_reject_raw;
    sample->steer_raw_last_reject_ref_raw =
        g_steer_raw_last_reject_ref_raw;
    sample->steer_raw_last_reject_jump_counts =
        g_steer_raw_last_reject_jump_counts;
    sample->steer_scan_phase = g_steer_scan_phase;
    sample->steer_scan_success = g_steer_scan_success;
    sample->steer_scan_left_raw = g_steer_scan_left_raw;
    sample->steer_scan_right_raw = g_steer_scan_right_raw;
    sample->steer_scan_center_raw = g_steer_scan_center_raw;
    sample->steer_scan_span_counts = g_steer_scan_span_counts;
    sample->ecod1_count = g_ecod1_count;
    sample->ecod2_count = g_ecod2_count;
    sample->ecod1_speed = g_ecod1_speed;
    sample->ecod2_speed = g_ecod2_speed;
    sample->ecod1_speed_filt = g_ecod1_speed_filt;
    sample->ecod2_speed_filt = g_ecod2_speed_filt;
    sample->target_speed_mps = g_pid_used_target_speed_mps;
    sample->pid_out_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].output;
    sample->pid_out_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].output;
    sample->pid_out_steer = pid_status.ch[PID_CTRL_FRONT_STEER_ANGLE].output;
    sample->pid_p_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].p_term;
    sample->pid_i_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].i_term;
    sample->pid_d_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].d_term;
    sample->pid_boost_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].accel_boost_ticks;
    sample->pid_dir_guard_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].direction_guard_active;
    sample->pid_p_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].p_term;
    sample->pid_i_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].i_term;
    sample->pid_d_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].d_term;
    sample->pid_boost_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].accel_boost_ticks;
    sample->pid_dir_guard_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].direction_guard_active;
    sample->pid_target_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].target;
    sample->pid_feedback_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].feedback;
    sample->pid_error_r = pid_status.ch[PID_CTRL_REAR_RIGHT_SPEED].error;
    sample->pid_target_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].target;
    sample->pid_feedback_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].feedback;
    sample->pid_error_l = pid_status.ch[PID_CTRL_REAR_LEFT_SPEED].error;
    sample->duty_m = motor_status.target_duty[DIANJI_MOTOR_M];
    sample->duty_r = motor_status.target_duty[DIANJI_MOTOR_R];
    sample->duty_l = motor_status.target_duty[DIANJI_MOTOR_L];
    sample->pid_ff_r = (float)sample->duty_r - sample->pid_out_r;
    sample->pid_ff_l = (float)sample->duty_l - sample->pid_out_l;
    sample->imu_yaw = IMU_RelYaw;
    sample->gyro_z = IMU_GYRO_Z;
    sample->imu_rel_pitch = IMU_RelPitch;
    sample->imu_rel_roll = IMU_RelRoll;
    sample->imu_abs_pitch = IMU_Pitch;
    sample->imu_abs_roll = IMU_Roll;
    sample->imu_abs_yaw = IMU_Yaw;
    sample->imu_mag_raw_x = IMU_MagRawX;
    sample->imu_mag_raw_y = IMU_MagRawY;
    sample->imu_mag_raw_z = IMU_MagRawZ;
    sample->imu_init_ret = g_imu963ra_init_ret;
    sample->imu_pit_count = g_imu_pit_count;
    sample->imu_update_count = g_imu_update_count;
    sample->imu_read_fail_count = g_imu_read_fail_count;
    if (g_imu_update_ms == 0U)
    {
        sample->imu_update_age_ms = 0xFFFFFFFFU;
    }
    else if (now_ms >= g_imu_update_ms)
    {
        sample->imu_update_age_ms = now_ms - g_imu_update_ms;
    }
    else
    {
        sample->imu_update_age_ms = 0U;
    }
    IMU_AHRS_GetDebugInfo(&sample->imu_dbg);
    sample->target_src = g_pid_target_src;
    sample->gnss_state = gnss.state;
    sample->remote_connected = g_yaokong.connected;
    sample->remote_armed = yaokong_remote_is_armed();
    sample->remote_state = g_yaokong.state;
    sample->remote_guard = g_yaokong.guard_enable;
    sample->remote_motor_run = yaokong_is_motor_run_enabled();
    sample->remote_ch5_pressed = g_yaokong.ch5_pressed;
    sample->remote_ch6_pressed = g_yaokong.ch6_pressed;
    sample->remote_raw_steer = g_yaokong.channel[YAOKONG_CHANNEL_STEER];
    sample->remote_raw_throttle = g_yaokong.channel[YAOKONG_CHANNEL_THROTTLE];
    sample->remote_raw_ch5 = g_yaokong.channel[YAOKONG_CHANNEL_CH5_BUTTON];
    sample->remote_raw_ch[0] = g_yaokong.channel[0];
    sample->remote_raw_ch[1] = g_yaokong.channel[1];
    sample->remote_raw_ch[2] = g_yaokong.channel[2];
    sample->remote_raw_ch[3] = g_yaokong.channel[3];
    sample->remote_raw_ch[4] = g_yaokong.channel[4];
    sample->remote_raw_ch[5] = g_yaokong.channel[5];
    sample->remote_target_speed_mps = g_yaokong.target_speed_mps;
    sample->remote_target_steer_deg = g_yaokong.target_steer_angle_deg;
    sample->remote_last_frame_age_ms = yaokong_get_last_frame_age_ms();
    sample->remote_valid_frames = yaokong_get_valid_frame_count();
    sample->remote_bad_frames = yaokong_get_bad_frame_count();
    sample->pedal_raw_adc[TABAN_ID_0] = pedal_status.raw_adc[TABAN_ID_0];
    sample->pedal_raw_adc[TABAN_ID_1] = pedal_status.raw_adc[TABAN_ID_1];
    sample->pedal_filt_adc[TABAN_ID_0] = pedal_status.filt_adc[TABAN_ID_0];
    sample->pedal_filt_adc[TABAN_ID_1] = pedal_status.filt_adc[TABAN_ID_1];
    sample->pedal_permille[TABAN_ID_0] = pedal_status.throttle_permille[TABAN_ID_0];
    sample->pedal_permille[TABAN_ID_1] = pedal_status.throttle_permille[TABAN_ID_1];
    sample->pedal_released_once[TABAN_ID_0] = pedal_status.released_once[TABAN_ID_0];
    sample->pedal_released_once[TABAN_ID_1] = pedal_status.released_once[TABAN_ID_1];
    sample->pedal_fault_active[TABAN_ID_0] = pedal_status.fault_active[TABAN_ID_0];
    sample->pedal_fault_active[TABAN_ID_1] = pedal_status.fault_active[TABAN_ID_1];
    sample->pedal_target_speed_mps = pedal_status.target_speed_mps;
    sample->imu_acc_x_g = imu_data.acc_x;
    sample->imu_acc_y_g = imu_data.acc_y;
    sample->imu_acc_z_g = imu_data.acc_z;
    if (gps_override != NULL)
    {
        sample->gps_sample = *gps_override;
    }
    else
    {
        nav_gps_get_last_sample(&sample->gps_sample);
    }
    if ((sample->profile == (uint8)DATA_LOGGER_PROFILE_GPS_TEST) &&
        (sample->gps_sample.valid != 0U))
    {
        float dx_m;
        float dy_m;

        if (g_logger_gps_test_origin_valid == 0U)
        {
            g_logger_gps_test_origin_valid = 1U;
            g_logger_gps_test_origin_x_m = sample->gps_sample.x_m;
            g_logger_gps_test_origin_y_m = sample->gps_sample.y_m;
        }

        sample->gps_test_origin_x_m = g_logger_gps_test_origin_x_m;
        sample->gps_test_origin_y_m = g_logger_gps_test_origin_y_m;
        dx_m = sample->gps_sample.x_m - sample->gps_test_origin_x_m;
        dy_m = sample->gps_sample.y_m - sample->gps_test_origin_y_m;
        sample->gps_test_drift_m = sqrtf(dx_m * dx_m + dy_m * dy_m);
    }
    sample->ctrl_cmd = g_ctrl_cmd;
    sample->key_num = Key_Num;
    sample->key_event_num = g_key_last_event_num;
    sample->key_event_count = g_key_event_count;
    sample->key_event_age_ms = (g_key_last_event_time_ms != 0U) ? (now_ms - g_key_last_event_time_ms) : 0U;
    sample->ctrl_stop_reason = g_ctrl_stop_reason;
    sample->ctrl_safety_pause_active = g_ctrl_safety_pause_active;
    sample->ctrl_main_loop_age_ms = g_ctrl_main_loop_age_ms;
    sample->ctrl_cpu0_age_ms = g_ctrl_cpu0_age_ms;
    sample->ctrl_cpu0_stage = g_ctrl_cpu0_stage;
    sample->ctrl_cpu0_stall_stage = g_ctrl_cpu0_stall_stage;
    sample->ctrl_cpu0_stall_count = g_ctrl_cpu0_stall_count;
    sample->ctrl_cpu0_stall_max_age_ms = g_ctrl_cpu0_stall_max_age_ms;
    sample->ctrl_cmd_age_ms = g_ctrl_cmd_age_ms;
    sample->steer_feedback_age_ms = (g_steer_feedback_update_ms != 0U) ?
                                    (now_ms - g_steer_feedback_update_ms) :
                                    0xFFFFFFFFU;
    sample->ctrl_safety_pause_until_ms = g_ctrl_safety_pause_until_ms;
    sample->ctrl_pid_reset_reason = g_ctrl_pid_reset_reason;
    sample->ctrl_pid_reset_age_ms = (g_ctrl_pid_reset_time_ms != 0U) ?
                                    (now_ms - g_ctrl_pid_reset_time_ms) :
                                    0xFFFFFFFFU;
    sample->ctrl_pid_low_reason = g_ctrl_pid_low_reason;
    sample->ctrl_pid_low_age_ms = (g_ctrl_pid_low_time_ms != 0U) ?
                                  (now_ms - g_ctrl_pid_low_time_ms) :
                                  0xFFFFFFFFU;
    sample->ctrl_pid_low_target_r = g_ctrl_pid_low_target_r;
    sample->ctrl_pid_low_feedback_r = g_ctrl_pid_low_feedback_r;
    sample->ctrl_pid_low_output_r = g_ctrl_pid_low_output_r;
    sample->ctrl_pid_low_target_l = g_ctrl_pid_low_target_l;
    sample->ctrl_pid_low_feedback_l = g_ctrl_pid_low_feedback_l;
    sample->ctrl_pid_low_output_l = g_ctrl_pid_low_output_l;
    data_logger_update_task2_freeze_diag(sample, now_ms);
    sample->marker = g_logger_marker;
    g_logger_marker = 0U;
    sample->logger_dropped = g_logger_dropped_count;
}

static uint8 data_logger_format_drive_sample(const data_logger_sample_t *sample)
{
    int len;
    float drive_target_ecod = sample->target_speed_mps * PID_MPS_TO_ECOD_SPEED;

    len = sprintf(g_logger_line,
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%.2f,%u,%u,%u,%.3f,%.1f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%.1f,%d,%d,%d,%d,%d,%d,%.1f,%.2f,%.2f,%u,%u,%u,%u,%u,%d,%d,%d,%d,%.1f,%u,%u,%u,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%u\r\n",
                  (unsigned)sample->time_ms,
                  (unsigned)sample->marker,
                  (unsigned)sample->task_id,
                  (unsigned)sample->task_state,
                  (unsigned)sample->profile,
                  (unsigned)sample->target_src,
                  (unsigned)sample->remote_connected,
                  (unsigned)sample->remote_armed,
                  (unsigned)sample->remote_state,
                  (unsigned)sample->remote_guard,
                  (unsigned)sample->remote_motor_run,
                  (unsigned)sample->remote_ch5_pressed,
                  (unsigned)sample->remote_raw_steer,
                  (unsigned)sample->remote_raw_throttle,
                  (unsigned)sample->remote_raw_ch5,
                  (double)sample->remote_target_speed_mps,
                  (double)sample->remote_target_steer_deg,
                  (unsigned)sample->remote_last_frame_age_ms,
                  (unsigned)sample->remote_valid_frames,
                  (unsigned)sample->remote_bad_frames,
                  (double)sample->target_speed_mps,
                  (double)drive_target_ecod,
                  (unsigned)sample->pedal_raw_adc[TABAN_ID_0],
                  (unsigned)sample->pedal_raw_adc[TABAN_ID_1],
                  (unsigned)sample->pedal_filt_adc[TABAN_ID_0],
                  (unsigned)sample->pedal_filt_adc[TABAN_ID_1],
                  (unsigned)sample->pedal_permille[TABAN_ID_0],
                  (unsigned)sample->pedal_permille[TABAN_ID_1],
                  (unsigned)sample->pedal_released_once[TABAN_ID_0],
                  (unsigned)sample->pedal_released_once[TABAN_ID_1],
                  (unsigned)sample->pedal_fault_active[TABAN_ID_0],
                  (unsigned)sample->pedal_fault_active[TABAN_ID_1],
                  (double)sample->pedal_target_speed_mps,
                  (int)sample->ecod1_speed,
                  (int)sample->ecod2_speed,
                  (int)sample->ecod1_speed_filt,
                  (int)sample->ecod2_speed_filt,
                  (double)sample->pid_target_r,
                  (double)sample->pid_feedback_r,
                  (double)sample->pid_error_r,
                  (double)sample->pid_ff_r,
                  (double)sample->pid_p_r,
                  (double)sample->pid_i_r,
                  (double)sample->pid_d_r,
                  (unsigned)sample->pid_boost_r,
                  (unsigned)sample->pid_dir_guard_r,
                  (double)sample->pid_out_r,
                  (double)sample->pid_target_l,
                  (double)sample->pid_feedback_l,
                  (double)sample->pid_error_l,
                  (double)sample->pid_ff_l,
                  (double)sample->pid_p_l,
                  (double)sample->pid_i_l,
                  (double)sample->pid_d_l,
                  (unsigned)sample->pid_boost_l,
                  (unsigned)sample->pid_dir_guard_l,
                  (double)sample->pid_out_l,
                  (int)sample->duty_r,
                  (int)sample->duty_l,
                  (int)sample->duty_m,
                  (int)sample->abs_raw,
                  (int)sample->steer_center_raw,
                  (int)sample->steer_delta_counts,
                  (double)sample->steer_speed_counts_s,
                  (double)sample->steer_target_deg,
                  (double)sample->steer_feedback_deg,
                  (unsigned)sample->steer_valid,
                  (unsigned)sample->steer_sensor_fault,
                  (unsigned)sample->steer_stall_fault,
                  (unsigned)sample->steer_scan_phase,
                  (unsigned)sample->steer_scan_success,
                  (int)sample->steer_scan_left_raw,
                  (int)sample->steer_scan_right_raw,
                  (int)sample->steer_scan_center_raw,
                  (int)sample->steer_scan_span_counts,
                  (double)sample->pid_out_steer,
                  (unsigned)sample->ctrl_cmd.use_pid,
                  (unsigned)sample->ctrl_cmd.direct_pwm,
                  (unsigned)sample->ctrl_cmd.steer_closed_loop,
                  (unsigned)sample->ctrl_cmd.brake_to_stop,
                  (double)sample->ctrl_cmd.target_angle_deg,
                  (unsigned)sample->key_num,
                  (unsigned)sample->key_event_num,
                  (unsigned)sample->key_event_count,
                  (unsigned)sample->key_event_age_ms,
                  (unsigned)sample->ctrl_stop_reason,
                  (unsigned)sample->ctrl_safety_pause_active,
                  (unsigned)sample->ctrl_main_loop_age_ms,
                  (unsigned)sample->ctrl_safety_pause_until_ms,
                  (unsigned)sample->logger_dropped,
                  (double)sample->ctrl_cmd.target_speed_mps,
                  (unsigned)sample->ctrl_cmd_age_ms,
                  (unsigned)sample->steer_feedback_age_ms,
                  (unsigned)sample->ctrl_pid_reset_reason,
                  (unsigned)sample->ctrl_pid_reset_age_ms,
                  (unsigned)sample->ctrl_pid_low_reason,
                  (unsigned)sample->ctrl_pid_low_age_ms,
                  (double)sample->ctrl_pid_low_target_r,
                  (double)sample->ctrl_pid_low_feedback_r,
                  (double)sample->ctrl_pid_low_output_r,
                  (double)sample->ctrl_pid_low_target_l,
                  (double)sample->ctrl_pid_low_feedback_l,
                  (double)sample->ctrl_pid_low_output_l,
                  (unsigned)sample->remote_raw_ch[0],
                  (unsigned)sample->remote_raw_ch[1],
                  (unsigned)sample->remote_raw_ch[2],
                  (unsigned)sample->remote_raw_ch[3],
                  (unsigned)sample->remote_raw_ch[4],
                  (unsigned)sample->remote_raw_ch[5],
                  (unsigned)sample->remote_ch6_pressed);

    if ((len <= 0) || (len >= (int)sizeof(g_logger_line)))
    {
        printf("[LOGGER] drive line format failed, len=%d\r\n", len);
        return 1U;
    }

    return data_logger_append(g_logger_line, (uint32)len);
}

static uint8 data_logger_format_full_sample(const data_logger_sample_t *sample)
{
    int len;
    const nav_state_t *nav = &sample->nav_state;

    len = sprintf(g_logger_line,
                  "%u,%u,%u,%u,%u,%d,%d,%d,%.1f,%.2f,%.2f,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.1f,%.1f,%u,%u,%.1f,%d,%d,%d,%.2f,%.2f,%u,"
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%.2f,%u,%u,%u,%u,%u,%u,%u,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%u,%u,%u,%.2f,%.2f,"
                  "%u,%u,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%d,%.2f,%.3f,%u,%u,%u,%u,"
                  "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%.3f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%u,"
                  "%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%.3f,%.2f,%d,%d,%d,%u,%u,%u,%u,%d,%d,%d\r\n",
                  (unsigned)sample->time_ms,
                  (unsigned)sample->marker,
                  (unsigned)sample->task_id,
                  (unsigned)sample->task_state,
                  (unsigned)sample->profile,
                  (int)sample->abs_raw,
                  (int)sample->steer_center_raw,
                  (int)sample->steer_delta_counts,
                  (double)sample->steer_speed_counts_s,
                  (double)sample->steer_target_deg,
                  (double)sample->steer_feedback_deg,
                  (unsigned)sample->steer_valid,
                  (unsigned)sample->steer_sensor_fault,
                  (unsigned)sample->steer_stall_fault,
                  (unsigned)sample->steer_scan_phase,
                  (unsigned)sample->steer_scan_success,
                  (int)sample->steer_scan_left_raw,
                  (int)sample->steer_scan_right_raw,
                  (int)sample->steer_scan_center_raw,
                  (int)sample->steer_scan_span_counts,
                  (int)sample->ecod1_count,
                  (int)sample->ecod2_count,
                  (int)sample->ecod1_speed,
                  (int)sample->ecod2_speed,
                  (int)sample->ecod1_speed_filt,
                  (int)sample->ecod2_speed_filt,
                  (double)sample->target_speed_mps,
                  (double)sample->pid_out_r,
                  (double)sample->pid_out_l,
                  (unsigned)sample->pid_dir_guard_r,
                  (unsigned)sample->pid_dir_guard_l,
                  (double)sample->pid_out_steer,
                  (int)sample->duty_m,
                  (int)sample->duty_r,
                  (int)sample->duty_l,
                  (double)sample->imu_yaw,
                  (double)sample->gyro_z,
                  (unsigned)sample->target_src,
                  (unsigned)sample->remote_connected,
                  (unsigned)sample->remote_armed,
                  (unsigned)sample->remote_state,
                  (unsigned)sample->remote_guard,
                  (unsigned)sample->remote_motor_run,
                  (unsigned)sample->remote_ch5_pressed,
                  (unsigned)sample->remote_raw_steer,
                  (unsigned)sample->remote_raw_throttle,
                  (unsigned)sample->remote_raw_ch5,
                  (double)sample->remote_target_speed_mps,
                  (double)sample->remote_target_steer_deg,
                  (unsigned)sample->remote_last_frame_age_ms,
                  (unsigned)sample->remote_valid_frames,
                  (unsigned)sample->remote_bad_frames,
                  (unsigned)nav->valid,
                  (unsigned)nav->gps_valid,
                  (unsigned)nav->ins_valid,
                  (unsigned)nav->timestamp_ms,
                  (double)nav->x_m,
                  (double)nav->y_m,
                  (double)nav->yaw_deg,
                  (double)nav->yaw_rate_dps,
                  (double)nav->speed_mps,
                  (double)nav->encoder_speed_mps,
                  (double)nav->distance_m,
                  (double)nav->forward_accel_mps2,
                  (double)sample->imu_acc_x_g,
                  (double)sample->imu_acc_y_g,
                  (double)sample->imu_acc_z_g,
                  (unsigned)sample->imu_dbg.compare_enabled,
                  (unsigned)sample->imu_dbg.compare_valid,
                  (unsigned)sample->imu_dbg.compare_shadow_algo,
                  (double)sample->imu_dbg.cmp_shadow_yaw,
                  (double)sample->imu_dbg.cmp_delta_yaw,
                  (unsigned)nav->path_valid,
                  (unsigned)nav->path_active,
                  (unsigned)nav->path_finished,
                  (unsigned)nav->path_mode,
                  (unsigned)nav->path_sample_count,
                  (unsigned)nav->path_replay_index,
                  (double)nav->path_sample_interval_m,
                  (double)nav->path_progress_m,
                  (double)nav->path_ref_x_m,
                  (double)nav->path_ref_y_m,
                  (double)nav->path_ref_yaw_deg,
                  (double)nav->path_ref_kappa_1pm,
                  (double)nav->path_ref_kappa_ahead_1pm,
                  (double)nav->path_ref_kappa_yaw_1pm,
                  (double)nav->path_ref_speed_mps,
                  (double)nav->path_ref_speed_ahead_mps,
                  (int)nav->path_ref_motion_dir,
                  (int)nav->path_ref_motion_dir_ahead,
                  (double)nav->path_yaw_error_deg,
                  (double)nav->path_lateral_error_m,
                  (unsigned)nav->path_last_sd_result,
                  (unsigned)nav->path_last_flash_result,
                  (unsigned)nav->path_last_flash_load_result,
                  (unsigned)sample->logger_dropped,
                  (double)sample->pid_target_r,
                  (double)sample->pid_feedback_r,
                  (double)sample->pid_error_r,
                  (double)sample->pid_target_l,
                  (double)sample->pid_feedback_l,
                  (double)sample->pid_error_l,
                  (unsigned)sample->ctrl_cmd.use_pid,
                  (unsigned)sample->ctrl_cmd.direct_pwm,
                  (unsigned)sample->ctrl_cmd.steer_closed_loop,
                  (unsigned)sample->ctrl_cmd.brake_to_stop,
                  (double)sample->ctrl_cmd.target_speed_mps,
                  (double)sample->ctrl_cmd.target_angle_deg,
                  (unsigned)sample->ctrl_stop_reason,
                  (unsigned)sample->ctrl_safety_pause_active,
                  (unsigned)sample->ctrl_main_loop_age_ms,
                  (unsigned)sample->ctrl_cpu0_age_ms,
                  (unsigned)sample->ctrl_cpu0_stage,
                  (unsigned)sample->ctrl_cpu0_stall_stage,
                  (unsigned)sample->ctrl_cpu0_stall_count,
                  (unsigned)sample->ctrl_cpu0_stall_max_age_ms,
                  (unsigned)sample->ctrl_cmd_age_ms,
                  (unsigned)sample->steer_feedback_age_ms,
                  (unsigned)sample->ctrl_safety_pause_until_ms,
                  (unsigned)sample->ctrl_pid_reset_reason,
                  (unsigned)sample->ctrl_pid_reset_age_ms,
                  (unsigned)sample->ctrl_pid_low_reason,
                  (unsigned)sample->ctrl_pid_low_age_ms,
                  (double)sample->ctrl_pid_low_target_r,
                  (double)sample->ctrl_pid_low_feedback_r,
                  (double)sample->ctrl_pid_low_output_r,
                  (double)sample->ctrl_pid_low_target_l,
                  (double)sample->ctrl_pid_low_feedback_l,
                  (double)sample->ctrl_pid_low_output_l,
                  (unsigned)sample->remote_raw_ch[0],
                  (unsigned)sample->remote_raw_ch[1],
                  (unsigned)sample->remote_raw_ch[2],
                  (unsigned)sample->remote_raw_ch[3],
                  (unsigned)sample->remote_raw_ch[4],
                  (unsigned)sample->remote_raw_ch[5],
                  (unsigned)sample->remote_ch6_pressed,
                  (unsigned)sample->task3_follow_status.active,
                  (unsigned)sample->task3_follow_status.phase,
                  (unsigned)sample->task3_follow_status.error,
                  (unsigned)sample->task3_follow_status.target_valid,
                  (unsigned)sample->task3_follow_status.target_stable,
                  (unsigned)sample->task3_follow_status.confidence_pct,
                  (unsigned)sample->task3_follow_status.distance_valid,
                  (int)sample->task3_follow_status.bearing_deg_x10,
                  (unsigned)sample->task3_follow_status.bbox_height,
                  (unsigned)sample->task3_follow_status.distance_cm,
                  (unsigned)sample->task3_follow_status.path_samples,
                  (unsigned)sample->task3_follow_status.frame_age_ms,
                  (double)sample->task3_follow_status.target_speed_mps,
                  (double)sample->task3_follow_status.target_angle_deg,
                  (int)sample->steer_raw_read,
                  (int)sample->steer_raw_last_valid,
                  (int)sample->steer_raw_jump_counts,
                  (unsigned)sample->steer_raw_last_reject_reason,
                  (unsigned)sample->steer_raw_last_reject_streak,
                  (unsigned)sample->steer_raw_reject_count,
                  (unsigned)sample->steer_raw_last_reject_ms,
                  (int)sample->steer_raw_last_reject_raw,
                  (int)sample->steer_raw_last_reject_ref_raw,
                  (int)sample->steer_raw_last_reject_jump_counts);

    if ((len <= 0) || (len >= (int)sizeof(g_logger_line)))
    {
        printf("[LOGGER] line format failed, len=%d\r\n", len);
        return 1U;
    }

    return data_logger_append(g_logger_line, (uint32)len);
}

static uint8 data_logger_format_imu_test_sample(const data_logger_sample_t *sample)
{
    int len;
    const imu_ahrs_debug_t *dbg = &sample->imu_dbg;

    len = sprintf(g_logger_line,
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
                  "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,"
                  "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                  "%.4f,%.4f,%.4f,%.4f,%.3f,"
                  "%.3f,%.3f,%.3f,%.3f,%.3f,"
                  "%.3f,%.3f,%.3f,%.3f,"
                  "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                  "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\r\n",
                  (unsigned)sample->time_ms,
                  (unsigned)sample->marker,
                  (unsigned)sample->task_id,
                  (unsigned)sample->task_state,
                  (unsigned)sample->profile,
                  (unsigned)dbg->fusion_algo,
                  (unsigned)dbg->compare_enabled,
                  (unsigned)dbg->compare_valid,
                  (unsigned)dbg->compare_shadow_algo,
                  (unsigned)sample->imu_init_ret,
                  (unsigned)sample->imu_pit_count,
                  (unsigned)sample->imu_update_count,
                  (unsigned)sample->imu_update_age_ms,
                  (unsigned)sample->imu_read_fail_count,
                  (double)sample->imu_rel_pitch,
                  (double)sample->imu_rel_roll,
                  (double)sample->imu_yaw,
                  (double)sample->imu_abs_pitch,
                  (double)sample->imu_abs_roll,
                  (double)sample->imu_abs_yaw,
                  (double)sample->gyro_z,
                  (unsigned)dbg->receive_flag,
                  (unsigned)dbg->filter_initialized,
                  (unsigned)dbg->dt_initialized,
                  (unsigned)dbg->attitude_initialized,
                  (unsigned)dbg->ref_locked,
                  (unsigned)dbg->mag_enabled,
                  (unsigned)dbg->ref_capture_request,
                  (unsigned)dbg->gyro_bias_ready,
                  (unsigned)dbg->gyro_bias_sample_count,
                  (unsigned)dbg->gyro_bias_sample_target,
                  (double)dbg->sample_dt_s,
                  (double)dbg->sample_dt_cfg_s,
                  (double)dbg->gyro_x_dps,
                  (double)dbg->gyro_y_dps,
                  (double)dbg->gyro_z_dps,
                  (double)dbg->gyro_norm_dps,
                  (double)dbg->gyro_bias_x_dps,
                  (double)dbg->gyro_bias_y_dps,
                  (double)dbg->gyro_bias_z_dps,
                  (double)sample->imu_acc_x_g,
                  (double)sample->imu_acc_y_g,
                  (double)sample->imu_acc_z_g,
                  (double)dbg->acc_norm_g,
                  (double)dbg->accel_confidence,
                  (double)imu_data.mag_x,
                  (double)imu_data.mag_y,
                  (double)imu_data.mag_z,
                  (double)dbg->mag_norm,
                  (double)dbg->mag_confidence,
                  (double)sample->imu_mag_raw_x,
                  (double)sample->imu_mag_raw_y,
                  (double)sample->imu_mag_raw_z,
                  (double)dbg->mag_norm_ref,
                  (double)dbg->cmp_main_pitch,
                  (double)dbg->cmp_main_roll,
                  (double)dbg->cmp_main_yaw,
                  (double)dbg->cmp_shadow_pitch,
                  (double)dbg->cmp_shadow_roll,
                  (double)dbg->cmp_shadow_yaw,
                  (double)dbg->shadow_abs_pitch,
                  (double)dbg->shadow_abs_roll,
                  (double)dbg->shadow_abs_yaw,
                  (double)dbg->cmp_delta_pitch,
                  (double)dbg->cmp_delta_roll,
                  (double)dbg->cmp_delta_yaw,
                  (unsigned)sample->logger_dropped);

    if ((len <= 0) || (len >= (int)sizeof(g_logger_line)))
    {
        printf("[LOGGER] imu test line format failed, len=%d\r\n", len);
        return 1U;
    }

    return data_logger_append(g_logger_line, (uint32)len);
}

static uint8 data_logger_format_gps_test_sample(const data_logger_sample_t *sample)
{
    int len;
    const nav_gps_sample_t *gps = &sample->gps_sample;
    const nav_state_t *nav = &sample->nav_state;

    len = sprintf(g_logger_line,
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.8f,%.8f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%u,%u,%u,%.3f,%.3f,%.2f,%.3f,%.3f,%u,%.3f,%.3f,%u\r\n",
                  (unsigned)sample->time_ms,
                  (unsigned)sample->marker,
                  (unsigned)sample->gps_test_event,
                  (unsigned)sample->gps_test_point_id,
                  (unsigned)sample->task_id,
                  (unsigned)sample->task_state,
                  (unsigned)sample->profile,
                  (unsigned)gps->valid,
                  (unsigned)gps->timestamp_ms,
                  (unsigned)gps->satellite_used,
                  (unsigned)sample->gnss_state,
                  (double)gps->latitude_deg,
                  (double)gps->longitude_deg,
                  (double)gps->x_m,
                  (double)gps->y_m,
                  (double)gps->speed_mps,
                  (double)gps->course_deg,
                  (double)sample->gps_test_origin_x_m,
                  (double)sample->gps_test_origin_y_m,
                  (double)sample->gps_test_drift_m,
                  (unsigned)nav->valid,
                  (unsigned)nav->gps_valid,
                  (unsigned)nav->ins_valid,
                  (double)nav->x_m,
                  (double)nav->y_m,
                  (double)nav->yaw_deg,
                  (double)nav->speed_mps,
                  (double)nav->gps_ins_error_m,
                  (unsigned)nav->gps_ins_correction_enabled,
                  (double)nav->gps_ins_correction_gain,
                  (double)nav->gps_ins_snap_threshold_m,
                  (unsigned)sample->logger_dropped);

    if ((len <= 0) || (len >= (int)sizeof(g_logger_line)))
    {
        printf("[LOGGER] gps test line format failed, len=%d\r\n", len);
        return 1U;
    }

    return data_logger_append(g_logger_line, (uint32)len);
}

static uint8 data_logger_format_task2_ins_sample(const data_logger_sample_t *sample)
{
    int len;
    const nav_state_t *nav = &sample->nav_state;
    const nav_ins_state_t *ins = &sample->ins_state;

    len = sprintf(g_logger_line,
                  "%u,%u,%u,%u,%u,%u,"
                  "%d,%d,%d,%d,%d,%d,"
                  "%u,%u,%u,%u,%u,"
                  "%d,%d,%.1f,%.4f,%.7f,"
                  "%.2f,%.2f,%.2f,%.2f,%.2f,"
                  "%.2f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.2f,%u,%u,"
                  "%.4f,%.4f,%.3f,%.3f,%.3f,"
                  "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
                  "%u,%u,%u,%u,%u,%u,%.3f,%.3f,"
                  "%.3f,%.3f,%.2f,%.4f,%.4f,%.3f,%.2f,%.3f,%u,%u,%u,"
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,"
                  "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,"
                  "%.2f,%.2f,%u,%u,%u,%.3f,%.1f,%.1f,%.1f,%.1f,%u,%u,%d,%d,%d,"
                  "%u,%u,%u,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%u,%u,%u,%u,%.3f,%.2f,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                  (unsigned)sample->time_ms,
                  (unsigned)sample->marker,
                  (unsigned)sample->task_id,
                  (unsigned)sample->task_state,
                  (unsigned)sample->profile,
                  (unsigned)sample->target_src,
                  (int)sample->ecod1_count,
                  (int)sample->ecod2_count,
                  (int)sample->ecod1_speed,
                  (int)sample->ecod2_speed,
                  (int)sample->ecod1_speed_filt,
                  (int)sample->ecod2_speed_filt,
                  (unsigned)nav->valid,
                  (unsigned)nav->gps_valid,
                  (unsigned)ins->valid,
                  (unsigned)ins->timestamp_ms,
                  (unsigned)ins->update_dt_ms,
                  (int)ins->encoder_delta_r,
                  (int)ins->encoder_delta_l,
                  (double)ins->encoder_delta_avg,
                  (double)ins->delta_distance_m,
                  (double)ins->count_to_meter,
                  (double)sample->imu_yaw,
                  (double)ins->yaw_deg,
                  (double)ins->yaw_used_deg,
                  (double)sample->gyro_z,
                  (double)ins->yaw_rate_dps,
                  (double)sample->imu_abs_yaw,
                  (unsigned)sample->imu_pit_count,
                  (unsigned)sample->imu_update_count,
                  (unsigned)sample->imu_update_age_ms,
                  (unsigned)sample->imu_read_fail_count,
                  (unsigned)sample->imu_dbg.receive_flag,
                  (unsigned)sample->imu_dbg.gyro_bias_ready,
                  (unsigned)sample->imu_dbg.gyro_bias_sample_count,
                  (double)sample->imu_dbg.sample_dt_s,
                  (double)sample->imu_dbg.gyro_z_dps,
                  (unsigned)sample->imu_yaw_freeze_ms,
                  (unsigned)sample->imu_yaw_freeze_suspect,
                  (double)ins->delta_x_m,
                  (double)ins->delta_y_m,
                  (double)ins->x_m,
                  (double)ins->y_m,
                  (double)ins->distance_m,
                  (double)ins->speed_mps,
                  (double)ins->encoder_speed_mps,
                  (double)ins->forward_accel_mps2,
                  (double)sample->imu_acc_x_g,
                  (double)sample->imu_acc_y_g,
                  (double)sample->imu_acc_z_g,
                  (unsigned)nav->path_valid,
                  (unsigned)nav->path_active,
                  (unsigned)nav->path_finished,
                  (unsigned)nav->path_mode,
                  (unsigned)nav->path_sample_count,
                  (unsigned)nav->path_replay_index,
                  (double)nav->path_sample_interval_m,
                  (double)nav->path_progress_m,
                  (double)nav->path_ref_x_m,
                  (double)nav->path_ref_y_m,
                  (double)nav->path_ref_yaw_deg,
                  (double)nav->path_ref_kappa_1pm,
                  (double)nav->path_ref_kappa_yaw_1pm,
                  (double)nav->path_ref_speed_mps,
                  (double)nav->path_yaw_error_deg,
                  (double)nav->path_lateral_error_m,
                  (unsigned)nav->path_last_sd_result,
                  (unsigned)nav->path_last_flash_result,
                  (unsigned)nav->path_last_flash_load_result,
                  (unsigned)sample->task2_path_dbg.valid,
                  (unsigned)sample->task2_path_dbg.motion,
                  (unsigned)sample->task2_path_dbg.route_cmd,
                  (unsigned)sample->task2_path_dbg.route_step_index,
                  (unsigned)sample->task2_path_dbg.route_step_count,
                  (unsigned)sample->task2_path_dbg.path_index,
                  (unsigned)sample->task2_path_dbg.path_count,
                  (unsigned)sample->task2_path_dbg.final_approach,
                  (unsigned)sample->task2_path_dbg.return_path,
                  (double)sample->task2_path_dbg.target_x_m,
                  (double)sample->task2_path_dbg.target_y_m,
                  (double)sample->task2_path_dbg.preview_x_m,
                  (double)sample->task2_path_dbg.preview_y_m,
                  (double)sample->task2_path_dbg.desired_yaw_deg,
                  (double)sample->task2_path_dbg.yaw_error_deg,
                  (double)sample->task2_path_dbg.lateral_error_m,
                  (double)sample->task2_path_dbg.projection_m,
                  (double)sample->task2_path_dbg.dist_to_target_m,
                  (double)sample->task2_path_dbg.dist_to_park_m,
                  (double)sample->task2_path_dbg.target_angle_deg,
                  (double)sample->task2_path_dbg.target_speed_mps,
                  (double)sample->steer_target_deg,
                  (double)sample->steer_feedback_deg,
                  (unsigned)sample->steer_valid,
                  (unsigned)sample->steer_sensor_fault,
                  (unsigned)sample->steer_stall_fault,
                  (double)sample->target_speed_mps,
                  (double)sample->pid_target_r,
                  (double)sample->pid_feedback_r,
                  (double)sample->pid_target_l,
                  (double)sample->pid_feedback_l,
                  (unsigned)sample->pid_dir_guard_r,
                  (unsigned)sample->pid_dir_guard_l,
                  (int)sample->duty_r,
                  (int)sample->duty_l,
                  (int)sample->duty_m,
                  (unsigned)sample->key_num,
                  (unsigned)sample->key_event_num,
                  (unsigned)sample->key_event_count,
                  (unsigned)sample->key_event_age_ms,
                  (unsigned)sample->ctrl_stop_reason,
                  (unsigned)sample->ctrl_safety_pause_active,
                  (unsigned)sample->ctrl_main_loop_age_ms,
                  (unsigned)sample->ctrl_safety_pause_until_ms,
                  (unsigned)sample->logger_dropped,
                  (double)sample->pid_error_r,
                  (double)sample->pid_error_l,
                  (unsigned)sample->ctrl_cmd.use_pid,
                  (unsigned)sample->ctrl_cmd.direct_pwm,
                  (unsigned)sample->ctrl_cmd.steer_closed_loop,
                  (unsigned)sample->ctrl_cmd.brake_to_stop,
                  (double)sample->ctrl_cmd.target_speed_mps,
                  (double)sample->ctrl_cmd.target_angle_deg,
                  (unsigned)sample->ctrl_cmd_age_ms,
                  (unsigned)sample->steer_feedback_age_ms,
                  (unsigned)sample->ctrl_pid_reset_reason,
                  (unsigned)sample->ctrl_pid_reset_age_ms,
                  (unsigned)sample->ctrl_pid_low_reason,
                  (unsigned)sample->ctrl_pid_low_age_ms,
                  (double)sample->ctrl_pid_low_target_r,
                  (double)sample->ctrl_pid_low_feedback_r,
                  (double)sample->ctrl_pid_low_output_r,
                  (double)sample->ctrl_pid_low_target_l,
                  (double)sample->ctrl_pid_low_feedback_l,
                  (double)sample->ctrl_pid_low_output_l);

    if ((len <= 0) || (len >= (int)sizeof(g_logger_line)))
    {
        printf("[LOGGER] task2 ins line format failed, len=%d\r\n", len);
        return 1U;
    }

    return data_logger_append(g_logger_line, (uint32)len);
}

static uint8 data_logger_format_sample(const data_logger_sample_t *sample)
{
    if (sample->profile == (uint8)DATA_LOGGER_PROFILE_GPS_TEST)
    {
        return data_logger_format_gps_test_sample(sample);
    }

    if (sample->profile == (uint8)DATA_LOGGER_PROFILE_IMU_TEST)
    {
        return data_logger_format_imu_test_sample(sample);
    }

    if (sample->profile == (uint8)DATA_LOGGER_PROFILE_TASK2_INS_DEBUG)
    {
        return data_logger_format_task2_ins_sample(sample);
    }

    if (sample->profile == (uint8)DATA_LOGGER_PROFILE_DRIVE_DEBUG)
    {
        return data_logger_format_drive_sample(sample);
    }

    return data_logger_format_full_sample(sample);
}

uint8 data_logger_init(void)
{
    return 0U;
}

uint8 data_logger_start(void)
{
    return data_logger_start_task(0U, DATA_LOGGER_PROFILE_FULL_NAV);
}

uint8 data_logger_start_task(uint8 task_id, data_logger_profile_t profile)
{
    return data_logger_start_task_src(task_id, profile, g_pid_target_src);
}

uint8 data_logger_start_task_src(uint8 task_id, data_logger_profile_t profile, uint8 target_src)
{
    if ((data_logger_camera_record_is_active() != 0U) ||
        (data_logger_camera_test_is_active() != 0U))
    {
        return 5U;
    }

    if (g_logger_state == DATA_LOGGER_STATE_STOPPING)
    {
        return data_logger_restart_task_src(task_id, profile, target_src);
    }

    if ((g_logger_state == DATA_LOGGER_STATE_STARTING) ||
        (g_logger_state == DATA_LOGGER_STATE_RUNNING))
    {
        return 0U;
    }

    data_logger_queue_reset();
    g_logger_last_error = 0U;
    if (task_id >= DATA_LOGGER_TASK_COUNT)
    {
        task_id = 0U;
    }
    if (target_src >= DATA_LOGGER_TARGET_SRC_COUNT)
    {
        target_src = TARGET_SRC_TASK;
    }
    g_logger_task_id = task_id;
    g_logger_profile = (uint8)profile;
    g_logger_target_src = target_src;
    g_logger_restart_pending = 0U;
    data_logger_memory_sync();
    g_logger_state = DATA_LOGGER_STATE_STARTING;
    printf("[LOGGER] start requested: task=%u profile=%u src=%u, CPU0 samples, CPU1 writes SD.\r\n",
           (unsigned)g_logger_task_id,
           (unsigned)g_logger_profile,
           (unsigned)g_logger_target_src);
    return 0U;
}

uint8 data_logger_restart_task_src(uint8 task_id, data_logger_profile_t profile, uint8 target_src)
{
    if (task_id >= DATA_LOGGER_TASK_COUNT)
    {
        task_id = 0U;
    }
    if (target_src >= DATA_LOGGER_TARGET_SRC_COUNT)
    {
        target_src = TARGET_SRC_TASK;
    }

    if ((g_logger_state == DATA_LOGGER_STATE_IDLE) ||
        (g_logger_state == DATA_LOGGER_STATE_FAILED))
    {
        return data_logger_start_task_src(task_id, profile, target_src);
    }

    if ((g_logger_task_id == task_id) &&
        (g_logger_profile == (uint8)profile) &&
        (g_logger_target_src == target_src) &&
        (g_logger_restart_pending == 0U) &&
        ((g_logger_state == DATA_LOGGER_STATE_STARTING) ||
         (g_logger_state == DATA_LOGGER_STATE_RUNNING)))
    {
        if (profile != DATA_LOGGER_PROFILE_GPS_TEST)
        {
            return 0U;
        }
    }

    g_logger_restart_task_id = task_id;
    g_logger_restart_profile = (uint8)profile;
    g_logger_restart_target_src = target_src;
    g_logger_restart_pending = 1U;
    data_logger_memory_sync();

    if ((g_logger_state == DATA_LOGGER_STATE_STARTING) ||
        (g_logger_state == DATA_LOGGER_STATE_RUNNING))
    {
        data_logger_stop();
    }

    printf("[LOGGER] restart requested: task=%u profile=%u src=%u state=%u\r\n",
           (unsigned)task_id,
           (unsigned)profile,
           (unsigned)target_src,
           (unsigned)g_logger_state);
    return 0U;
}

uint8 data_logger_force_restart_task_src(uint8 task_id, data_logger_profile_t profile, uint8 target_src)
{
    if (task_id >= DATA_LOGGER_TASK_COUNT)
    {
        task_id = 0U;
    }
    if (target_src >= DATA_LOGGER_TARGET_SRC_COUNT)
    {
        target_src = TARGET_SRC_TASK;
    }

    if ((g_logger_state == DATA_LOGGER_STATE_IDLE) ||
        (g_logger_state == DATA_LOGGER_STATE_FAILED))
    {
        return data_logger_start_task_src(task_id, profile, target_src);
    }

    g_logger_restart_task_id = task_id;
    g_logger_restart_profile = (uint8)profile;
    g_logger_restart_target_src = target_src;
    g_logger_restart_pending = 1U;
    data_logger_memory_sync();
    data_logger_stop();

    printf("[LOGGER] force restart requested: task=%u profile=%u src=%u state=%u\r\n",
           (unsigned)task_id,
           (unsigned)profile,
           (unsigned)target_src,
           (unsigned)g_logger_state);
    return 0U;
}

void data_logger_stop(void)
{
    if ((g_logger_state == DATA_LOGGER_STATE_STARTING) ||
        (g_logger_state == DATA_LOGGER_STATE_RUNNING))
    {
        data_logger_sample_t sample;
        data_logger_capture_sample(system_getval_ms(), &sample, NULL);
        (void)data_logger_queue_push(&sample);
        data_logger_memory_sync();
        g_logger_state = DATA_LOGGER_STATE_STOPPING;
    }
}

void data_logger_stop_cancel_restart(void)
{
    uint8 state = g_logger_state;
    uint8 cancel_restart = 1U;

    if ((g_logger_restart_pending != 0U) &&
        ((g_logger_restart_task_id != g_logger_task_id) ||
         (g_logger_restart_profile != g_logger_profile) ||
         (g_logger_restart_target_src != g_logger_target_src)))
    {
        cancel_restart = 0U;
    }
    if (cancel_restart != 0U)
    {
        g_logger_restart_pending = 0U;
    }
    data_logger_memory_sync();
    data_logger_stop();
    if ((state != DATA_LOGGER_STATE_STARTING) &&
        (state != DATA_LOGGER_STATE_RUNNING))
    {
        g_logger_marker = 0U;
        data_logger_memory_sync();
    }
}

void data_logger_mark(uint16 marker)
{
    g_logger_marker = marker;
    data_logger_memory_sync();
}

void data_logger_mark_now(uint16 marker)
{
    uint8 state = g_logger_state;

    g_logger_marker = marker;
    data_logger_memory_sync();

    if ((state == DATA_LOGGER_STATE_STARTING) ||
        (state == DATA_LOGGER_STATE_RUNNING))
    {
        data_logger_sample_t sample;

        data_logger_capture_sample(system_getval_ms(), &sample, NULL);
        (void)data_logger_queue_push(&sample);
        data_logger_memory_sync();
    }
}

void data_logger_gps_test_mark_point(uint16 point_id)
{
    data_logger_gps_test_mark_point_sample(point_id, NULL);
}

void data_logger_gps_test_mark_point_sample(uint16 point_id, const nav_gps_sample_t *gps_sample)
{
    uint8 state = g_logger_state;

    if (((state == DATA_LOGGER_STATE_STARTING) ||
         (state == DATA_LOGGER_STATE_RUNNING)) &&
        (g_logger_task_id == 1U) &&
        (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_GPS_TEST))
    {
        data_logger_sample_t sample;

        g_logger_marker = 4204U;
        data_logger_capture_sample(system_getval_ms(), &sample, gps_sample);
        sample.gps_test_event = 1U;
        sample.gps_test_point_id = point_id;
        (void)data_logger_queue_push(&sample);
        data_logger_memory_sync();
    }
}

void data_logger_task(uint32 now_ms)
{
    data_logger_sample_t sample;
    uint8 state = g_logger_state;

    if ((state != DATA_LOGGER_STATE_STARTING) &&
        (state != DATA_LOGGER_STATE_RUNNING))
    {
        return;
    }

    {
        uint32 sample_period_ms;

        if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_DRIVE_DEBUG)
        {
            sample_period_ms = DATA_LOGGER_DRIVE_SAMPLE_PERIOD_MS;
        }
        else if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_GPS_TEST)
        {
            sample_period_ms = DATA_LOGGER_GPS_TEST_SAMPLE_PERIOD_MS;
        }
        else if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_IMU_TEST)
        {
            sample_period_ms = DATA_LOGGER_IMU_TEST_SAMPLE_PERIOD_MS;
        }
        else if (g_logger_profile == (uint8)DATA_LOGGER_PROFILE_TASK2_INS_DEBUG)
        {
            sample_period_ms = DATA_LOGGER_TASK2_INS_SAMPLE_PERIOD_MS;
        }
        else
        {
            sample_period_ms = DATA_LOGGER_FULL_SAMPLE_PERIOD_MS;
        }

        if ((g_logger_last_sample_ms != 0U) &&
            ((now_ms - g_logger_last_sample_ms) < sample_period_ms))
        {
            return;
        }
    }
    g_logger_last_sample_ms = now_ms;

    data_logger_capture_sample(now_ms, &sample, NULL);
    (void)data_logger_queue_push(&sample);
}

void data_logger_worker_task(uint32 now_ms)
{
    data_logger_sample_t sample;
    uint8 state = g_logger_state;

    if ((data_logger_camera_snapshot_is_busy() != 0U) ||
        (data_logger_camera_record_is_active() != 0U) ||
        ((data_logger_camera_test_is_active() != 0U) &&
         (g_camera_test_allow_logger == 0U)))
    {
        return;
    }

    if (state == DATA_LOGGER_STATE_STARTING)
    {
        uint8 ret = data_logger_backend_start(now_ms);
        if (ret != 0U)
        {
            data_logger_backend_fail(ret);
            return;
        }
        data_logger_memory_sync();
        g_logger_state = DATA_LOGGER_STATE_RUNNING;
        state = DATA_LOGGER_STATE_RUNNING;
    }

    if ((state == DATA_LOGGER_STATE_RUNNING) ||
        (state == DATA_LOGGER_STATE_STOPPING))
    {
        while (data_logger_queue_pop(&sample) == 0U)
        {
            if (data_logger_format_sample(&sample) != 0U)
            {
                data_logger_backend_fail(20U);
                return;
            }
        }

        if ((g_logger_last_sync_ms == 0U) ||
            ((now_ms - g_logger_last_sync_ms) >= DATA_LOGGER_SYNC_PERIOD_MS))
        {
            g_logger_last_sync_ms = now_ms;
            if (g_logger_file_open != 0U)
            {
                if (data_logger_flush() != 0U)
                {
                    data_logger_backend_fail(21U);
                    return;
                }
                if (f_sync(&g_logger_file) != FR_OK)
                {
                    data_logger_backend_fail(22U);
                    return;
                }
            }
        }
    }

    if (state == DATA_LOGGER_STATE_STOPPING)
    {
        data_logger_backend_close();
        printf("[LOGGER] stop on CPU1: %s\r\n", g_logger_filename);
        if (g_logger_restart_pending != 0U)
        {
            data_logger_queue_reset();
            g_logger_last_error = 0U;
            g_logger_task_id = g_logger_restart_task_id;
            g_logger_profile = g_logger_restart_profile;
            g_logger_target_src = g_logger_restart_target_src;
            g_logger_restart_pending = 0U;
            data_logger_memory_sync();
            g_logger_state = DATA_LOGGER_STATE_STARTING;
            printf("[LOGGER] restart on CPU1: task=%u profile=%u src=%u\r\n",
                   (unsigned)g_logger_task_id,
                   (unsigned)g_logger_profile,
                   (unsigned)g_logger_target_src);
        }
        else
        {
            data_logger_memory_sync();
            g_logger_state = DATA_LOGGER_STATE_IDLE;
        }
    }
}

uint8 data_logger_is_running(void)
{
    return (g_logger_state == DATA_LOGGER_STATE_RUNNING) ? 1U : 0U;
}

uint8 data_logger_is_busy(void)
{
    uint8 state = g_logger_state;

    return ((state == DATA_LOGGER_STATE_STARTING) ||
            (state == DATA_LOGGER_STATE_RUNNING) ||
            (state == DATA_LOGGER_STATE_STOPPING) ||
            (data_logger_camera_snapshot_is_busy() != 0U) ||
            (data_logger_camera_record_is_active() != 0U) ||
            (data_logger_camera_test_is_active() != 0U)) ? 1U : 0U;
}

uint8 data_logger_get_state(void)
{
    return g_logger_state;
}

uint8 data_logger_get_last_error(void)
{
    return g_logger_last_error;
}

uint8 data_logger_get_profile(void)
{
    return g_logger_profile;
}

uint8 data_logger_get_task_id(void)
{
    return g_logger_task_id;
}

uint8 data_logger_get_target_src(void)
{
    return g_logger_target_src;
}

uint32 data_logger_get_dropped_count(void)
{
    return g_logger_dropped_count;
}

const char *data_logger_get_filename(void)
{
    return g_logger_filename;
}
