/*
 * UI Key Table (same for Task0..Task4)
 * +------+----------------------+------------------------------+------------------------------+---------------------------------------+----------------------------------+
 * | Key  | Page1 InfoPage       | Page2 SetPage                | Page3 ParamPage              | Page4 GPSPage                        | Page5 GPS_SetPage                |
 * +------+----------------------+------------------------------+------------------------------+---------------------------------------+----------------------------------+
 * | K1   | Task--               | UI_KeyChange_Callback(0)     | UI_KeyChange_Callback(0)     | Start GPS point picking              | UI_Image_Callback(0)             |
 * | K2   | Task++               | UI_KeyChange_Callback(1)     | UI_KeyChange_Callback(1)     | Cancel picking / remove last point   | UI_Image_Callback(1)             |
 * | K3   | Mode: TASK/PEDAL/    | program_select++ (1..2 loop) | No action                    | No action                            | UI_Image_SelectNext()            |
 * |      | REMOTE/PWMTEST/SCAN  |                              |                              |                                       |                                  |
 * | K4   | Task start/stop      | program_select-- (1..2 loop) | No action                    | No action                            | No action                        |
 * |      | only in TASK mode    |                              |                              |                                       |                                  |
 * | K5   | Next page            | Next page                    | Next page                    | Next page                            | Next page                        |
 * +------+----------------------+------------------------------+------------------------------+---------------------------------------+----------------------------------+
 *
 * Notes:
 * - Page4 K1/K2 edit GPS points, except Task3 Page4.
 * - Task1 Page2 is path record, Task1 Page3 is IMU test.
 * - Task2 Page2 is INS map; Task2 Page3 is point/debug table.
 * - Task3 Page4 is SCC8660 color camera preview.
 * - Page5 is camera preview for Task0, GPS test page for Task1,
 *   and GPS point edit/map page for Task2..Task4.
 */
#include "UI.h"
#include "math.h"
#include "ui_task.h"
#include "taban/taban.h"
#include "yaokong.h"
#include "dici.h"
#include "isr.h"
#include "task_ctrl.h"
#include "nav_gps.h"
#include "nav_ins.h"
#include "nav_control.h"
#include "nav_path.h"
#include "data_logger.h"
#include "blue_target.h"
#include "guimai/guimai_board.h"
#include "tld7002_project_config.h"

extern volatile int16 g_ecod1_count;
extern volatile int16 g_ecod2_count;
extern volatile int16 g_ecod1_speed;
extern volatile int16 g_ecod2_speed;
extern volatile int16 g_abs_encoder_raw;
extern volatile uint8 g_imu963ra_init_ret;
extern uint8 Key_Task2DebugGetGoCmd(void);
extern const char *Key_Task2DebugGetGoName(void);
extern uint8 Key_Task2DebugGetRetCmd(void);
extern const char *Key_Task2DebugGetRetName(void);
extern uint8 Key_Task2DebugGetActionCmd(void);
extern const char *Key_Task2DebugGetActionName(void);
extern const char *Key_Task2DebugGetLastName(void);
extern uint8 Key_Task2DebugIsBusy(void);

#define UI_ENCODER_WHEEL_CIRCUMFERENCE_M   (0.200f)
#define UI_ENCODER_PULSE_PER_REV            (360.0f)
#define UI_ENCODER_COUNTPS_TO_MPS           (UI_ENCODER_WHEEL_CIRCUMFERENCE_M / UI_ENCODER_PULSE_PER_REV)

#define UI_TASK3_CAMERA_MODE_PICTURE         (0U)
#define UI_TASK3_CAMERA_MODE_RECORD          (1U)
#define UI_TASK3_CAMERA_STATS_PERIOD_MS      (200U)

UI_Page Page_Num = InfoPage;
uint8 UI_Mode = 0;
bool Gui_Refersh_Bool = ZF_FALSE;
uint8 g_ui_task_id = UI_TASK_DEFAULT_ID;
ui_task_state_t g_ui_task_state[UI_TASK_COUNT];
uint8 Camera_compare = 200;

extern uint8 program_select;
static uint16 camera_exposure = 1000;
static uint8 ui_inited = 0;
static uint8 ui_camera_init_done = 0;
static uint8 ui_camera_init_ok = 0;
static uint8 ui_camera_page_entered = 0;
static uint16 ui_camera_last_exposure = 0;
static uint8 ui_color_camera_init_ok = 0;
static uint8 ui_color_camera_page_entered = 0;
static uint32 ui_color_camera_fps = 0U;
static uint32 ui_color_camera_fps_last_ms = 0U;
static uint32 ui_color_camera_fps_frame_base = 0U;
static uint32 ui_color_camera_vsync_fps = 0U;
static uint32 ui_color_camera_vsync_frame_base = 0U;
static uint32 ui_color_camera_dma_fps = 0U;
static uint32 ui_color_camera_dma_frame_base = 0U;
static uint32 ui_color_camera_preview_frame_count = 0U;
static uint8 ui_color_camera_snapshot_size = DATA_LOGGER_CAMERA_SNAPSHOT_FULL;
static uint8 ui_color_camera_capture_mode = UI_TASK3_CAMERA_MODE_RECORD;
static uint8 ui_color_camera_snapshot_request = 0U;
static uint8 ui_color_camera_action_last_result = 0U;
static uint32 ui_color_camera_record_last_request_ms = 0U;
static uint32 ui_color_camera_record_fps = 0U;
static uint32 ui_color_camera_record_fps_last_ms = 0U;
static uint32 ui_color_camera_record_fps_frame_base = 0U;
static uint32 ui_color_camera_stats_last_ms = 0U;
static UI_Page ui_color_camera_last_page = InfoPage;
static uint8 ui_task3_screen_frozen = 0U;
static uint8 ui_task3_camera_test_have_previous = 0U;
static uint8 ui_task3_camera_test_previous_valid = 0U;
static uint16 ui_task3_camera_test_previous_center_x = 0U;
static uint16 ui_task3_camera_test_previous_bbox_height = 0U;
static uint16 ui_task3_camera_test_previous_distance_cm = 0U;
static uint8 ui_task3_camera_test_invalid_frames = 0U;
static uint32 ui_task3_camera_test_last_event_ms = 0U;
static uint8 ui_task3_camera_test_manual_request = 0U;
static uint8 ui_task3_camera_test_action_result = 0U;
static uint8 ui_task3_camera_test_page_entered = 0U;
static uint8 ui_map_drawn = 0U;
static UI_Page ui_map_last_page = InfoPage;
static uint8 ui_map_last_task = 0xFFU;
static uint8 ui_map_last_point_count = 0xFFU;

#define UI_INS_TRACE_MAX          (512U)
#define UI_INS_TRACE_MIN_STEP_M   (0.05f)
#define UI_INS_TRACE_X0           (0U)
#define UI_INS_TRACE_Y0           (160U)
#define UI_INS_TRACE_W            (240U)
#define UI_INS_TRACE_H            (156U)

#define UI_TASK2_INS_TRACE_MAX       (512U)
#define UI_TASK2_INS_TRACE_STEP_M    (0.10f)
#define UI_TASK2_MAP_X0              (0U)
#define UI_TASK2_MAP_Y0              (64U)
#define UI_TASK2_MAP_W               (240U)
#define UI_TASK2_MAP_H               (252U)
#define UI_TASK2_MAP_ORIGIN_X        (17)
#define UI_TASK2_MAP_ORIGIN_Y        (302)
#define UI_TASK2_MAP_PX_PER_M        (5.5f)
#define UI_TASK2_MAP_SCALE_BAR_M     (3U)
#define UI_TASK2_MAP_TRACE_DRAW_MS   (200U)
#define UI_TASK2_POINT_COUNT         (8U)
#define UI_LOGGER_STATE_IDLE         (0U)
#define UI_LOGGER_STATE_STARTING     (1U)
#define UI_LOGGER_STATE_RUNNING      (2U)
#define UI_LOGGER_STATE_STOPPING     (3U)
#define UI_LOGGER_STATE_FAILED       (4U)
#define UI_TASK1_IMU_LOG_RETRY_MS    (10000U)
#define UI_TASK2_INS_LOG_RETRY_MS    (10000U)

static float ui_path_trace_x[UI_INS_TRACE_MAX];
static float ui_path_trace_y[UI_INS_TRACE_MAX];
static uint16 ui_path_trace_count = 0U;
static uint16 ui_path_trace_last_count = 0xFFFFU;
static uint8 ui_path_trace_drawn = 0U;
static uint32 ui_path_trace_last_draw_ms = 0U;

static float ui_task2_ins_trace_x[UI_TASK2_INS_TRACE_MAX];
static float ui_task2_ins_trace_y[UI_TASK2_INS_TRACE_MAX];
static uint16 ui_task2_ins_trace_count = 0U;
static uint8 ui_task2_ins_trace_started = 0U;
static uint16 ui_task2_ins_trace_drawn_count = 0U;
static uint8 ui_task2_ins_map_drawn = 0U;
static uint8 ui_task2_ins_last_running = 0U;
static uint32 ui_task2_ins_last_draw_ms = 0U;
static uint32 ui_task2_ins_logger_last_try_ms = 0U;
static uint8 ui_task1_imu_logger_owned = 0U;
static uint32 ui_task1_imu_logger_last_try_ms = 0U;

static uint8 ui_gps_test_active = 0U;
static uint8 ui_gps_test_first_valid = 0U;
static uint32 ui_gps_test_count = 0U;
static uint32 ui_gps_test_last_sample_ms = 0U;
static float ui_gps_test_first_x_m = 0.0f;
static float ui_gps_test_first_y_m = 0.0f;
static float ui_gps_test_sum_x_m = 0.0f;
static float ui_gps_test_sum_y_m = 0.0f;
static float ui_gps_test_sum_err2_m = 0.0f;
static float ui_gps_test_last_err_m = 0.0f;
static float ui_gps_test_max_err_m = 0.0f;
static uint16 ui_gps_test_point_id = 0U;
#define UI_GPS_TEST_PICK_TARGET   (10U)
static uint8 ui_gps_test_pick_active = 0U;
static uint8 ui_gps_test_pick_count = 0U;
static uint32 ui_gps_test_pick_last_ms = 0U;
static uint16 ui_gps_test_pick_point_id = 0U;
static nav_gps_average_t ui_gps_test_pick_avg;

typedef struct
{
    const char *name;
    float x_m;
    float y_m;
    uint8 valid;
} ui_task2_point_t;

static const ui_task2_point_t ui_task2_points[UI_TASK2_POINT_COUNT] =
{
    {"O",   0.0f,  0.0f, 1U},
    {"G1L", -1.0f, 4.5f, 1U},
    {"G1",  -1.63f, 4.5f, 1U},
    {"G2",  0.17f, 4.5f, 1U},
    {"G3",  1.87f, 4.5f, 1U},
    {"G3R", 7.0f,  4.5f, 1U},
    {"A",   0.10f, 9.25f, 1U},
    {"P",   10.0f, 0.6f, 1U},
};
typedef struct
{
    const char *set_title;
    const char *set_item1;
    const char *set_item2;
    const char *param_title;
    const char *param_item1;
    const char *param_item2;
    const char *gps_title;
    const char *gps_edit_title;
} ui_task_display_cfg_t;

static const ui_task_display_cfg_t g_ui_task_cfg[UI_TASK_COUNT] =
{
    {"T0-Set", "select", "value1", "T0-Param", "value1", "value2", "T0-GPS", "T0-GPS-Edit"},
    {"T1-Set", "select", "value1", "T1-Param", "value1", "value2", "T1-GPS", "T1-GPS-Edit"},
    {"T2-Set", "select", "value1", "T2-Param", "value1", "value2", "T2-GPS", "T2-GPS-Edit"},
    {"T3-Set", "select", "value1", "T3-Param", "value1", "value2", "T3-GPS", "T3-GPS-Edit"},
    {"T4-Set", "select", "value1", "T4-Param", "value1", "value2", "T4-GPS", "T4-GPS-Edit"}
};

static void UI_PathRecordPage(void);
static void UI_Task1ImuPage(void);
static void UI_Task2PointPage(void);
static void UI_Task3ColorCameraPage(void);

static const ui_task_display_cfg_t *UI_GetDisplayCfg(void)
{
    return &g_ui_task_cfg[ui_task_to_index(ui_task_get())];
}

static void UI_Task1RequestSdDir(void)
{
    nav_path_request_task1_sd_dir();
}

static uint8 UI_Task1CompareShouldEnable(void)
{
    if (ui_task_get() != 1U)
    {
        return 0U;
    }

#if IMU_TASK1_COMPARE_ENABLE_IN_RUN
    if ((g_task_ctrl_id == 1U) &&
        (g_task_ctrl_state == TASK_CTRL_STATE_RUNNING))
    {
        return 1U;
    }
#endif

#if IMU_TASK1_COMPARE_ENABLE_IN_IMU_PAGE
    if (Page_Num == ParamPage)
    {
        return 1U;
    }
#endif

    return 0U;
}

static void UI_Task1CompareUpdate(void)
{
    IMU_AHRS_SetCompareEnable(UI_Task1CompareShouldEnable());
}

static uint8 UI_Task1ImuLoggerIsCurrent(void)
{
    return ((data_logger_get_task_id() == 1U) &&
            (data_logger_get_profile() == (uint8)DATA_LOGGER_PROFILE_IMU_TEST)) ? 1U : 0U;
}

static void UI_Task1ImuLoggerEnter(uint32 now_ms)
{
    uint8 state = data_logger_get_state();

    UI_Task1RequestSdDir();
    if ((g_task_ctrl_id == 1U) &&
        (g_task_ctrl_state == TASK_CTRL_STATE_RUNNING))
    {
        return;
    }

    if ((UI_Task1ImuLoggerIsCurrent() != 0U) &&
        ((state == UI_LOGGER_STATE_STARTING) ||
         (state == UI_LOGGER_STATE_RUNNING) ||
         (state == UI_LOGGER_STATE_STOPPING)))
    {
        ui_task1_imu_logger_owned = 1U;
        return;
    }

    if ((state != UI_LOGGER_STATE_IDLE) && (state != UI_LOGGER_STATE_FAILED))
    {
        return;
    }

    if ((ui_task1_imu_logger_last_try_ms != 0U) &&
        ((now_ms - ui_task1_imu_logger_last_try_ms) < UI_TASK1_IMU_LOG_RETRY_MS))
    {
        return;
    }

    ui_task1_imu_logger_last_try_ms = now_ms;
    if (data_logger_start_task_src(1U, DATA_LOGGER_PROFILE_IMU_TEST, TARGET_SRC_TASK) == 0U)
    {
        data_logger_mark(4301U);
        ui_task1_imu_logger_owned = 1U;
    }
}

static void UI_Task1ImuLoggerLeaveIfNeeded(void)
{
    if (ui_task1_imu_logger_owned == 0U)
    {
        return;
    }

    if (UI_Task1ImuLoggerIsCurrent() != 0U)
    {
        data_logger_mark(4302U);
        data_logger_stop_cancel_restart();
    }
    ui_task1_imu_logger_owned = 0U;
}

static uint8 UI_Task2InsLoggerIsCurrent(void)
{
    return ((data_logger_get_task_id() == 2U) &&
            (data_logger_get_profile() == (uint8)DATA_LOGGER_PROFILE_TASK2_INS_DEBUG)) ? 1U : 0U;
}

static void UI_Task2InsLoggerEnter(uint32 now_ms, const nav_ins_state_t *ins)
{
    uint8 state = data_logger_get_state();

    if ((ins == NULL) || (ins->valid == 0U))
    {
        return;
    }

    if ((UI_Task2InsLoggerIsCurrent() != 0U) &&
        ((state == UI_LOGGER_STATE_STARTING) ||
         (state == UI_LOGGER_STATE_RUNNING) ||
         (state == UI_LOGGER_STATE_STOPPING)))
    {
        return;
    }

    if ((state != UI_LOGGER_STATE_IDLE) && (state != UI_LOGGER_STATE_FAILED))
    {
        return;
    }

    if ((ui_task2_ins_logger_last_try_ms != 0U) &&
        ((now_ms - ui_task2_ins_logger_last_try_ms) < UI_TASK2_INS_LOG_RETRY_MS))
    {
        return;
    }

    ui_task2_ins_logger_last_try_ms = now_ms;
    if (data_logger_start_task_src(2U, DATA_LOGGER_PROFILE_TASK2_INS_DEBUG, g_pid_target_src) == 0U)
    {
        data_logger_mark(4401U);
    }
}

static const char *UI_TargetSrcName(uint8 target_src)
{
    switch (target_src)
    {
        case TARGET_SRC_TASK:
            return "TASK";
        case TARGET_SRC_PEDAL:
            return "PEDAL";
        case TARGET_SRC_REMOTE:
            return "REMOTE";
        case TARGET_SRC_PWM_TEST:
            return "PWMTEST";
        case TARGET_SRC_STEER_SCAN:
            return "SCAN";
        default:
            return "UNKNOWN";
    }
}

static const char *UI_SteerScanPhaseName(uint8 phase)
{
    switch (phase)
    {
        case CTRL_STEER_SCAN_PHASE_LEFT:
            return "LEFT";
        case CTRL_STEER_SCAN_PHASE_WAIT:
            return "WAIT";
        case CTRL_STEER_SCAN_PHASE_RIGHT:
            return "RIGHT";
        case CTRL_STEER_SCAN_PHASE_RETURN:
            return "BACK";
        case CTRL_STEER_SCAN_PHASE_DONE:
            return "DONE";
        case CTRL_STEER_SCAN_PHASE_SENSOR_ERR:
            return "SENS";
        case CTRL_STEER_SCAN_PHASE_TIMEOUT:
            return "TIME";
        default:
            return "IDLE";
    }
}
static const char *UI_TaskStateName(uint8 state)
{
    switch (state)
    {
        case TASK_CTRL_STATE_IDLE:
            return "IDLE";
        case TASK_CTRL_STATE_RUNNING:
            return "RUN";
        case TASK_CTRL_STATE_FINISHED:
            return "DONE";
        default:
            return "UNKNOWN";
    }
}

static void UI_Task2InfoPage(void)
{
    ctrl_task2_status_t status;
    nav_ins_state_t ins_state;
    float display_distance_m;
    uint8 ref_ready;
    uint8 step_display = 0U;

    ctrl_task2_get_status(&status);
    nav_ins_get_state(&ins_state);
    display_distance_m = ins_state.distance_m;
    if (display_distance_m < 0.0f)
    {
        display_distance_m = 0.0f;
    }
    if (display_distance_m > 999.9f)
    {
        display_distance_m = 999.9f;
    }
    ref_ready = IMU_AHRS_IsReferenceReady();
    if (status.route_step_count != 0U)
    {
        step_display = (uint8)(status.route_step_index + 1U);
    }

    ips200_show_string(0, 16 * 0, "Task2 Light");
    ips200_show_string(152, 16 * 0, "Pg");
    ips200_show_uint(184, 16 * 0, (uint32)Page_Num, 1);

    ips200_show_string(0, 16 * 1, "Mode");
    ips200_show_string(64, 16 * 1, UI_TargetSrcName(g_pid_target_src));
    ips200_show_string(144, 16 * 1, "State");
    ips200_show_string(192, 16 * 1, UI_TaskStateName(g_task_ctrl_state));

    ips200_show_string(0, 16 * 2, "Yaw");
    ips200_show_float(56, 16 * 2, IMU_RelYaw, 3, 1);
    ips200_show_string(144, 16 * 2, "INS");
    ips200_show_uint(192, 16 * 2, ins_state.valid, 1);

    ips200_show_string(0, 16 * 3, "Ref");
    ips200_show_uint(56, 16 * 3, ref_ready, 1);
    ips200_show_string(112, 16 * 3, "Cmd");
    ips200_show_uint(160, 16 * 3, status.last_cmd, 3);

    ips200_show_string(0, 16 * 4, "WiFi");
    ips200_show_string(64, 16 * 4, guimai_board_wifi_status_text());
    ips200_show_string(0, 16 * 5, "Voice");
    ips200_show_string(64, 16 * 5, guimai_board_record_status_text());

    ips200_show_string(0, 16 * 6, "Mot");
    ips200_show_string(56, 16 * 6, status.motion_name);
    ips200_show_string(112, 16 * 6, "Stop");
    ips200_show_string(160, 16 * 6, status.stop_reason);

    ips200_show_string(0, 16 * 7, "Prog");
    ips200_show_float(56, 16 * 7, status.progress, 3, 1);
    ips200_show_string(144, 16 * 7, "Tar");
    ips200_show_float(184, 16 * 7, status.target, 3, 1);

    ips200_show_string(0, 16 * 8, "Route");
    ips200_show_uint(56, 16 * 8, status.route_cmd, 3);
    ips200_show_string(112, 16 * 8, "Step");
    ips200_show_uint(160, 16 * 8, step_display, 2);
    ips200_show_string(184, 16 * 8, "/");
    ips200_show_uint(200, 16 * 8, status.route_step_count, 2);

    ips200_show_string(0, 16 * 9, "D");
    ips200_show_float(24, 16 * 9, display_distance_m, 3, 1);
    ips200_show_string(88, 16 * 9, "m");
    ips200_show_string(128, 16 * 9, "K3 rec");
}

static float UI_EncoderSpeedToMps(int16 encoder_speed_countps)
{
    return ((float)encoder_speed_countps) * UI_ENCODER_COUNTPS_TO_MPS;
}

static float UI_Task2ClipF32(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }
    return value;
}

static float UI_Wrap180Deg(float angle_deg)
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

static int32 UI_Task2RoundToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int32)(value + 0.5f);
    }
    return (int32)(value - 0.5f);
}


static uint16 UI_Task2ClampU16(int32 value, int32 min_value, int32 max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }
    return (uint16)value;
}

static void UI_Task2InsTraceClear(void)
{
    ui_task2_ins_trace_count = 0U;
    ui_task2_ins_trace_drawn_count = 0U;
    ui_task2_ins_trace_started = 0U;
    ui_task2_ins_map_drawn = 0U;
}

static void UI_Task2InsTraceAdd(const nav_ins_state_t *ins)
{
    float dx;
    float dy;
    uint16 i;

    if ((ins == NULL) || (ins->valid == 0U))
    {
        return;
    }

    if ((ui_task2_ins_trace_started == 0U) || (ui_task2_ins_trace_count == 0U))
    {
        ui_task2_ins_trace_x[0] = ins->x_m;
        ui_task2_ins_trace_y[0] = ins->y_m;
        ui_task2_ins_trace_count = 1U;
        ui_task2_ins_trace_started = 1U;
        return;
    }

    dx = ins->x_m - ui_task2_ins_trace_x[ui_task2_ins_trace_count - 1U];
    dy = ins->y_m - ui_task2_ins_trace_y[ui_task2_ins_trace_count - 1U];
    if ((dx * dx + dy * dy) < (UI_TASK2_INS_TRACE_STEP_M * UI_TASK2_INS_TRACE_STEP_M))
    {
        return;
    }

    if (ui_task2_ins_trace_count >= UI_TASK2_INS_TRACE_MAX)
    {
        for (i = 1U; i < ui_task2_ins_trace_count; i++)
        {
            ui_task2_ins_trace_x[i - 1U] = ui_task2_ins_trace_x[i];
            ui_task2_ins_trace_y[i - 1U] = ui_task2_ins_trace_y[i];
        }
        ui_task2_ins_trace_count--;
        if (ui_task2_ins_trace_drawn_count > 0U)
        {
            ui_task2_ins_trace_drawn_count--;
        }
        ui_task2_ins_map_drawn = 0U;
    }

    ui_task2_ins_trace_x[ui_task2_ins_trace_count] = ins->x_m;
    ui_task2_ins_trace_y[ui_task2_ins_trace_count] = ins->y_m;
    ui_task2_ins_trace_count++;
}

static void UI_Task2MapClearArea(void)
{
    uint16 y;
    uint16 x0 = UI_TASK2_MAP_X0;
    uint16 y0 = UI_TASK2_MAP_Y0;
    uint16 x1 = (uint16)(UI_TASK2_MAP_X0 + UI_TASK2_MAP_W - 1U);
    uint16 y1 = (uint16)(UI_TASK2_MAP_Y0 + UI_TASK2_MAP_H - 1U);

    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, RGB565_WHITE);
    }

    ips200_draw_line(x0, y0, x1, y0, RGB565_GRAY);
    ips200_draw_line(x0, y1, x1, y1, RGB565_GRAY);
    ips200_draw_line(x0, y0, x0, y1, RGB565_GRAY);
    ips200_draw_line(x1, y0, x1, y1, RGB565_GRAY);
}

static void UI_Task2MapToPixel(float x_m, float y_m, uint16 *pixel_x, uint16 *pixel_y)
{
    int32 x;
    int32 y;

    x = UI_Task2RoundToInt((float)UI_TASK2_MAP_ORIGIN_X + x_m * UI_TASK2_MAP_PX_PER_M);
    y = UI_Task2RoundToInt((float)UI_TASK2_MAP_ORIGIN_Y - y_m * UI_TASK2_MAP_PX_PER_M);

    *pixel_x = UI_Task2ClampU16(x,
                                (int32)(UI_TASK2_MAP_X0 + 1U),
                                (int32)(UI_TASK2_MAP_X0 + UI_TASK2_MAP_W - 2U));
    *pixel_y = UI_Task2ClampU16(y,
                                (int32)(UI_TASK2_MAP_Y0 + 1U),
                                (int32)(UI_TASK2_MAP_Y0 + UI_TASK2_MAP_H - 2U));
}

static void UI_Task2MapDrawCross(uint16 x, uint16 y, uint16 color)
{
    uint16 x_min = (uint16)(UI_TASK2_MAP_X0 + 1U);
    uint16 x_max = (uint16)(UI_TASK2_MAP_X0 + UI_TASK2_MAP_W - 2U);
    uint16 y_min = (uint16)(UI_TASK2_MAP_Y0 + 1U);
    uint16 y_max = (uint16)(UI_TASK2_MAP_Y0 + UI_TASK2_MAP_H - 2U);

    ips200_draw_point(x, y, color);
    if (x > x_min) ips200_draw_point((uint16)(x - 1U), y, color);
    if (x < x_max) ips200_draw_point((uint16)(x + 1U), y, color);
    if (y > y_min) ips200_draw_point(x, (uint16)(y - 1U), color);
    if (y < y_max) ips200_draw_point(x, (uint16)(y + 1U), color);
}

static void UI_Task2MapDrawBase(void)
{
    uint16 x0 = UI_TASK2_MAP_X0;
    uint16 y0 = UI_TASK2_MAP_Y0;
    uint16 x1 = (uint16)(UI_TASK2_MAP_X0 + UI_TASK2_MAP_W - 1U);
    uint16 y1 = (uint16)(UI_TASK2_MAP_Y0 + UI_TASK2_MAP_H - 1U);
    uint16 ox = UI_Task2ClampU16(UI_TASK2_MAP_ORIGIN_X, (int32)(x0 + 1U), (int32)(x1 - 1U));
    uint16 oy = UI_Task2ClampU16(UI_TASK2_MAP_ORIGIN_Y, (int32)(y0 + 1U), (int32)(y1 - 1U));
    uint16 scale_x0 = (uint16)(UI_TASK2_MAP_X0 + 8U);
    uint16 scale_y = (uint16)(UI_TASK2_MAP_Y0 + 14U);
    uint16 scale_x1 = (uint16)(scale_x0 + (uint16)UI_Task2RoundToInt((float)UI_TASK2_MAP_SCALE_BAR_M * UI_TASK2_MAP_PX_PER_M));

    ips200_draw_line(ox, (uint16)(y0 + 1U), ox, (uint16)(y1 - 1U), RGB565_GRAY);
    ips200_draw_line((uint16)(x0 + 1U), oy, (uint16)(x1 - 1U), oy, RGB565_GRAY);
    ips200_draw_line(scale_x0, scale_y, scale_x1, scale_y, RGB565_BLACK);
    ips200_draw_line(scale_x0, (uint16)(scale_y - 3U), scale_x0, (uint16)(scale_y + 3U), RGB565_BLACK);
    ips200_draw_line(scale_x1, (uint16)(scale_y - 3U), scale_x1, (uint16)(scale_y + 3U), RGB565_BLACK);
    ips200_show_string((uint16)(scale_x1 + 6U), (uint16)(UI_TASK2_MAP_Y0 + 6U), "3m");
    UI_Task2MapDrawCross(ox, oy, RGB565_GREEN);
}

static void UI_Task2MapDrawTrace(void)
{
    uint16 i;

    if (ui_task2_ins_trace_count == 0U)
    {
        return;
    }

    for (i = 0U; i < ui_task2_ins_trace_count; i++)
    {
        uint16 x0;
        uint16 y0;
        UI_Task2MapToPixel(ui_task2_ins_trace_x[i], ui_task2_ins_trace_y[i], &x0, &y0);
        ips200_draw_point(x0, y0, RGB565_BLUE);
        if ((i + 1U) < ui_task2_ins_trace_count)
        {
            uint16 x1;
            uint16 y1;
            UI_Task2MapToPixel(ui_task2_ins_trace_x[i + 1U], ui_task2_ins_trace_y[i + 1U], &x1, &y1);
            ips200_draw_line(x0, y0, x1, y1, RGB565_BLUE);
        }
    }
}

static void UI_Task2MapDrawPoints(void)
{
    uint8 i;

    for (i = 0U; i < UI_TASK2_POINT_COUNT; i++)
    {
        uint16 x;
        uint16 y;
        if (ui_task2_points[i].valid == 0U)
        {
            continue;
        }
        UI_Task2MapToPixel(ui_task2_points[i].x_m, ui_task2_points[i].y_m, &x, &y);
        UI_Task2MapDrawCross(x, y, (i == 0U) ? RGB565_GREEN : RGB565_BLACK);
    }
}

static void UI_Task2MapDrawNewTrace(void)
{
    uint16 i;
    uint16 start_index;

    if (ui_task2_ins_trace_count == 0U)
    {
        ui_task2_ins_trace_drawn_count = 0U;
        return;
    }

    if (ui_task2_ins_trace_drawn_count > ui_task2_ins_trace_count)
    {
        ui_task2_ins_trace_drawn_count = 0U;
    }

    if (ui_task2_ins_trace_drawn_count == 0U)
    {
        uint16 x0;
        uint16 y0;
        UI_Task2MapToPixel(ui_task2_ins_trace_x[0], ui_task2_ins_trace_y[0], &x0, &y0);
        ips200_draw_point(x0, y0, RGB565_BLUE);
        start_index = 1U;
    }
    else
    {
        start_index = ui_task2_ins_trace_drawn_count;
    }

    for (i = start_index; i < ui_task2_ins_trace_count; i++)
    {
        uint16 x0;
        uint16 y0;
        uint16 x1;
        uint16 y1;
        UI_Task2MapToPixel(ui_task2_ins_trace_x[i - 1U], ui_task2_ins_trace_y[i - 1U], &x0, &y0);
        UI_Task2MapToPixel(ui_task2_ins_trace_x[i], ui_task2_ins_trace_y[i], &x1, &y1);
        ips200_draw_line(x0, y0, x1, y1, RGB565_BLUE);
        ips200_draw_point(x1, y1, RGB565_BLUE);
    }

    ui_task2_ins_trace_drawn_count = ui_task2_ins_trace_count;
}

static void UI_Task2InsMapDrawFull(uint32 now_ms)
{
    UI_Task2MapClearArea();
    UI_Task2MapDrawBase();
    UI_Task2MapDrawPoints();
    UI_Task2MapDrawTrace();
    ui_task2_ins_trace_drawn_count = ui_task2_ins_trace_count;
    ui_task2_ins_last_draw_ms = now_ms;
    ui_task2_ins_map_drawn = 1U;
}

static void UI_Task2InsPage(void)
{
    nav_ins_state_t ins;
    ctrl_task2_status_t status;
    uint32 now_ms = system_getval_ms();
    uint8 running_now;
    uint8 step_display = 0U;
    float display_x_m;
    float display_y_m;
    float display_distance_m;
    float display_speed_mps;
    float display_encoder_speed_mps;

    nav_ins_get_state(&ins);
    ctrl_task2_get_status(&status);
    UI_Task2InsLoggerEnter(now_ms, &ins);

    running_now = ((g_task_ctrl_id == 2U) &&
                   (g_task_ctrl_state == TASK_CTRL_STATE_RUNNING)) ? 1U : 0U;
    if ((running_now != 0U) && (ui_task2_ins_last_running == 0U))
    {
        UI_Task2InsTraceClear();
    }
    ui_task2_ins_last_running = running_now;

    UI_Task2InsTraceAdd(&ins);

    if (status.route_step_count != 0U)
    {
        step_display = (uint8)(status.route_step_index + 1U);
    }

    display_x_m = UI_Task2ClipF32(ins.x_m, -99.99f, 99.99f);
    display_y_m = UI_Task2ClipF32(ins.y_m, -99.99f, 99.99f);
    display_distance_m = UI_Task2ClipF32(ins.distance_m, 0.0f, 999.9f);
    display_speed_mps = UI_Task2ClipF32(ins.speed_mps, -9.99f, 9.99f);
    display_encoder_speed_mps = UI_Task2ClipF32(ins.encoder_speed_mps, -9.99f, 9.99f);

    ips200_show_string(0, 16 * 0, "T2-INS Pg2");
    ips200_show_string(104, 16 * 0, "I");
    ips200_show_uint(120, 16 * 0, ins.valid, 1);
    ips200_show_string(144, 16 * 0, UI_TargetSrcName(g_pid_target_src));
    ips200_show_string(208, 16 * 0, UI_TaskStateName(g_task_ctrl_state));

    ips200_show_string(0, 16 * 1, "X");
    ips200_show_float(16, 16 * 1, display_x_m, 2, 2);
    ips200_show_string(112, 16 * 1, "Y");
    ips200_show_float(128, 16 * 1, display_y_m, 2, 2);

    ips200_show_string(0, 16 * 2, "Yaw");
    ips200_show_float(40, 16 * 2, ins.yaw_deg, 3, 1);
    ips200_show_string(112, 16 * 2, "D");
    ips200_show_float(128, 16 * 2, display_distance_m, 3, 1);
    ips200_show_string(192, 16 * 2, "m");

    ips200_show_string(0, 16 * 3, "V");
    ips200_show_float(16, 16 * 3, display_speed_mps, 1, 2);
    ips200_show_string(72, 16 * 3, "E");
    ips200_show_float(88, 16 * 3, display_encoder_speed_mps, 1, 2);
    ips200_show_string(144, 16 * 3, "C");
    ips200_show_uint(160, 16 * 3, status.last_cmd, 3);
    ips200_show_string(192, 16 * 3, "W");
    ips200_show_uint(208, 16 * 3, step_display, 1);
    ips200_show_string(216, 16 * 3, "/");
    ips200_show_uint(224, 16 * 3, status.route_step_count, 1);

    if (ui_task2_ins_map_drawn == 0U)
    {
        UI_Task2InsMapDrawFull(now_ms);
    }
    else if ((ui_task2_ins_trace_drawn_count < ui_task2_ins_trace_count) &&
             ((now_ms - ui_task2_ins_last_draw_ms) >= UI_TASK2_MAP_TRACE_DRAW_MS))
    {
        UI_Task2MapDrawNewTrace();
        ui_task2_ins_last_draw_ms = now_ms;
    }
}

static uint8 UI_MapShouldRedraw(uint8 point_count)
{
    uint8 task_id = ui_task_get();

    if (point_count <= 1U)
    {
        return 0U;
    }

    if ((0U == ui_map_drawn) ||
        (ui_map_last_page != Page_Num) ||
        (ui_map_last_task != task_id) ||
        (ui_map_last_point_count != point_count))
    {
        ui_map_drawn = 1U;
        ui_map_last_page = Page_Num;
        ui_map_last_task = task_id;
        ui_map_last_point_count = point_count;
        return 1U;
    }

    return 0U;
}

static void UI_TaskSyncToLegacyVars(void)
{
    ui_task_state_t *state = ui_task_curr_state();
    if (state->program_select < 1 || state->program_select > 2)
    {
        state->program_select = 1;
    }
    program_select = state->program_select;
    camera_exposure = state->value1;
    Camera_compare = state->value2;
}

static void UI_TaskSyncFromLegacyVars(void)
{
    ui_task_state_t *state = ui_task_curr_state();
    state->program_select = program_select;
    state->value1 = camera_exposure;
    state->value2 = Camera_compare;
}

static void UI_Config(void)
{
    if (ui_task_get() == 2U)
    {
        UI_Task2InfoPage();
        return;
    }

    ips200_show_string(0, 16 * 1, "Info-Page");
    ips200_show_string(0, 16 * 0, "task");
    ips200_show_uint(120, 16 * 0, ui_task_get(), 1);

    // IMU attitude
    ips200_show_string(0, 16 * 2, "Pitch");
    ips200_show_float(120, 16 * 2, IMU_RelPitch, 3, 2);
    ips200_show_string(0, 16 * 3, "Roll");
    ips200_show_float(120, 16 * 3, IMU_RelRoll, 3, 2);
    ips200_show_string(0, 16 * 4, "Yaw");
    ips200_show_float(120, 16 * 4, IMU_RelYaw, 3, 2);
    ips200_show_string(0, 16 * 5, "Gyro_Z");
    ips200_show_float(120, 16 * 5, IMU_GYRO_Z, 3, 2);

    // GNSS status
    ips200_show_string(0, 16 * 6, "GNSS state");
    ips200_show_uint(120, 16 * 6, gnss.state, 2);
    ips200_show_string(0, 16 * 7, "satellite");
    ips200_show_uint(120, 16 * 7, gnss.satellite_used, 2);

    ips200_show_string(0, 16 * 8, "Page");
    ips200_show_uint(120, 16 * 8, (uint32)Page_Num, 1);
    ips200_show_string(0, 16 * 9, "UI mode");
    ips200_show_uint(120, 16 * 9, UI_Mode, 1);

    ips200_show_string(0, 16 * 10, "Mode");
    ips200_show_string(120, 16 * 10, UI_TargetSrcName(g_pid_target_src));

    ips200_show_string(0, 16 * 11, "TargetUse");
    ips200_show_float(120, 16 * 11, g_pid_used_target_speed_mps, 1, 2);
    ips200_show_string(200, 16 * 11, "m/s");

    ips200_show_string(0, 16 * 12, "TargetAng");
    ips200_show_float(120, 16 * 12, g_steer_target_angle_deg, 3, 1);
    ips200_show_string(200, 16 * 12, "deg");

    ips200_show_string(0, 16 * 13, "SteerNow");
    if (g_steer_feedback_valid)
    {
        ips200_show_float(120, 16 * 13, g_steer_feedback_angle_deg, 3, 2);
        ips200_show_string(200, 16 * 13, "deg");
    }
    else
    {
        ips200_show_string(120, 16 * 13, "INVALID");
    }

    if (g_pid_target_src == TARGET_SRC_REMOTE)
    {
        ips200_show_string(0, 16 * 14, "R1");
        ips200_show_uint(24, 16 * 14, g_yaokong.channel[YAOKONG_CHANNEL_STEER], 4);
        ips200_show_string(64, 16 * 14, "M");
        ips200_show_uint(80, 16 * 14, YAOKONG_STEER_MID, 4);
        ips200_show_string(120, 16 * 14, "A");
        ips200_show_uint(136, 16 * 14, yaokong_remote_is_armed(), 1);
        ips200_show_string(160, 16 * 14, "Run");
        ips200_show_uint(192, 16 * 14, yaokong_is_motor_run_enabled(), 1);

        ips200_show_string(0, 16 * 15, "R2");
        ips200_show_uint(24, 16 * 15, g_yaokong.channel[YAOKONG_CHANNEL_THROTTLE], 4);
        ips200_show_string(64, 16 * 15, "M");
        ips200_show_uint(80, 16 * 15, YAOKONG_THROTTLE_MID, 4);
        ips200_show_string(120, 16 * 15, "Ang");
        ips200_show_float(152, 16 * 15, yaokong_get_target_steer_angle(), 3, 1);
    }
    else
    {
        ips200_show_string(0, 16 * 14, "TaskRun");
        ips200_show_uint(80, 16 * 14, g_task_ctrl_id, 1);
        ips200_show_string(120, 16 * 14, UI_TaskStateName(g_task_ctrl_state));

        if (g_pid_target_src == TARGET_SRC_PWM_TEST)
        {
            ips200_show_string(0, 16 * 15, "PWM");
            ips200_show_int(80, 16 * 15, g_pwm_test_duty, 5);
            ips200_show_string(130, 16 * 15, "Duty");
            ips200_show_int(176, 16 * 15, g_ctrl_cmd.pwm_duty, 5);
        }
        else if (g_pid_target_src == TARGET_SRC_STEER_SCAN)
        {
            ips200_show_string(0, 16 * 15, "Scan");
            ips200_show_string(40, 16 * 15, UI_SteerScanPhaseName(g_steer_scan_phase));
            if (g_steer_scan_success)
            {
                ips200_show_string(96, 16 * 15, "C");
                ips200_show_int(112, 16 * 15, g_steer_scan_center_raw, 5);
                ips200_show_string(160, 16 * 15, "S");
                ips200_show_int(176, 16 * 15, g_steer_scan_span_counts, 5);
            }
            else
            {
                ips200_show_string(96, 16 * 15, "L");
                ips200_show_int(112, 16 * 15, g_steer_scan_left_raw, 5);
                ips200_show_string(160, 16 * 15, "R");
                ips200_show_int(176, 16 * 15, g_steer_scan_right_raw, 5);
            }
        }
        else
        {
            ips200_show_string(0, 16 * 15, "ABSraw");
            ips200_show_int(80, 16 * 15, g_abs_encoder_raw, 6);
            ips200_show_string(160, 16 * 15, "cnt");
        }
    }
}

static void UI_Set(void)
{
    const ui_task_display_cfg_t *cfg = UI_GetDisplayCfg();

    if (ui_task_get() == 1U)
    {
        UI_Task1RequestSdDir();
        UI_PathRecordPage();
        return;
    }
    if (ui_task_get() == 2U)
    {
        UI_Task2InsPage();
        return;
    }
    if (ui_task_get() == 3U)
    {
        UI_Task3ColorCameraPage();
        return;
    }
    ips200_show_string(0, 16 * 1, cfg->set_title);
    ips200_show_string(0, 16 * 2, cfg->set_item1);
    ips200_show_uint(120, 16 * 2, program_select, 2);
    ips200_show_string(0, 16 * 3, cfg->set_item2);
    ips200_show_uint(120, 16 * 3, camera_exposure, 4);
    ips200_show_string(210, 16 * 2, "   ");
    ips200_show_string(210, 16 * 3, "   ");
    if (program_select == 1)
    {
        ips200_show_string(210, 16 * 2, "<--");
    }
    else
    {
        ips200_show_string(210, 16 * 3, "<--");
    }

    if (ui_task_get() == 0U)
    {
        taban_status_t taban_status;
        float ecod1_mps;
        float ecod2_mps;
        taban_get_status(&taban_status);
        ecod1_mps = UI_EncoderSpeedToMps(g_ecod1_speed);
        ecod2_mps = UI_EncoderSpeedToMps(g_ecod2_speed);

        ips200_show_string(0, 16 * 5, "ER");
        ips200_show_int(24, 16 * 5, g_ecod1_count, 6);
        ips200_show_string(140, 16 * 5, "EL");
        ips200_show_int(164, 16 * 5, g_ecod2_count, 6);

        ips200_show_string(0, 16 * 6, "SR");
        ips200_show_int(24, 16 * 6, g_ecod1_speed, 6);
        ips200_show_string(140, 16 * 6, "SL");
        ips200_show_int(164, 16 * 6, g_ecod2_speed, 6);

        ips200_show_string(0, 16 * 7, "A16");
        ips200_show_uint(32, 16 * 7, taban_status.raw_adc[TABAN_ID_0], 4);
        ips200_show_string(140, 16 * 7, "A17");
        ips200_show_uint(172, 16 * 7, taban_status.raw_adc[TABAN_ID_1], 4);

        ips200_show_string(0, 16 * 8, "F16");
        ips200_show_uint(32, 16 * 8, taban_status.filt_adc[TABAN_ID_0], 4);
        ips200_show_string(140, 16 * 8, "F17");
        ips200_show_uint(172, 16 * 8, taban_status.filt_adc[TABAN_ID_1], 4);

        ips200_show_string(0, 16 * 9, "P16(%)");
        ips200_show_float(56, 16 * 9, (double)taban_status.throttle_permille[TABAN_ID_0] / 10.0, 3, 1);
        ips200_show_string(140, 16 * 9, "P17(%)");
        ips200_show_float(188, 16 * 9, (double)taban_status.throttle_permille[TABAN_ID_1] / 10.0, 3, 1);

        ips200_show_string(0, 16 * 10, "Vt");
        ips200_show_float(72, 16 * 10, taban_status.target_speed_mps, 1, 2);
        ips200_show_string(0, 16 * 11, "VR(m/s)");
        ips200_show_float(80, 16 * 11, ecod1_mps, 1, 3);
        ips200_show_string(0, 16 * 12, "VL(m/s)");
        ips200_show_float(80, 16 * 12, ecod2_mps, 1, 3);
    }
}


static void UI_Task1ImuPage(void)
{
    imu_ahrs_debug_t dbg;
    uint32 now_ms;
    uint32 update_ms;
    uint32 imu_age_ms;
    uint32 pit_count;
    uint32 update_count;
    uint32 fail_count;

    now_ms = system_getval_ms();
    UI_Task1ImuLoggerEnter(now_ms);
    IMU_AHRS_GetDebugInfo(&dbg);
    update_ms = g_imu_update_ms;
    pit_count = g_imu_pit_count;
    update_count = g_imu_update_count;
    fail_count = g_imu_read_fail_count;
    if (update_ms == 0U)
    {
        imu_age_ms = 9999U;
    }
    else
    {
        imu_age_ms = now_ms - update_ms;
        if (imu_age_ms > 9999U)
        {
            imu_age_ms = 9999U;
        }
    }

    ips200_show_string(0, 16 * 0, "T1-IMU init"); ips200_show_uint(96, 16 * 0, g_imu963ra_init_ret, 3); ips200_show_string(128, 16 * 0, "up"); ips200_show_uint(152, 16 * 0, update_count % 100000U, 5);
    ips200_show_string(0, 16 * 1, "Pitch"); ips200_show_float(72, 16 * 1, IMU_RelPitch, 4, 1); ips200_show_string(160, 16 * 1, "deg");
    ips200_show_string(0, 16 * 2, "Roll"); ips200_show_float(72, 16 * 2, IMU_RelRoll, 4, 1); ips200_show_string(160, 16 * 2, "deg");
    ips200_show_string(0, 16 * 3, "Yaw"); ips200_show_float(72, 16 * 3, IMU_RelYaw, 4, 1); ips200_show_string(160, 16 * 3, "deg");
    ips200_show_string(0, 16 * 4, "AbsP"); ips200_show_float(40, 16 * 4, IMU_Pitch, 3, 1); ips200_show_string(96, 16 * 4, "R"); ips200_show_float(112, 16 * 4, IMU_Roll, 3, 1); ips200_show_string(168, 16 * 4, "Y"); ips200_show_float(184, 16 * 4, IMU_Yaw, 3, 1);
    ips200_show_string(0, 16 * 5, "rx"); ips200_show_uint(24, 16 * 5, dbg.receive_flag, 1); ips200_show_string(48, 16 * 5, "flt"); ips200_show_uint(80, 16 * 5, dbg.filter_initialized, 1); ips200_show_string(104, 16 * 5, "dt"); ips200_show_uint(128, 16 * 5, dbg.dt_initialized, 1); ips200_show_string(152, 16 * 5, "att"); ips200_show_uint(184, 16 * 5, dbg.attitude_initialized, 1);
    ips200_show_string(0, 16 * 6, "ref"); ips200_show_uint(32, 16 * 6, dbg.ref_locked, 1); ips200_show_string(56, 16 * 6, "mag"); ips200_show_uint(88, 16 * 6, dbg.mag_enabled, 1); ips200_show_string(112, 16 * 6, "req"); ips200_show_uint(144, 16 * 6, dbg.ref_capture_request, 1); ips200_show_string(168, 16 * 6, "br"); ips200_show_uint(200, 16 * 6, dbg.gyro_bias_ready, 1);
    ips200_show_string(0, 16 * 7, "bias"); ips200_show_uint(48, 16 * 7, dbg.gyro_bias_sample_count, 4); ips200_show_string(88, 16 * 7, "/"); ips200_show_uint(104, 16 * 7, dbg.gyro_bias_sample_target, 4);
    ips200_show_string(0, 16 * 8, "age"); ips200_show_uint(32, 16 * 8, imu_age_ms, 4); ips200_show_string(72, 16 * 8, "pit"); ips200_show_uint(104, 16 * 8, pit_count % 100000U, 5); ips200_show_string(152, 16 * 8, "fail"); ips200_show_uint(192, 16 * 8, fail_count % 10000U, 4);
    ips200_show_string(0, 16 * 9, "log"); ips200_show_uint(32, 16 * 9, data_logger_get_state(), 1); ips200_show_string(56, 16 * 9, "err"); ips200_show_uint(88, 16 * 9, data_logger_get_last_error(), 2); ips200_show_string(128, 16 * 9, "dt"); ips200_show_float(152, 16 * 9, dbg.sample_dt_s, 1, 3);
    ips200_show_string(0, 16 * 10, "gyroX"); ips200_show_float(56, 16 * 10, dbg.gyro_x_dps, 4, 1); ips200_show_string(128, 16 * 10, "Y"); ips200_show_float(144, 16 * 10, dbg.gyro_y_dps, 4, 1);
    ips200_show_string(0, 16 * 11, "gyroZ"); ips200_show_float(56, 16 * 11, dbg.gyro_z_dps, 4, 1); ips200_show_string(128, 16 * 11, "all"); ips200_show_float(160, 16 * 11, dbg.gyro_norm_dps, 4, 1);
    ips200_show_string(0, 16 * 12, "biasX"); ips200_show_float(56, 16 * 12, dbg.gyro_bias_x_dps, 4, 1); ips200_show_string(128, 16 * 12, "Y"); ips200_show_float(144, 16 * 12, dbg.gyro_bias_y_dps, 4, 1);
    ips200_show_string(0, 16 * 13, "biasZ"); ips200_show_float(56, 16 * 13, dbg.gyro_bias_z_dps, 4, 1); ips200_show_string(128, 16 * 13, "GZ"); ips200_show_float(160, 16 * 13, IMU_GYRO_Z, 4, 1);
    ips200_show_string(0, 16 * 14, "accX"); ips200_show_float(48, 16 * 14, imu_data.acc_x, 1, 3); ips200_show_string(112, 16 * 14, "Y"); ips200_show_float(128, 16 * 14, imu_data.acc_y, 1, 3);
    ips200_show_string(0, 16 * 15, "accZ"); ips200_show_float(48, 16 * 15, imu_data.acc_z, 1, 3); ips200_show_string(112, 16 * 15, "all"); ips200_show_float(152, 16 * 15, dbg.acc_norm_g, 1, 3);
    ips200_show_string(0, 16 * 16, "accOK"); ips200_show_float(56, 16 * 16, dbg.accel_confidence, 1, 2); ips200_show_string(120, 16 * 16, "magOK"); ips200_show_float(176, 16 * 16, dbg.mag_confidence, 1, 2);
    ips200_show_string(0, 16 * 17, "magX"); ips200_show_float(48, 16 * 17, imu_data.mag_x, 4, 1); ips200_show_string(112, 16 * 17, "Y"); ips200_show_float(128, 16 * 17, imu_data.mag_y, 4, 1);
    ips200_show_string(0, 16 * 18, "magZ"); ips200_show_float(48, 16 * 18, imu_data.mag_z, 4, 1); ips200_show_string(112, 16 * 18, "all"); ips200_show_float(152, 16 * 18, dbg.mag_norm, 4, 1);
    ips200_show_string(0, 16 * 19, "rawZ"); ips200_show_float(48, 16 * 19, IMU_MagRawZ, 4, 1); ips200_show_string(112, 16 * 19, "mRef"); ips200_show_float(160, 16 * 19, dbg.mag_norm_ref, 4, 1);
}

static uint16 UI_InsClampU16(int32 value, int32 min_value, int32 max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }

    return (uint16)value;
}

static void UI_InsTraceClearArea(void)
{
    uint16 y;
    uint16 x0 = UI_INS_TRACE_X0;
    uint16 y0 = UI_INS_TRACE_Y0;
    uint16 x1 = (uint16)(UI_INS_TRACE_X0 + UI_INS_TRACE_W - 1U);
    uint16 y1 = (uint16)(UI_INS_TRACE_Y0 + UI_INS_TRACE_H - 1U);

    for (y = y0; y <= y1; y++)
    {
        ips200_draw_line(x0, y, x1, y, RGB565_WHITE);
    }

    ips200_draw_line(x0, y0, x1, y0, RGB565_GRAY);
    ips200_draw_line(x0, y1, x1, y1, RGB565_GRAY);
    ips200_draw_line(x0, y0, x0, y1, RGB565_GRAY);
    ips200_draw_line(x1, y0, x1, y1, RGB565_GRAY);
}

static uint16 UI_InsMapX(float value, float min_value, float scale, float offset)
{
    int32 x = (int32)(offset + (value - min_value) * scale + 0.5f);
    return UI_InsClampU16(x,
                          (int32)(UI_INS_TRACE_X0 + 1U),
                          (int32)(UI_INS_TRACE_X0 + UI_INS_TRACE_W - 2U));
}

static uint16 UI_InsMapY(float value, float min_value, float scale, float offset)
{
    int32 y = (int32)((float)(UI_INS_TRACE_Y0 + UI_INS_TRACE_H - 2U) -
                      (offset + (value - min_value) * scale) + 0.5f);
    return UI_InsClampU16(y,
                          (int32)(UI_INS_TRACE_Y0 + 1U),
                          (int32)(UI_INS_TRACE_Y0 + UI_INS_TRACE_H - 2U));
}

static void UI_TraceDraw(const float *trace_x, const float *trace_y, uint16 count, const char *empty_text)
{
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float span_x;
    float span_y;
    float scale_x;
    float scale_y;
    float scale;
    float offset_x;
    float offset_y;
    uint16 i;

    UI_InsTraceClearArea();

    if ((trace_x == NULL) || (trace_y == NULL) || (count == 0U))
    {
        ips200_show_string(8, UI_INS_TRACE_Y0 + 8U, empty_text);
        return;
    }

    min_x = trace_x[0];
    max_x = trace_x[0];
    min_y = trace_y[0];
    max_y = trace_y[0];

    for (i = 1U; i < count; i++)
    {
        if (trace_x[i] < min_x) min_x = trace_x[i];
        if (trace_x[i] > max_x) max_x = trace_x[i];
        if (trace_y[i] < min_y) min_y = trace_y[i];
        if (trace_y[i] > max_y) max_y = trace_y[i];
    }

    span_x = max_x - min_x;
    span_y = max_y - min_y;
    if (span_x < 0.5f)
    {
        min_x -= (0.5f - span_x) * 0.5f;
        span_x = 0.5f;
    }
    if (span_y < 0.5f)
    {
        min_y -= (0.5f - span_y) * 0.5f;
        span_y = 0.5f;
    }

    scale_x = (float)(UI_INS_TRACE_W - 12U) / span_x;
    scale_y = (float)(UI_INS_TRACE_H - 12U) / span_y;
    scale = (scale_x < scale_y) ? scale_x : scale_y;
    offset_x = ((float)UI_INS_TRACE_W - span_x * scale) * 0.5f;
    offset_y = ((float)UI_INS_TRACE_H - span_y * scale) * 0.5f;

    for (i = 0U; i < count; i++)
    {
        uint16 x0 = UI_InsMapX(trace_x[i], min_x, scale, offset_x);
        uint16 y0 = UI_InsMapY(trace_y[i], min_y, scale, offset_y);
        ips200_draw_point(x0, y0, RGB565_BLUE);
        if ((i + 1U) < count)
        {
            uint16 x1 = UI_InsMapX(trace_x[i + 1U], min_x, scale, offset_x);
            uint16 y1 = UI_InsMapY(trace_y[i + 1U], min_y, scale, offset_y);
            ips200_draw_line(x0, y0, x1, y1, RGB565_BLUE);
        }
    }

    ips200_draw_point(UI_InsMapX(trace_x[0], min_x, scale, offset_x),
                      UI_InsMapY(trace_y[0], min_y, scale, offset_y),
                      RGB565_GREEN);
    ips200_draw_point(UI_InsMapX(trace_x[count - 1U], min_x, scale, offset_x),
                      UI_InsMapY(trace_y[count - 1U], min_y, scale, offset_y),
                      RGB565_RED);
}

static void UI_PathTraceBuild(const nav_path_state_t *path_state)
{
    uint16 i;
    uint16 step;
    uint16 drawn = 0U;
    float x = 0.0f;
    float y = 0.0f;
    float yaw_deg;
    float yaw_rad;
    float interval_m;

    if ((path_state == NULL) || (path_state->sample_count == 0U))
    {
        ui_path_trace_count = 0U;
        ui_path_trace_last_count = 0U;
        ui_path_trace_drawn = 0U;
        return;
    }

    step = (uint16)(((uint32)path_state->sample_count + UI_INS_TRACE_MAX - 1U) / UI_INS_TRACE_MAX);
    if (step == 0U)
    {
        step = 1U;
    }

    interval_m = path_state->sample_interval_m;
    if (interval_m <= 0.0f)
    {
        interval_m = NAV_PATH_DEFAULT_INTERVAL_M;
    }

    for (i = 0U; i < path_state->sample_count; i++)
    {
        if (nav_path_get_yaw_deg(i, &yaw_deg) != 0U)
        {
            break;
        }

        if ((i == 0U) || ((i % step) == 0U) || ((i + 1U) == path_state->sample_count))
        {
            if (drawn < UI_INS_TRACE_MAX)
            {
                ui_path_trace_x[drawn] = x;
                ui_path_trace_y[drawn] = y;
                drawn++;
            }
        }

        yaw_rad = yaw_deg * NAV_PI / 180.0f;
        x += sinf(yaw_rad) * interval_m;
        y += cosf(yaw_rad) * interval_m;
    }

    ui_path_trace_count = drawn;
    ui_path_trace_last_count = path_state->sample_count;
    ui_path_trace_drawn = 0U;
}
static void UI_Task0_InsPage(void)
{
    nav_ins_state_t ins;

    nav_ins_get_state(&ins);

    ips200_show_string(0, 16 * 0, "T0-INS");
    ips200_show_string(80, 16 * 0, "valid");
    ips200_show_uint(128, 16 * 0, ins.valid, 1);

    ips200_show_string(0, 16 * 1, "X(m)");
    ips200_show_float(56, 16 * 1, ins.x_m, 4, 2);
    ips200_show_string(128, 16 * 1, "Y");
    ips200_show_float(152, 16 * 1, ins.y_m, 4, 2);

    ips200_show_string(0, 16 * 2, "Dist");
    ips200_show_float(56, 16 * 2, ins.distance_m, 4, 2);
    ips200_show_string(128, 16 * 2, "Yaw");
    ips200_show_float(168, 16 * 2, ins.yaw_deg, 3, 1);

    ips200_show_string(0, 16 * 3, "Spd");
    ips200_show_float(56, 16 * 3, ins.speed_mps, 2, 2);
    ips200_show_string(128, 16 * 3, "Enc");
    ips200_show_float(168, 16 * 3, ins.encoder_speed_mps, 2, 2);

    ips200_show_string(0, 16 * 4, "GyroZ");
    ips200_show_float(56, 16 * 4, ins.yaw_rate_dps, 3, 1);
    ips200_show_string(128, 16 * 4, "Acc");
    ips200_show_float(168, 16 * 4, ins.forward_accel_mps2, 2, 2);

    ips200_show_string(0, 16 * 5, "ER");
    ips200_show_int(24, 16 * 5, ins.encoder_r_count, 6);
    ips200_show_string(128, 16 * 5, "EL");
    ips200_show_int(152, 16 * 5, ins.encoder_l_count, 6);

    ips200_show_string(0, 16 * 6, "Pure INS: encoder + IMU yaw");
    ips200_show_string(0, 16 * 7, "Path map: T1/T3 K2 record");
}

static const char *UI_CameraSnapshotSizeName(uint8 size)
{
    if (size == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_HALF)
    {
        return "HALF";
    }
    if (size == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_QUARTER)
    {
        return "QTR ";
    }
    return "FULL";
}

static uint8 UI_CameraRecordFps(uint8 size)
{
    if (size == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_HALF)
    {
        return 10U;
    }
    if (size == (uint8)DATA_LOGGER_CAMERA_SNAPSHOT_QUARTER)
    {
        return 15U;
    }
    return 5U;
}

static uint8 UI_CameraConfiguredFps(void)
{
    uint32 fps = SCC8660_FPS_DEF;

    switch (SCC8660_PCLK_DIV_DEF)
    {
        case 1:
            fps = (fps * 2U + 1U) / 3U;
            break;
        case 2:
            fps = (fps + 1U) / 2U;
            break;
        case 3:
            fps = (fps + 1U) / 3U;
            break;
        case 4:
            fps = (fps + 2U) / 4U;
            break;
        case 5:
            fps = (fps + 4U) / 8U;
            break;
        default:
            break;
    }
    return (uint8)fps;
}

static const char *UI_CameraSnapshotStateName(uint8 state)
{
    switch (state)
    {
        case DATA_LOGGER_CAMERA_SNAPSHOT_PENDING:
            return "WAIT ";
        case DATA_LOGGER_CAMERA_SNAPSHOT_WRITING:
            return "WRITE";
        case DATA_LOGGER_CAMERA_SNAPSHOT_DONE:
            return "DONE ";
        case DATA_LOGGER_CAMERA_SNAPSHOT_FAILED:
            return "FAIL ";
        default:
            return "IDLE ";
    }
}

static const char *UI_CameraRecordStateName(uint8 state)
{
    switch (state)
    {
        case DATA_LOGGER_CAMERA_RECORD_STARTING:
            return "START";
        case DATA_LOGGER_CAMERA_RECORD_RUNNING:
            return "RUN  ";
        case DATA_LOGGER_CAMERA_RECORD_STOPPING:
            return "STOP ";
        case DATA_LOGGER_CAMERA_RECORD_DONE:
            return "DONE ";
        case DATA_LOGGER_CAMERA_RECORD_FAILED:
            return "FAIL ";
        default:
            return "IDLE ";
    }
}

static void UI_Task3CameraBuildMeta(
    data_logger_camera_frame_meta_t *meta,
    const blue_target_result_t *target,
    uint32 now_ms,
    uint32 vision_load_pct)
{
    meta->time_ms = now_ms;
    meta->target_valid = target->valid;
    meta->confidence_pct = target->confidence_pct;
    meta->fill_pct = target->fill_pct;
    meta->distance_valid = target->distance_valid;
    meta->center_x = target->center_x;
    meta->center_y = target->center_y;
    meta->offset_x = target->offset_x;
    meta->bearing_deg_x10 = target->bearing_deg_x10;
    meta->bbox_x = target->bbox_x;
    meta->bbox_y = target->bbox_y;
    meta->bbox_width = target->bbox_width;
    meta->bbox_height = target->bbox_height;
    meta->area_px = target->area_px;
    meta->distance_cm = target->distance_cm;
    meta->process_us = target->process_us;
    meta->camera_fps = (uint16)ui_color_camera_fps;
    meta->vision_load_pct = (uint16)((vision_load_pct > 65535U) ? 65535U : vision_load_pct);
}

void UI_Task3CameraSnapshotCycleSize(void)
{
    if ((ui_task_get() == 3U) && (Page_Num == GPSPage))
    {
        if (data_logger_camera_record_is_active() != 0U)
        {
            data_logger_camera_record_stop();
        }
        ui_color_camera_snapshot_size++;
        if (ui_color_camera_snapshot_size >= DATA_LOGGER_CAMERA_SNAPSHOT_SIZE_COUNT)
        {
            ui_color_camera_snapshot_size = DATA_LOGGER_CAMERA_SNAPSHOT_FULL;
        }
        ui_color_camera_record_last_request_ms = 0U;
        ui_color_camera_action_last_result = 0U;
        Gui_Refersh_Bool = ZF_TRUE;
    }
}

void UI_Task3CameraSnapshotRequest(void)
{
    uint8 target_fps;

    if ((ui_task_get() == 3U) && (Page_Num == GPSPage))
    {
        if (guimai_voice_is_recording() != 0U)
        {
            data_logger_camera_record_stop();
            ui_color_camera_action_last_result = 4U;
        }
        else if (ui_color_camera_init_ok == 0U)
        {
            ui_color_camera_action_last_result = 9U;
        }
        else if (ui_color_camera_capture_mode == UI_TASK3_CAMERA_MODE_RECORD)
        {
            if (data_logger_camera_record_is_active() != 0U)
            {
                data_logger_camera_record_stop();
                ui_color_camera_action_last_result = 0U;
            }
            else
            {
                target_fps = UI_CameraRecordFps(ui_color_camera_snapshot_size);
                ui_color_camera_action_last_result =
                    data_logger_camera_record_start(
                        (data_logger_camera_snapshot_size_t)ui_color_camera_snapshot_size,
                        target_fps);
                if (ui_color_camera_action_last_result == 0U)
                {
                    ui_color_camera_record_last_request_ms = 0U;
                }
            }
        }
        else
        {
            ui_color_camera_snapshot_request = 1U;
            ui_color_camera_action_last_result = 0U;
        }
        Gui_Refersh_Bool = ZF_TRUE;
    }
}

void UI_Task3CameraCaptureModeToggle(void)
{
    if ((ui_task_get() == 3U) && (Page_Num == GPSPage))
    {
        if (data_logger_camera_record_is_active() != 0U)
        {
            data_logger_camera_record_stop();
        }
        ui_color_camera_snapshot_request = 0U;
        ui_color_camera_record_last_request_ms = 0U;
        ui_color_camera_action_last_result = 0U;
        ui_color_camera_capture_mode =
            (ui_color_camera_capture_mode == UI_TASK3_CAMERA_MODE_RECORD) ?
            UI_TASK3_CAMERA_MODE_PICTURE :
            UI_TASK3_CAMERA_MODE_RECORD;
        Gui_Refersh_Bool = ZF_TRUE;
    }
}

static uint16 UI_Task3CameraTestAbsDiffU16(uint16 a, uint16 b)
{
    return (a >= b) ? (uint16)(a - b) : (uint16)(b - a);
}

static const char *UI_Task3CameraTestStateName(uint8 state)
{
    switch (state)
    {
        case DATA_LOGGER_CAMERA_TEST_STARTING:
            return "START";
        case DATA_LOGGER_CAMERA_TEST_RUNNING:
            return "RUN";
        case DATA_LOGGER_CAMERA_TEST_STOPPING:
            return "STOP";
        case DATA_LOGGER_CAMERA_TEST_DONE:
            return "DONE";
        case DATA_LOGGER_CAMERA_TEST_FAILED:
            return "FAIL";
        default:
            return "IDLE";
    }
}

static const char *UI_Task3CameraTestReasonName(uint8 reason)
{
    switch (reason)
    {
        case DATA_LOGGER_CAMERA_TEST_REASON_MANUAL:
            return "MANUAL";
        case DATA_LOGGER_CAMERA_TEST_REASON_LOST:
            return "LOST";
        case DATA_LOGGER_CAMERA_TEST_REASON_CONFIDENCE:
            return "CONF";
        case DATA_LOGGER_CAMERA_TEST_REASON_OFFSET_JUMP:
            return "DXJMP";
        case DATA_LOGGER_CAMERA_TEST_REASON_DISTANCE_JUMP:
            return "DJMP";
        case DATA_LOGGER_CAMERA_TEST_REASON_BOX_JUMP:
            return "HJMP";
        default:
            return "NONE";
    }
}

static void UI_Task3CameraTestResetTracking(void)
{
    ui_task3_camera_test_have_previous = 0U;
    ui_task3_camera_test_previous_valid = 0U;
    ui_task3_camera_test_previous_center_x = 0U;
    ui_task3_camera_test_previous_bbox_height = 0U;
    ui_task3_camera_test_previous_distance_cm = 0U;
    ui_task3_camera_test_invalid_frames = 0U;
    ui_task3_camera_test_last_event_ms = 0U;
    ui_task3_camera_test_manual_request = 0U;
}

void UI_Task3CameraTestStart(uint8 mode)
{
    uint8 ret;

    if ((ui_task_get() != 3U) || (Page_Num != GPS_SetPage))
    {
        return;
    }
    if (ctrl_task_is_running() != 0U)
    {
        ui_task3_camera_test_action_result = 10U;
        Gui_Refersh_Bool = ZF_TRUE;
        return;
    }
    if (data_logger_camera_record_is_active() != 0U)
    {
        data_logger_camera_record_stop();
        ui_task3_camera_test_action_result = 11U;
        Gui_Refersh_Bool = ZF_TRUE;
        return;
    }
    if (blue_target_camera_start() != 0U)
    {
        ui_task3_camera_test_action_result = 9U;
        Gui_Refersh_Bool = ZF_TRUE;
        return;
    }

    ret = data_logger_camera_test_start(mode);
    ui_task3_camera_test_action_result = ret;
    if (ret == 0U)
    {
        UI_Task3CameraTestResetTracking();
    }
    Gui_Refersh_Bool = ZF_TRUE;
}

uint8 UI_Task3FollowTestCaptureStart(void)
{
    ctrl_task3_follow_status_t follow_status;
    uint8 ret;

    ctrl_task3_follow_get_status(&follow_status);
    if ((ui_task_get() != 3U) ||
        ((Page_Num != SetPage) && (Page_Num != GPS_SetPage)) ||
        (follow_status.active == 0U) ||
        (follow_status.record_path != 0U))
    {
        ui_task3_camera_test_action_result = 10U;
        Gui_Refersh_Bool = ZF_TRUE;
        return 10U;
    }

    ret = data_logger_camera_test_start_parallel(
        DATA_LOGGER_CAMERA_TEST_MODE_MOTION);
    ui_task3_camera_test_action_result = ret;
    if (ret == 0U)
    {
        UI_Task3CameraTestResetTracking();
    }
    Gui_Refersh_Bool = ZF_TRUE;
    return ret;
}

void UI_Task3CameraTestStop(void)
{
    if ((ui_task_get() == 3U) &&
        (data_logger_camera_test_is_active() != 0U))
    {
        data_logger_camera_test_stop();
        ui_task3_camera_test_action_result = 0U;
        Gui_Refersh_Bool = ZF_TRUE;
    }
}

void UI_Task3CameraTestManualSave(void)
{
    if ((ui_task_get() == 3U) && (Page_Num == GPS_SetPage) &&
        (data_logger_camera_test_is_active() != 0U))
    {
        ui_task3_camera_test_manual_request = 1U;
        Gui_Refersh_Bool = ZF_TRUE;
    }
}

static void UI_Task3CameraTestUpdate(
    const uint16 *image,
    const data_logger_camera_frame_meta_t *meta)
{
    uint8 reason = 0U;
    uint8 mode;
    uint16 center_jump;
    uint16 height_jump;
    uint16 distance_jump;
    uint16 center_limit;
    uint16 height_limit;
    uint16 distance_limit;
    uint32 now_ms;
    uint8 ret;

    if ((image == NULL) || (meta == NULL) ||
        (data_logger_camera_test_get_state() !=
         (uint8)DATA_LOGGER_CAMERA_TEST_RUNNING))
    {
        return;
    }

    now_ms = system_getval_ms();
    mode = data_logger_camera_test_get_mode();
    center_limit = (mode == DATA_LOGGER_CAMERA_TEST_MODE_STATIC) ? 24U : 48U;
    height_limit = (mode == DATA_LOGGER_CAMERA_TEST_MODE_STATIC) ? 16U : 28U;
    distance_limit = (mode == DATA_LOGGER_CAMERA_TEST_MODE_STATIC) ? 75U : 140U;

    if (meta->target_valid == 0U)
    {
        if (ui_task3_camera_test_invalid_frames < 255U)
        {
            ui_task3_camera_test_invalid_frames++;
        }
    }
    else
    {
        ui_task3_camera_test_invalid_frames = 0U;
    }

    if (ui_task3_camera_test_manual_request != 0U)
    {
        reason = DATA_LOGGER_CAMERA_TEST_REASON_MANUAL;
        ui_task3_camera_test_manual_request = 0U;
    }
    else if ((ui_task3_camera_test_have_previous != 0U) &&
             (ui_task3_camera_test_previous_valid != 0U) &&
             (meta->target_valid == 0U))
    {
        reason = DATA_LOGGER_CAMERA_TEST_REASON_LOST;
    }
    else if ((meta->target_valid == 0U) &&
             (ui_task3_camera_test_invalid_frames == 5U))
    {
        reason = DATA_LOGGER_CAMERA_TEST_REASON_LOST;
    }
    else if ((meta->target_valid != 0U) &&
             (meta->confidence_pct < 70U))
    {
        reason = DATA_LOGGER_CAMERA_TEST_REASON_CONFIDENCE;
    }
    else if ((ui_task3_camera_test_have_previous != 0U) &&
             (ui_task3_camera_test_previous_valid != 0U) &&
             (meta->target_valid != 0U))
    {
        center_jump = UI_Task3CameraTestAbsDiffU16(
            meta->center_x,
            ui_task3_camera_test_previous_center_x);
        height_jump = UI_Task3CameraTestAbsDiffU16(
            meta->bbox_height,
            ui_task3_camera_test_previous_bbox_height);
        distance_jump = UI_Task3CameraTestAbsDiffU16(
            meta->distance_cm,
            ui_task3_camera_test_previous_distance_cm);
        if (center_jump > center_limit)
        {
            reason = DATA_LOGGER_CAMERA_TEST_REASON_OFFSET_JUMP;
        }
        else if (height_jump > height_limit)
        {
            reason = DATA_LOGGER_CAMERA_TEST_REASON_BOX_JUMP;
        }
        else if ((meta->distance_valid != 0U) &&
                 (ui_task3_camera_test_previous_distance_cm != 0U) &&
                 (distance_jump > distance_limit))
        {
            reason = DATA_LOGGER_CAMERA_TEST_REASON_DISTANCE_JUMP;
        }
    }

    if ((reason != 0U) &&
        ((reason == DATA_LOGGER_CAMERA_TEST_REASON_MANUAL) ||
         (ui_task3_camera_test_last_event_ms == 0U) ||
         ((uint32)(now_ms - ui_task3_camera_test_last_event_ms) >= 1500U)))
    {
        ret = data_logger_camera_test_frame_request(image, meta, reason);
        ui_task3_camera_test_action_result = ret;
        if (ret == 0U)
        {
            ui_task3_camera_test_last_event_ms = now_ms;
        }
    }

    ui_task3_camera_test_have_previous = 1U;
    ui_task3_camera_test_previous_valid = meta->target_valid;
    ui_task3_camera_test_previous_center_x = meta->center_x;
    ui_task3_camera_test_previous_bbox_height = meta->bbox_height;
    ui_task3_camera_test_previous_distance_cm = meta->distance_cm;
}

static void UI_Task3CameraTestDraw(void)
{
    uint32 bytes = data_logger_camera_test_get_bytes();
    uint8 state = data_logger_camera_test_get_state();
    uint8 mode = data_logger_camera_test_get_mode();
    uint8 error = data_logger_camera_test_get_last_error();
    uint8 action = ui_task3_camera_test_action_result;

    if (action != 0U)
    {
        error = action;
    }
    ips200_show_string(0, 16 * 16, "TEST");
    ips200_show_string(40, 16 * 16,
                       (mode == DATA_LOGGER_CAMERA_TEST_MODE_MOTION) ?
                       "MOTION" : "STATIC");
    ips200_show_string(104, 16 * 16, UI_Task3CameraTestStateName(state));
    ips200_show_string(152, 16 * 16, "e");
    ips200_show_uint(168, 16 * 16, error, 2);

    ips200_show_string(0, 16 * 17, "E");
    ips200_show_uint(16, 16 * 17,
                     data_logger_camera_test_get_event_count(), 3);
    ips200_show_string(56, 16 * 17, "D");
    ips200_show_uint(72, 16 * 17,
                     data_logger_camera_test_get_dropped_count(), 3);
    ips200_show_string(120, 16 * 17, "KB");
    ips200_show_uint(152, 16 * 17, (bytes + 1023U) / 1024U, 5);

    ips200_show_string(0, 16 * 18, "K1 ST K2 MV K3 FOLLOW K4 STOP");
    ips200_show_string(0, 16 * 19, "LAST");
    ips200_show_string(40, 16 * 19,
                       UI_Task3CameraTestReasonName(
                           data_logger_camera_test_get_last_reason()));
}

static void UI_Task3DrawBlueTarget(const blue_target_result_t *target)
{
    uint16 x0;
    uint16 y0;
    uint16 x1;
    uint16 y1;
    uint16 cx;
    uint16 cy;

    if ((target == NULL) || (target->valid == 0U))
    {
        return;
    }

    x0 = (uint16)(40U + target->bbox_x);
    y0 = (uint16)(32U + target->bbox_y);
    x1 = (uint16)(x0 + target->bbox_width - 1U);
    y1 = (uint16)(y0 + target->bbox_height - 1U);
    if (x1 > 199U) x1 = 199U;
    if (y1 > 191U) y1 = 191U;

    ips200_draw_line(x0, y0, x1, y0, RGB565_YELLOW);
    ips200_draw_line(x0, y1, x1, y1, RGB565_YELLOW);
    ips200_draw_line(x0, y0, x0, y1, RGB565_YELLOW);
    ips200_draw_line(x1, y0, x1, y1, RGB565_YELLOW);

    cx = (uint16)(40U + target->center_x);
    cy = (uint16)(32U + target->center_y);
    ips200_draw_line((uint16)(cx - ((cx > 42U) ? 3U : 0U)),
                     cy,
                     (uint16)(cx + ((cx < 197U) ? 3U : 0U)),
                     cy,
                     RGB565_RED);
    ips200_draw_line(cx,
                     (uint16)(cy - ((cy > 34U) ? 3U : 0U)),
                     cx,
                     (uint16)(cy + ((cy < 189U) ? 3U : 0U)),
                     RGB565_RED);
}

static const char *UI_Task3FollowPhaseName(uint8 phase)
{
    switch (phase)
    {
        case CTRL_TASK3_FOLLOW_PHASE_WAIT:
            return "WAIT";
        case CTRL_TASK3_FOLLOW_PHASE_TRACK:
            return "RUN ";
        case CTRL_TASK3_FOLLOW_PHASE_LOST:
            return "LOST";
        case CTRL_TASK3_FOLLOW_PHASE_STOPPING:
            return "STOP";
        case CTRL_TASK3_FOLLOW_PHASE_SAVE_FAIL:
            return "FAIL";
        default:
            return "IDLE";
    }
}

static void UI_Task3ColorCameraPage(void)
{
    uint32 now_ms = system_getval_ms();
    uint32 dt_ms;
    uint32 worker_frame_count;
    uint32 vsync_frame_count;
    uint32 dma_frame_count;
    uint32 dma_lost_count;
    uint32 preview_copy_us = 0U;
    uint32 record_dt_ms;
    uint32 record_saved_frames;
    uint32 storage_bytes;
    uint32 vision_load_pct;
    uint32 record_period_ms;
    uint32 record_frames;
    uint32 record_dropped;
    uint8 snapshot_state;
    uint8 record_state;
    uint8 display_error;
    uint8 target_fps;
    uint8 request_ret;
    uint8 stats_due = 0U;
    uint8 preview_locked = 0U;
    data_logger_camera_frame_meta_t frame_meta;
    blue_target_result_t target;
    const uint16 *preview_image = NULL;
    uint32 preview_frame_count = 0U;

    if (Page_Num == GPS_SetPage)
    {
        if (ui_task3_camera_test_page_entered == 0U)
        {
            ui_task3_camera_test_page_entered = 1U;
            ui_color_camera_snapshot_request = 0U;
            if (data_logger_camera_record_is_active() != 0U)
            {
                data_logger_camera_record_stop();
                printf("[CAMERA_TEST] stop previous camera record on test page\r\n");
            }
        }
    }
    else
    {
        ui_task3_camera_test_page_entered = 0U;
    }

    blue_target_get_result(&target);
    if (guimai_voice_is_recording() != 0U)
    {
        if (data_logger_camera_record_is_active() != 0U)
        {
            data_logger_camera_record_stop();
        }
        if (data_logger_camera_test_is_active() != 0U)
        {
            data_logger_camera_test_stop();
        }
        ips200_show_string(0, 16 * 0, "T3-ColorCam");
        ips200_show_string(0, 16 * 2, "VOICE BUSY");
        ips200_show_string(0, 16 * 3, "record stopped");
        return;
    }

    if (0U == ui_color_camera_page_entered)
    {
        ui_color_camera_page_entered = 1U;
        ui_color_camera_last_page = Page_Num;
        ui_color_camera_fps = 0U;
        ui_color_camera_fps_last_ms = now_ms;
        ui_color_camera_fps_frame_base = blue_target_camera_get_frame_count();
        ui_color_camera_vsync_fps = 0U;
        ui_color_camera_vsync_frame_base = scc8660_get_vsync_count();
        ui_color_camera_dma_fps = 0U;
        ui_color_camera_dma_frame_base = scc8660_get_dma_frame_count();
        ui_color_camera_preview_frame_count = 0U;
        ui_color_camera_record_last_request_ms = 0U;
        ui_color_camera_record_fps = 0U;
        ui_color_camera_record_fps_last_ms = now_ms;
        ui_color_camera_record_fps_frame_base =
            data_logger_camera_record_get_frame_count();
        ui_color_camera_stats_last_ms = now_ms;
        ui_color_camera_snapshot_request = 0U;
        stats_due = 1U;
        if (ctrl_task3_follow_is_active() == 0U)
        {
            blue_target_reset();
        }

        ui_color_camera_init_ok =
            (blue_target_camera_start() == 0U) ? 1U : 0U;
    }
    else if (ui_color_camera_last_page != Page_Num)
    {
        ui_color_camera_last_page = Page_Num;
        ui_color_camera_fps = 0U;
        ui_color_camera_fps_last_ms = now_ms;
        ui_color_camera_fps_frame_base = blue_target_camera_get_frame_count();
        ui_color_camera_vsync_fps = 0U;
        ui_color_camera_vsync_frame_base = scc8660_get_vsync_count();
        ui_color_camera_dma_fps = 0U;
        ui_color_camera_dma_frame_base = scc8660_get_dma_frame_count();
        ui_color_camera_preview_frame_count = 0U;
        ui_color_camera_stats_last_ms = now_ms;
        stats_due = 1U;
    }

    ui_color_camera_init_ok = blue_target_camera_is_ready();

    if ((now_ms - ui_color_camera_stats_last_ms) >=
        UI_TASK3_CAMERA_STATS_PERIOD_MS)
    {
        ui_color_camera_stats_last_ms = now_ms;
        stats_due = 1U;
    }

    worker_frame_count = blue_target_camera_get_frame_count();
    vsync_frame_count = scc8660_get_vsync_count();
    dma_frame_count = scc8660_get_dma_frame_count();
    dma_lost_count = scc8660_get_dma_lost_count();
    dt_ms = now_ms - ui_color_camera_fps_last_ms;
    if (dt_ms >= 1000U)
    {
        ui_color_camera_fps =
            ((worker_frame_count - ui_color_camera_fps_frame_base) * 1000U) / dt_ms;
        ui_color_camera_vsync_fps =
            ((vsync_frame_count - ui_color_camera_vsync_frame_base) * 1000U) / dt_ms;
        ui_color_camera_dma_fps =
            ((dma_frame_count - ui_color_camera_dma_frame_base) * 1000U) / dt_ms;
        ui_color_camera_fps_last_ms = now_ms;
        ui_color_camera_fps_frame_base = worker_frame_count;
        ui_color_camera_vsync_frame_base = vsync_frame_count;
        ui_color_camera_dma_frame_base = dma_frame_count;
    }
    blue_target_camera_get_timing(&preview_copy_us, NULL);

    if (stats_due != 0U)
    {
        ips200_show_string(0, 16 * 0, "C");
        ips200_show_uint(16, 16 * 0, UI_CameraConfiguredFps(), 2);
        ips200_show_string(48, 16 * 0, "V");
        ips200_show_uint(64, 16 * 0, ui_color_camera_vsync_fps, 2);
        ips200_show_string(96, 16 * 0, "D");
        ips200_show_uint(112, 16 * 0, ui_color_camera_dma_fps, 2);
        ips200_show_string(144, 16 * 0, "P");
        ips200_show_uint(160, 16 * 0, ui_color_camera_fps, 2);
        ips200_show_string(192, 16 * 0, "E");
        ips200_show_uint(208, 16 * 0, dma_lost_count, 4);
    }

    if (0U == ui_color_camera_init_ok)
    {
        ips200_show_string(0, 16 * 2, "SCC8660 init fail");
        ips200_show_string(0, 16 * 3, "check P00/P02 wire");
        return;
    }

    if (ui_color_camera_init_ok != 0U)
    {
        preview_locked = blue_target_preview_acquire(&preview_image,
                                                     &target,
                                                     &preview_frame_count);
        if (preview_locked != 0U)
        {
            if (preview_frame_count == ui_color_camera_preview_frame_count)
            {
                blue_target_preview_release();
                preview_locked = 0U;
            }
        }
        if (preview_locked != 0U)
        {
            ui_color_camera_preview_frame_count = preview_frame_count;
            vision_load_pct =
                (((uint32)target.process_us * ui_color_camera_fps) + 5000U) / 10000U;
            UI_Task3CameraBuildMeta(&frame_meta, &target, now_ms, vision_load_pct);

            if ((Page_Num == GPS_SetPage) ||
                ((ctrl_task3_follow_is_active() != 0U) &&
                 (data_logger_camera_test_is_active() != 0U)))
            {
                UI_Task3CameraTestUpdate(preview_image, &frame_meta);
            }

            if (0U != ui_color_camera_snapshot_request)
            {
                ui_color_camera_action_last_result =
                    data_logger_camera_snapshot_request_with_meta(
                        (data_logger_camera_snapshot_size_t)ui_color_camera_snapshot_size,
                        preview_image,
                        &frame_meta);
                ui_color_camera_snapshot_request = 0U;
            }

            record_state = data_logger_camera_record_get_state();
            if ((ui_color_camera_capture_mode == UI_TASK3_CAMERA_MODE_RECORD) &&
                (record_state == (uint8)DATA_LOGGER_CAMERA_RECORD_RUNNING))
            {
                target_fps = data_logger_camera_record_get_target_fps();
                if (target_fps == 0U)
                {
                    target_fps = UI_CameraRecordFps(ui_color_camera_snapshot_size);
                }
                record_period_ms = (1000U + ((uint32)target_fps / 2U)) / target_fps;
                if ((ui_color_camera_record_last_request_ms == 0U) ||
                    ((now_ms - ui_color_camera_record_last_request_ms) >= record_period_ms))
                {
                    request_ret =
                        data_logger_camera_record_frame_request(preview_image, &frame_meta);
                    ui_color_camera_record_last_request_ms = now_ms;
                    if ((request_ret != 0U) && (request_ret != 6U))
                    {
                        ui_color_camera_action_last_result = request_ret;
                    }
                }
            }

            ips200_show_rgb565_image(40,
                                      32,
                                      preview_image,
                                      SCC8660_W,
                                      SCC8660_H,
                                      SCC8660_W,
                                      SCC8660_H,
                                      1);
            UI_Task3DrawBlueTarget(&target);
            blue_target_preview_release();
        }
    }

    snapshot_state = data_logger_camera_snapshot_get_state();
    record_state = data_logger_camera_record_get_state();
    record_frames = data_logger_camera_record_get_frame_count();
    record_dropped = data_logger_camera_record_get_dropped_count();
    target_fps = UI_CameraRecordFps(ui_color_camera_snapshot_size);
    vision_load_pct =
        (((uint32)target.process_us * ui_color_camera_fps) + 5000U) / 10000U;

    if (record_frames < ui_color_camera_record_fps_frame_base)
    {
        ui_color_camera_record_fps = 0U;
        ui_color_camera_record_fps_last_ms = now_ms;
        ui_color_camera_record_fps_frame_base = record_frames;
    }
    record_dt_ms = now_ms - ui_color_camera_record_fps_last_ms;
    if (record_dt_ms >= 1000U)
    {
        record_saved_frames =
            record_frames - ui_color_camera_record_fps_frame_base;
        ui_color_camera_record_fps =
            (record_saved_frames * 1000U) / record_dt_ms;
        ui_color_camera_record_fps_last_ms = now_ms;
        ui_color_camera_record_fps_frame_base = record_frames;
    }

    if (stats_due == 0U)
    {
        return;
    }

    ips200_show_string(0, 16 * 13, "ok");
    ips200_show_uint(24, 16 * 13, target.valid, 1);
    ips200_show_string(48, 16 * 13, "cf");
    ips200_show_uint(72, 16 * 13, target.confidence_pct, 3);
    ips200_show_string(104, 16 * 13, "us");
    ips200_show_uint(128, 16 * 13, target.process_us, 5);
    ips200_show_string(176, 16 * 13, "cp");
    ips200_show_uint(200, 16 * 13, preview_copy_us, 5);

    ips200_show_string(0, 16 * 14, "cx");
    ips200_show_uint(24, 16 * 14, target.center_x, 3);
    ips200_show_string(56, 16 * 14, "dx");
    ips200_show_int(80, 16 * 14, target.offset_x, 3);
    ips200_show_string(128, 16 * 14, "ang");
    ips200_show_float(160, 16 * 14, (double)target.bearing_deg_x10 / 10.0, 3, 1);

    ips200_show_string(0, 16 * 15, "w");
    ips200_show_uint(16, 16 * 15, target.bbox_width, 3);
    ips200_show_string(48, 16 * 15, "h");
    ips200_show_uint(64, 16 * 15, target.bbox_height, 3);
    ips200_show_string(96, 16 * 15, "A");
    ips200_show_uint(112, 16 * 15, target.area_px, 5);
    ips200_show_string(160, 16 * 15, "d         ");
    if (target.distance_valid != 0U)
    {
        ips200_show_float(176,
                          16 * 15,
                          (double)target.distance_cm / 100.0,
                          1,
                          2);
        ips200_show_string(216, 16 * 15, "m");
    }
    else
    {
        ips200_show_string(176, 16 * 15, "---");
    }

    if (ui_color_camera_capture_mode == UI_TASK3_CAMERA_MODE_RECORD)
    {
        display_error = data_logger_camera_record_get_last_error();
        if (ui_color_camera_action_last_result != 0U)
        {
            display_error = ui_color_camera_action_last_result;
        }
        storage_bytes = data_logger_camera_record_get_bytes();

        ips200_show_string(0, 16 * 16, "REC");
        ips200_show_string(32, 16 * 16,
                           UI_CameraSnapshotSizeName(ui_color_camera_snapshot_size));
        ips200_show_string(72, 16 * 16, UI_CameraRecordStateName(record_state));
        ips200_show_string(120, 16 * 16, "e");
        ips200_show_uint(136, 16 * 16, display_error, 2);
        ips200_show_string(168, 16 * 16, "SD");
        ips200_show_string(192, 16 * 16, "ON");

        ips200_show_string(0, 16 * 17, "F");
        ips200_show_uint(16, 16 * 17, record_frames, 6);
        ips200_show_string(72, 16 * 17, "D");
        ips200_show_uint(88, 16 * 17, record_dropped, 5);
        ips200_show_uint(136, 16 * 17, (storage_bytes + 1023U) / 1024U, 6);
        ips200_show_string(192, 16 * 17, "KB");

        ips200_show_string(0, 16 * 18, "K1");
        ips200_show_string(24, 16 * 18,
                           UI_CameraSnapshotSizeName(ui_color_camera_snapshot_size));
        ips200_show_string(64, 16 * 18, "/");
        ips200_show_uint(72, 16 * 18, target_fps, 2);
        ips200_show_string(96, 16 * 18, "K2");
        ips200_show_string(120, 16 * 18,
                           (data_logger_camera_record_is_active() != 0U) ?
                           "STOP " : "START");
        ips200_show_string(168, 16 * 18, "K3");
        ips200_show_string(192, 16 * 18, "PIC");
    }
    else
    {
        display_error = data_logger_camera_snapshot_get_last_error();
        if (ui_color_camera_action_last_result != 0U)
        {
            display_error = ui_color_camera_action_last_result;
        }
        storage_bytes = data_logger_camera_snapshot_get_bytes();

        ips200_show_string(0, 16 * 16, "PIC");
        ips200_show_string(32, 16 * 16,
                           UI_CameraSnapshotSizeName(ui_color_camera_snapshot_size));
        ips200_show_string(72, 16 * 16, UI_CameraSnapshotStateName(snapshot_state));
        ips200_show_string(120, 16 * 16, "e");
        ips200_show_uint(136, 16 * 16, display_error, 2);
        ips200_show_string(168, 16 * 16, "v");
        ips200_show_uint(184, 16 * 16, guimai_voice_is_recording(), 1);

        ips200_show_string(0, 16 * 17, "SD");
        ips200_show_uint(24, 16 * 17, (storage_bytes + 1023U) / 1024U, 6);
        ips200_show_string(80, 16 * 17, "KB");
        ips200_show_string(120, 16 * 17, "BOX BMP");

        ips200_show_string(0, 16 * 18, "K1");
        ips200_show_string(24, 16 * 18,
                           UI_CameraSnapshotSizeName(ui_color_camera_snapshot_size));
        ips200_show_string(72, 16 * 18, "K2 SAVE");
        ips200_show_string(152, 16 * 18, "K3 REC");
    }

    if (Page_Num == SetPage)
    {
        ctrl_task3_follow_status_t follow_status;

        ctrl_task3_follow_get_status(&follow_status);
        if ((follow_status.active != 0U) &&
            (follow_status.record_path == 0U))
        {
            display_error = data_logger_camera_test_get_last_error();
            if (ui_task3_camera_test_action_result != 0U)
            {
                display_error = ui_task3_camera_test_action_result;
            }
            ips200_show_string(0, 16 * 16, "                              ");
            ips200_show_string(0, 16 * 16, "ERRCAP");
            ips200_show_string(56, 16 * 16, "MOTION");
            ips200_show_string(112, 16 * 16,
                               UI_Task3CameraTestStateName(
                                   data_logger_camera_test_get_state()));
            ips200_show_string(168, 16 * 16, "e");
            ips200_show_uint(184, 16 * 16, display_error, 2);
            ips200_show_string(0, 16 * 17, "                              ");
            ips200_show_string(0, 16 * 17, "E");
            ips200_show_uint(16, 16 * 17,
                             data_logger_camera_test_get_event_count(), 3);
            ips200_show_string(64, 16 * 17, "D");
            ips200_show_uint(80, 16 * 17,
                             data_logger_camera_test_get_dropped_count(), 3);
            ips200_show_string(128, 16 * 17, "KB");
            ips200_show_uint(152, 16 * 17,
                             (data_logger_camera_test_get_bytes() + 1023U) /
                             1024U,
                             5);
        }
        ips200_show_string(0, 16 * 18, "                              ");
        ips200_show_string(0, 16 * 18, "F");
        ips200_show_string(16, 16 * 18,
                           UI_Task3FollowPhaseName(follow_status.phase));
        ips200_show_string(56, 16 * 18, "e");
        ips200_show_uint(72, 16 * 18, follow_status.error, 1);
        ips200_show_string(88, 16 * 18, "SD");
        ips200_show_uint(112, 16 * 18, data_logger_is_busy(), 1);
        ips200_show_string(136, 16 * 18, "M");
        ips200_show_string(152, 16 * 18,
                           (follow_status.active == 0U) ? "----" :
                           ((follow_status.record_path != 0U) ? "PATH" : "TEST"));
        ips200_show_string(192, 16 * 18,
                           (follow_status.active != 0U) ? "RUN " : "IDLE");
        ips200_show_string(0, 16 * 19, "K2 PATH  K3 FOLLOW TEST");
    }
    if (Page_Num == GPS_SetPage)
    {
        UI_Task3CameraTestDraw();
    }
}
static const char *UI_PathModeName(nav_path_mode_t mode)
{
    switch (mode)
    {
        case NAV_PATH_MODE_RECORD:
            return "REC";
        case NAV_PATH_MODE_REPLAY:
            return "REPLAY";
        default:
            return "IDLE";
    }
}

static const char *UI_PathTaskTitle(uint8 task_id)
{
    if (task_id == 1U)
    {
        return "T1-PathRec";
    }
    if (task_id == 2U)
    {
        return "T2-PathRec";
    }
    return "T3-PathRec";
}

static const char *UI_PathTaskFileName(uint8 task_id)
{
    if (task_id == 1U)
    {
        return "File: TASK1/PATH_T1.CSV";
    }
    if (task_id == 2U)
    {
        return "File: PATH_T2.CSV";
    }
    return "File: PATH_T3.CSV";
}

static void UI_PathRecordPage(void)
{
    nav_path_state_t path_state;
    nav_ins_state_t ins_state;
    uint32 now_ms = system_getval_ms();
    uint8 task_id = ui_task_get();
    float yaw_delta_180_deg;

    nav_control_path_get_state(&path_state);
    nav_ins_get_state(&ins_state);
    yaw_delta_180_deg = UI_Wrap180Deg(ins_state.yaw_deg - 180.0f);

    ips200_show_string(0, 16 * 0, UI_PathTaskTitle(task_id));
    ips200_show_string(0, 16 * 1, "K1 load K2 rec K3 dir");

    ips200_show_string(0, 16 * 2, "Mode");
    ips200_show_string(56, 16 * 2, UI_PathModeName(path_state.mode));
    ips200_show_string(128, 16 * 2, "act");
    ips200_show_uint(160, 16 * 2, path_state.active, 1);

    ips200_show_string(0, 16 * 3, "Samples");
    ips200_show_uint(72, 16 * 3, path_state.sample_count, 4);
    ips200_show_string(128, 16 * 3, "valid");
    ips200_show_uint(176, 16 * 3, path_state.valid, 1);

    ips200_show_string(0, 16 * 4, "Dist");
    ips200_show_float(56, 16 * 4, path_state.progress_m, 4, 2);
    ips200_show_string(128, 16 * 4, "Int");
    ips200_show_float(168, 16 * 4, path_state.sample_interval_m, 1, 2);

    if (task_id == 3U)
    {
        ips200_show_string(0, 16 * 5, "Yaw");
        ips200_show_float(40, 16 * 5, ins_state.yaw_deg, 3, 1);
        ips200_show_string(112, 16 * 5, "D180");
        ips200_show_float(160, 16 * 5, yaw_delta_180_deg, 4, 1);
    }
    else
    {
        ips200_show_string(0, 16 * 5, "RefYaw");
        ips200_show_float(72, 16 * 5, path_state.ref_yaw_deg, 3, 1);
        ips200_show_string(128, 16 * 5, "Err");
        ips200_show_float(168, 16 * 5, path_state.yaw_error_deg, 3, 1);
    }

    ips200_show_string(0, 16 * 6, "SD");
    ips200_show_uint(32, 16 * 6, path_state.last_sd_result, 3);
    ips200_show_string(80, 16 * 6, "FSave");
    ips200_show_uint(136, 16 * 6, path_state.last_flash_result, 3);
    ips200_show_string(0, 16 * 7, "FLoad");
    ips200_show_uint(64, 16 * 7, path_state.last_flash_load_result, 3);

    ips200_show_string(0, 16 * 8, "0=OK 255=not tried");
    ips200_show_string(0, 16 * 9, UI_PathTaskFileName(task_id));
    if (task_id == 1U)
    {
        ips200_show_string(0, 16 * 10, "Replay");
        ips200_show_string(72, 16 * 10,
                           (ctrl_task1_reverse_get() != 0U) ? "BACK" : "FWD");
        ips200_show_string(136, 16 * 10, "K4 clr");
    }

    if ((path_state.active != 0U) && (path_state.mode == NAV_PATH_MODE_RECORD))
    {
        ui_path_trace_drawn = 0U;
        ips200_show_string(0, 16 * 10, "Preview after stop");
        return;
    }

    if (g_task_ctrl_state == TASK_CTRL_STATE_RUNNING)
    {
        ips200_show_string(0, 16 * 10, "Map paused: control RUN");
        return;
    }

    if ((ui_path_trace_drawn == 0U) ||
        ((ui_path_trace_last_count != path_state.sample_count) &&
         ((now_ms - ui_path_trace_last_draw_ms) >= 200U)))
    {
        UI_PathTraceBuild(&path_state);
        UI_TraceDraw(ui_path_trace_x, ui_path_trace_y, ui_path_trace_count, "K2 start path record");
        ui_path_trace_last_draw_ms = now_ms;
        ui_path_trace_drawn = 1U;
    }
}

static void UI_Task2PointPage(void)
{
    uint8 i;

    ips200_show_string(0, 16 * 0, "T2-POINT Pg3");
    ips200_show_string(144, 16 * 0, "X/Y m");
    ips200_show_string(0, 16 * 1, "Name");
    ips200_show_string(48, 16 * 1, "X");
    ips200_show_string(136, 16 * 1, "Y");
    ips200_show_string(216, 16 * 1, "V");

    for (i = 0U; i < UI_TASK2_POINT_COUNT; i++)
    {
        uint16 y = (uint16)(16U * (uint16)(i + 2U));
        ips200_show_string(0, y, ui_task2_points[i].name);
        if (ui_task2_points[i].valid != 0U)
        {
            ips200_show_float(40, y, ui_task2_points[i].x_m, 2, 2);
            ips200_show_float(128, y, ui_task2_points[i].y_m, 2, 2);
            ips200_show_uint(216, y, 1U, 1);
        }
        else
        {
            ips200_show_string(40, y, "--");
            ips200_show_string(128, y, "--");
            ips200_show_uint(216, y, 0U, 1);
        }
    }

    ips200_show_string(0, 16 * 11, "K1 Go");
    ips200_show_string(72, 16 * 11, Key_Task2DebugGetGoName());
    ips200_show_uint(120, 16 * 11, Key_Task2DebugGetGoCmd(), 3);

    ips200_show_string(0, 16 * 12, "K2 Ret");
    ips200_show_string(72, 16 * 12, Key_Task2DebugGetRetName());
    ips200_show_uint(120, 16 * 12, Key_Task2DebugGetRetCmd(), 3);

    ips200_show_string(0, 16 * 13, "K3 Act");
    ips200_show_string(72, 16 * 13, Key_Task2DebugGetActionName());
    ips200_show_uint(120, 16 * 13, Key_Task2DebugGetActionCmd(), 3);
    ips200_show_string(160, 16 * 13, "GO>A>RET");

    ips200_show_string(0, 16 * 14, "K4");
    ips200_show_string(40, 16 * 14, (Key_Task2DebugIsBusy() != 0U) ? "STOP" : "START");
    ips200_show_string(112, 16 * 14, "Last");
    ips200_show_string(168, 16 * 14, Key_Task2DebugGetLastName());
}

static void UI_Param(void)
{
    const ui_task_display_cfg_t *cfg = UI_GetDisplayCfg();

    if (ui_task_get() == 0U)
    {
        UI_Task0_InsPage();
        return;
    }
    if (ui_task_get() == 1U)
    {
        UI_Task1ImuPage();
        return;
    }
    if (ui_task_get() == 2U)
    {
        UI_Task2PointPage();
        return;
    }
    if (ui_task_get() == 3U)
    {
        UI_PathRecordPage();
        return;
    }
    if (ui_task_get() == 4U)
    {
        ips200_show_string(0, 16 * 0, "T4-LIGHT TEST");
        ips200_show_string(0, 16 * 1, "K1 ALL ON");
        ips200_show_string(0, 16 * 2, "K2 LOOP");
        ips200_show_string(0, 16 * 3, "TURN HAZARD");
        ips200_show_string(0, 16 * 4, "LOW HIGH FOG");
        ips200_show_string(0, 16 * 5, "INTERIOR");
        return;
    }
    ips200_show_string(0, 16 * 1, cfg->param_title);
    ips200_show_string(0, 16 * 2, cfg->param_item1);
    ips200_show_uint(120, 16 * 2, camera_exposure, 4);
    ips200_show_string(0, 16 * 3, cfg->param_item2);
    ips200_show_uint(120, 16 * 3, Camera_compare, 3);
}

static void UI_GPS_Point(void)
{
    const ui_task_display_cfg_t *cfg = UI_GetDisplayCfg();
    const uint8 point_num = UI_Image_GetPointCount();
    const gps_point *points = UI_Image_GetPoints();
    nav_gps_sample_t gps_sample;

    nav_gps_get_last_sample(&gps_sample);

    ips200_show_string(0, 16 * 0, cfg->gps_title);
    ips200_show_string(0, 16 * 1, "state");
    ips200_show_uint(48, 16 * 1, gnss.state, 1);
    ips200_show_string(80, 16 * 1, "sat");
    ips200_show_uint(112, 16 * 1, gnss.satellite_used, 2);
    ips200_show_string(150, 16 * 1, "valid");
    ips200_show_uint(198, 16 * 1, gps_sample.valid, 1);
    ips200_show_string(0, 16 * 2, "now lat");
    ips200_show_float(90, 16 * 2, gnss.latitude, 3, 6);
    ips200_show_string(0, 16 * 3, "now lon");
    ips200_show_float(90, 16 * 3, gnss.longitude, 3, 6);
    ips200_show_string(0, 16 * 4, "local x");
    ips200_show_string(0, 16 * 5, "local y");
    if (0U != gps_sample.valid)
    {
        ips200_show_float(90, 16 * 4, gps_sample.x_m, 4, 2);
        ips200_show_float(90, 16 * 5, gps_sample.y_m, 4, 2);
    }
    else
    {
        ips200_show_string(90, 16 * 4, "--");
        ips200_show_string(90, 16 * 5, "--");
    }
    ips200_show_string(0, 16 * 6, "point num");
    ips200_show_uint(90, 16 * 6, point_num, 2);
    if (UI_Image_IsPicking())
    {
        ips200_show_string(0, 16 * 9, "picking");
        ips200_show_uint(90, 16 * 9, UI_Image_GetPickCount(), 2);
        ips200_show_string(110, 16 * 9, "/");
        ips200_show_uint(122, 16 * 9, UI_Image_GetPickTarget(), 2);
        if (0U == gps_sample.valid)
        {
            ips200_show_string(150, 16 * 9, "wait GPS");
        }
    }

    if (point_num > 0)
    {
        ips200_show_string(0, 16 * 7, "last lat");
        ips200_show_float(90, 16 * 7, points[point_num - 1].lat, 3, 6);
        ips200_show_string(0, 16 * 8, "last lon");
        ips200_show_float(90, 16 * 8, points[point_num - 1].lon, 3, 6);
    }
    else
    {
        ips200_show_string(0, 16 * 7, "last lat --");
        ips200_show_string(0, 16 * 8, "last lon --");
    }
}

static void UI_GpsTest_PickClear(void)
{
    ui_gps_test_pick_active = 0U;
    ui_gps_test_pick_count = 0U;
    ui_gps_test_pick_last_ms = 0U;
    ui_gps_test_pick_point_id = 0U;
    nav_gps_average_reset(&ui_gps_test_pick_avg, UI_GPS_TEST_PICK_TARGET);
}

static void UI_GpsTest_PickStart(void)
{
    if (ui_gps_test_pick_active != 0U)
    {
        return;
    }

    nav_gps_average_reset(&ui_gps_test_pick_avg, UI_GPS_TEST_PICK_TARGET);
    ui_gps_test_pick_active = 1U;
    ui_gps_test_pick_count = 0U;
    ui_gps_test_pick_last_ms = 0U;
    if (ui_gps_test_point_id >= 0xFFFFU)
    {
        ui_gps_test_pick_point_id = 1U;
    }
    else
    {
        ui_gps_test_pick_point_id = (uint16)(ui_gps_test_point_id + 1U);
    }
}

static void UI_GpsTest_PickCommit(const nav_gps_sample_t *sample)
{
    if (sample != NULL)
    {
        data_logger_gps_test_mark_point_sample(ui_gps_test_pick_point_id, sample);
        ui_gps_test_point_id = ui_gps_test_pick_point_id;
    }

    ui_gps_test_pick_active = 0U;
    ui_gps_test_pick_count = 0U;
    ui_gps_test_pick_last_ms = 0U;
    nav_gps_average_reset(&ui_gps_test_pick_avg, UI_GPS_TEST_PICK_TARGET);
}

static void UI_GpsTest_ClearStats(void)
{
    UI_GpsTest_PickClear();
    ui_gps_test_first_valid = 0U;
    ui_gps_test_count = 0U;
    ui_gps_test_point_id = 0U;
    ui_gps_test_last_sample_ms = 0U;
    ui_gps_test_first_x_m = 0.0f;
    ui_gps_test_first_y_m = 0.0f;
    ui_gps_test_sum_x_m = 0.0f;
    ui_gps_test_sum_y_m = 0.0f;
    ui_gps_test_sum_err2_m = 0.0f;
    ui_gps_test_last_err_m = 0.0f;
    ui_gps_test_max_err_m = 0.0f;
}

void UI_GpsTest_Start(void)
{
    UI_GpsTest_ClearStats();
    ui_gps_test_active = 1U;
    (void)data_logger_restart_task_src(1U, DATA_LOGGER_PROFILE_GPS_TEST, TARGET_SRC_TASK);
    data_logger_mark(4101U);
}

void UI_GpsTest_Stop(void)
{
    ui_gps_test_active = 0U;
    UI_GpsTest_PickClear();
    if ((data_logger_get_task_id() == 1U) &&
        (data_logger_get_profile() == (uint8)DATA_LOGGER_PROFILE_GPS_TEST))
    {
        data_logger_mark(4102U);
        data_logger_stop_cancel_restart();
    }
}

void UI_GpsTest_Reset(void)
{
    UI_GpsTest_ClearStats();
}

void UI_GpsTest_MarkPoint(void)
{
    if (ui_gps_test_active != 0U)
    {
        UI_GpsTest_PickStart();
    }
}

static void UI_GpsTest_Update(void)
{
    nav_gps_sample_t sample;
    float dx_m;
    float dy_m;
    float err_m;

    if (0U == ui_gps_test_active)
    {
        return;
    }

    nav_gps_get_last_sample(&sample);
    if (0U == sample.valid)
    {
        return;
    }

    if ((ui_gps_test_count != 0U) &&
        (sample.timestamp_ms == ui_gps_test_last_sample_ms))
    {
        return;
    }

    ui_gps_test_last_sample_ms = sample.timestamp_ms;
    if (0U == ui_gps_test_first_valid)
    {
        ui_gps_test_first_valid = 1U;
        ui_gps_test_first_x_m = sample.x_m;
        ui_gps_test_first_y_m = sample.y_m;
    }

    dx_m = sample.x_m - ui_gps_test_first_x_m;
    dy_m = sample.y_m - ui_gps_test_first_y_m;
    err_m = sqrtf(dx_m * dx_m + dy_m * dy_m);

    ui_gps_test_count++;
    ui_gps_test_sum_x_m += sample.x_m;
    ui_gps_test_sum_y_m += sample.y_m;
    ui_gps_test_sum_err2_m += err_m * err_m;
    ui_gps_test_last_err_m = err_m;
    if (err_m > ui_gps_test_max_err_m)
    {
        ui_gps_test_max_err_m = err_m;
    }

    if (ui_gps_test_pick_active != 0U)
    {
        nav_gps_sample_t pick_sample;

        if (sample.timestamp_ms != ui_gps_test_pick_last_ms)
        {
            ui_gps_test_pick_last_ms = sample.timestamp_ms;
            if (0U != nav_gps_average_add_sample(&ui_gps_test_pick_avg, &sample, &pick_sample))
            {
                ui_gps_test_pick_count = (uint8)ui_gps_test_pick_avg.sample_count;
                UI_GpsTest_PickCommit(&pick_sample);
            }
            else
            {
                ui_gps_test_pick_count = (uint8)ui_gps_test_pick_avg.sample_count;
            }
        }
    }
}

static void UI_Task1_GpsTestPage(void)
{
    nav_gps_sample_t gps_sample;
    nav_state_t nav_state;
    float avg_x_m = 0.0f;
    float avg_y_m = 0.0f;
    float rms_m = 0.0f;

    UI_GpsTest_Update();
    nav_gps_get_last_sample(&gps_sample);
    nav_control_get_state(&nav_state);

    if (ui_gps_test_count != 0U)
    {
        avg_x_m = ui_gps_test_sum_x_m / (float)ui_gps_test_count;
        avg_y_m = ui_gps_test_sum_y_m / (float)ui_gps_test_count;
        rms_m = sqrtf(ui_gps_test_sum_err2_m / (float)ui_gps_test_count);
    }

    ips200_show_string(0, 16 * 0, "T1-GPS-Test");
    ips200_show_string(0, 16 * 1, "K1 start K2 stop");
    ips200_show_string(0, 16 * 2, "K3 reset K4 avg10");

    ips200_show_string(0, 16 * 3, "act");
    ips200_show_uint(32, 16 * 3, ui_gps_test_active, 1);
    ips200_show_string(64, 16 * 3, "cnt");
    ips200_show_uint(96, 16 * 3, ui_gps_test_count, 5);
    ips200_show_string(152, 16 * 3, "mark");
    ips200_show_uint(200, 16 * 3, ui_gps_test_point_id, 3);

    ips200_show_string(0, 16 * 4, "sat");
    ips200_show_uint(32, 16 * 4, gps_sample.satellite_used, 2);
    ips200_show_string(64, 16 * 4, "valid");
    ips200_show_uint(112, 16 * 4, gps_sample.valid, 1);
    ips200_show_string(144, 16 * 4, "state");
    ips200_show_uint(192, 16 * 4, gnss.state, 1);

    ips200_show_string(0, 16 * 5, "now x");
    ips200_show_string(0, 16 * 6, "now y");
    if (0U != gps_sample.valid)
    {
        ips200_show_float(64, 16 * 5, gps_sample.x_m, 4, 2);
        ips200_show_float(64, 16 * 6, gps_sample.y_m, 4, 2);
    }
    else
    {
        ips200_show_string(64, 16 * 5, "--");
        ips200_show_string(64, 16 * 6, "--");
    }

    ips200_show_string(0, 16 * 7, "drift");
    ips200_show_float(64, 16 * 7, ui_gps_test_last_err_m, 4, 2);
    ips200_show_string(128, 16 * 7, "max");
    ips200_show_float(168, 16 * 7, ui_gps_test_max_err_m, 3, 2);

    ips200_show_string(0, 16 * 8, "rms");
    ips200_show_float(64, 16 * 8, rms_m, 4, 2);
    ips200_show_string(128, 16 * 8, "gps-ins");
    ips200_show_float(192, 16 * 8, nav_state.gps_ins_error_m, 2, 2);

    ips200_show_string(0, 16 * 9, "avg x");
    ips200_show_float(64, 16 * 9, avg_x_m, 4, 2);
    ips200_show_string(0, 16 * 10, "avg y");
    ips200_show_float(64, 16 * 10, avg_y_m, 4, 2);

    ips200_show_string(0, 16 * 11, "corr");
    ips200_show_uint(48, 16 * 11, nav_state.gps_ins_correction_enabled, 1);
    ips200_show_string(80, 16 * 11, "gain");
    ips200_show_float(128, 16 * 11, nav_state.gps_ins_correction_gain, 1, 2);

    ips200_show_string(0, 16 * 12, "log");
    ips200_show_uint(40, 16 * 12, data_logger_get_state(), 1);
    ips200_show_string(72, 16 * 12, "err");
    ips200_show_uint(112, 16 * 12, data_logger_get_last_error(), 2);
    ips200_show_string(144, 16 * 12, "drop");
    ips200_show_uint(192, 16 * 12, data_logger_get_dropped_count(), 4);

    if (ui_gps_test_pick_active != 0U)
    {
        ips200_show_string(0, 16 * 13, "pick");
        ips200_show_uint(40, 16 * 13, ui_gps_test_pick_count, 2);
        ips200_show_string(72, 16 * 13, "/");
        ips200_show_uint(88, 16 * 13, UI_GPS_TEST_PICK_TARGET, 2);
        ips200_show_string(128, 16 * 13, "pid");
        ips200_show_uint(152, 16 * 13, ui_gps_test_pick_point_id, 3);
    }
    else
    {
        ips200_show_string(0, 16 * 13, "file GPS_TEST CSV");
        ips200_show_uint(176, 16 * 13, ui_gps_test_point_id, 3);
    }
}
static void UI_Task0_CameraPage(void)
{
#if TLD7002_PROJECT_ENABLE
    ips200_show_string(0, 16 * 0, "T0-Camera");
    ips200_show_string(0, 16 * 2, "camera disabled");
    ips200_show_string(0, 16 * 3, "UART1 -> TLD7002");
    return;
#endif

    if (0U == ui_camera_page_entered)
    {
        ui_camera_page_entered = 1U;

        if (0U == ui_camera_init_done)
        {
            ui_camera_init_done = 1U;
            ui_camera_init_ok = (0U == mt9v03x_init()) ? 1U : 0U;
        }

        if (0U != ui_camera_init_ok)
        {
            (void)mt9v03x_set_exposure_time(camera_exposure);
            ui_camera_last_exposure = camera_exposure;
        }
    }

    ips200_show_string(0, 16 * 0, "T0-Camera");

    if (0U == ui_camera_init_ok)
    {
        ips200_show_string(0, 16 * 2, "camera init fail");
        return;
    }

    if (ui_camera_last_exposure != camera_exposure)
    {
        (void)mt9v03x_set_exposure_time(camera_exposure);
        ui_camera_last_exposure = camera_exposure;
    }

    ips200_show_string(0, 16 * 18, "exp");
    ips200_show_uint(40, 16 * 18, camera_exposure, 4);
    ips200_show_string(120, 16 * 18, "thr");
    ips200_show_uint(160, 16 * 18, Camera_compare, 3);

    if (0U != mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        ips200_show_gray_image(0, 16, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 240, 160, 0);
    }
}

void UI_Init(void)
{
    uint8 i;
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();
    Page_Num = InfoPage;
    ui_task_set(UI_TASK_DEFAULT_ID);
    for (i = 0; i < UI_TASK_COUNT; i++)
    {
        g_ui_task_state[i].program_select = 1;
        g_ui_task_state[i].value1 = 1000;
        g_ui_task_state[i].value2 = 200;
    }
    UI_TaskSyncToLegacyVars();
    UI_Image_LoadAllPoints();
    ui_inited = 1;
    Gui_Refersh_Bool = ZF_FALSE;
}

void UI_KeyChange_Callback(uint8 dir)
{
    ui_task_state_t *state = ui_task_curr_state();
    if (program_select == 1)
    {
        if (dir == 0)
        {
            if (camera_exposure > 2) camera_exposure -= 2;
        }
        else
        {
            camera_exposure += 2;
        }
        state->value1 = camera_exposure;
    }
    else if (program_select == 2)
    {
        if (dir == 0)
        {
            if (Camera_compare > 5) Camera_compare -= 5;
        }
        else
        {
            if (Camera_compare < 250) Camera_compare += 5;
        }
        state->value2 = Camera_compare;
    }
}

void UI(void)
{
    ctrl_task3_follow_status_t task3_follow_status;

    if (!ui_inited)
    {
        return;
    }

    ctrl_task3_follow_get_status(&task3_follow_status);
    if ((g_task_ctrl_id == 3U) &&
        (g_task_ctrl_state == TASK_CTRL_STATE_RUNNING) &&
        ((task3_follow_status.active == 0U) ||
         (ui_task_get() != 3U) ||
         ((Page_Num != SetPage) && (Page_Num != GPSPage) &&
          (Page_Num != GPS_SetPage))))
    {
        ui_task3_screen_frozen = 1U;
        return;
    }
    if (ui_task3_screen_frozen != 0U)
    {
        ui_task3_screen_frozen = 0U;
        Gui_Refersh_Bool = ZF_TRUE;
    }

    if (Gui_Refersh_Bool)
    {
        ips200_clear();
        ui_map_drawn = 0U;
        ui_path_trace_drawn = 0U;
        ui_task2_ins_map_drawn = 0U;
        ui_task2_ins_trace_drawn_count = 0U;
        Gui_Refersh_Bool = ZF_FALSE;
    }

    if (UI_Mode != 0)
    {
        return;
    }

    UI_TaskSyncToLegacyVars();
    UI_Image_Task();
    UI_Task1CompareUpdate();

    if (!((ui_task_get() == 1U) && (Page_Num == ParamPage)))
    {
        UI_Task1ImuLoggerLeaveIfNeeded();
    }

    if (!((ui_task_get() == 0U) && (Page_Num == GPS_SetPage)))
    {
        ui_camera_page_entered = 0U;
    }
    if (!((ui_task_get() == 3U) &&
          ((Page_Num == SetPage) || (Page_Num == GPSPage) ||
           (Page_Num == GPS_SetPage))))
    {
        if (ui_color_camera_page_entered != 0U)
        {
            data_logger_camera_record_stop();
            ui_color_camera_snapshot_request = 0U;
        }
        ui_color_camera_page_entered = 0U;
    }
    if (!((ui_task_get() == 3U) &&
          ((Page_Num == GPS_SetPage) ||
           ((task3_follow_status.active != 0U) &&
            (task3_follow_status.record_path == 0U)))) &&
        (data_logger_camera_test_is_active() != 0U))
    {
        data_logger_camera_test_stop();
    }
    if (!((ui_task_get() == 3U) && (Page_Num == GPS_SetPage)))
    {
        ui_task3_camera_test_page_entered = 0U;
    }

    ips200_set_font(IPS200_8X16_FONT);
    switch (Page_Num)
    {
        case InfoPage:
            UI_Config();
            break;
        case SetPage:
            UI_Set();
            break;
        case ParamPage:
            UI_Param();
            break;
        case GPSPage:
            if (ui_task_get() == 3U)
            {
                UI_Task3ColorCameraPage();
            }
            else
            {
                UI_GPS_Point();
                if (UI_MapShouldRedraw(UI_Image_GetPointCount()))
                {
                    ips200_set_font(IPS200_6X8_FONT);
                    user_gps_display_init(SCREEN_IPS200_SPI, 0, 168, 240, 140);
                    user_gps_transition((gps_point *)UI_Image_GetPoints(), UI_Image_GetPointCount());
                    user_gps_display(RGB565_BLUE);
                    ips200_set_font(IPS200_8X16_FONT);
                }
            }
            break;
        case GPS_SetPage:
            if (ui_task_get() == 0U)
            {
                UI_Task0_CameraPage();
            }
            else if (ui_task_get() == 1U)
            {
                UI_Task1_GpsTestPage();
            }
            else if (ui_task_get() == 3U)
            {
                UI_Task3ColorCameraPage();
            }
            else
            {
                ips200_show_string(0, 16 * 0, UI_GetDisplayCfg()->gps_edit_title);
                if (UI_MapShouldRedraw(UI_Image_GetPointCount()))
                {
                    ips200_set_font(IPS200_6X8_FONT);
                    user_gps_display_init(SCREEN_IPS200_SPI, 0, 168, 240, 140);
                    user_gps_transition((gps_point *)UI_Image_GetPoints(), UI_Image_GetPointCount());
                    user_gps_display(RGB565_RED);
                }
                UI_image_show();
            }
            break;
        default:
            break;
    }

    UI_TaskSyncFromLegacyVars();
}
