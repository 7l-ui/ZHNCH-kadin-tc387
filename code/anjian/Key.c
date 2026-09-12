#include "Key.h"
#include "ui_task.h"
#include "isr.h"
#include "task_ctrl.h"
#include "nav_control.h"
#include "nav_path.h"
#include "nav_ins.h"
#include "dici.h"
#include "data_logger.h"
#include "../guimai/guimai_board.h"
#include "../code/yaokong/yaokong.h"
#include "../TLD7002/tld7002_project_config.h"
#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
#include "../TLD7002/zf_device_dot_matrix_screen.h"
#endif

uint8 Key_Num = 0;
volatile uint8 g_key_last_event_num = 0U;
volatile uint32 g_key_last_event_time_ms = 0U;
volatile uint32 g_key_event_count = 0U;
uint8 test_flag = 0;
uint16 BUZZER_NUM = 0;
uint8 program_select = 1;

#define KEY_DEBOUNCE_MS          (30U)
#define KEY_STARTUP_RELEASE_MS   (50U)

static Key_status Status = {0, 0, 0, 0, 0};
static Last_status L_status = {0, 0, 0, 0, 0};
static uint32 g_key_candidate_since_ms[5] = {0U, 0U, 0U, 0U, 0U};
static uint32 g_key_startup_release_since_ms = 0U;
static uint8 g_key_scan_armed = 0U;

static uint8 Key_IsPathTaskPage(void);
static uint8 Key_IsTask1UiOverridePage(void);
static uint8 Key_IsTask2PointPage(void);
static uint8 Key_IsTask3CameraMirrorPage(void);
static uint8 Key_DebounceRelease(uint8 raw,
                                uint8 *candidate,
                                uint8 *stable,
                                uint32 *candidate_since_ms,
                                uint32 now_ms);
static void Key_Task2DebugNextGo(void);
static void Key_Task2DebugNextRet(void);
static void Key_Task2DebugNextAction(void);
static void Key_Task2DebugLaunchOrStop(void);
static const char *Key_PathFileForTask(uint8 task_id);
static uint8 Key_ShouldRecordFullNavLogger(uint8 task_id);
static uint8 Key_Task3RecordAnyKeyReplay(uint8 active_key);
static void Key_PathLoadFlash(void);
static void Key_PathRecordToggle(void);
static void Key_PathClearRam(void);

typedef struct
{
    uint8 cmd;
    const char *name;
} key_task2_debug_cmd_t;

#define KEY_TASK2_CMD_DOOR1_RIGHT_RET   (43U)
#define KEY_TASK2_CMD_DOOR1_RET         (44U)
#define KEY_TASK2_CMD_DOOR2_RET         (45U)
#define KEY_TASK2_CMD_DOOR3_RET         (46U)
#define KEY_TASK2_CMD_DOOR3_LEFT_RET    (47U)
#define KEY_TASK2_CMD_DOOR1_LEFT_PASS   (48U)
#define KEY_TASK2_CMD_DOOR1_PASS        (49U)
#define KEY_TASK2_CMD_DOOR2_PASS        (50U)
#define KEY_TASK2_CMD_DOOR3_PASS        (51U)
#define KEY_TASK2_CMD_DOOR3_RIGHT_PASS  (52U)
#define KEY_TASK2_CMD_FORWARD_10M       (1U)
#define KEY_TASK2_CMD_SNAKE_FORWARD     (9U)

#define KEY_TASK2_DEBUG_GO_COUNT        (5U)
#define KEY_TASK2_DEBUG_RET_COUNT       (5U)
#define KEY_TASK2_DEBUG_ACTION_COUNT    (2U)
#define KEY_TASK3_RECORD_STOP_YAW_DEG   (180.0f)

#define KEY_TASK2_DEBUG_RESULT_READY    (0U)
#define KEY_TASK2_DEBUG_RESULT_START    (1U)
#define KEY_TASK2_DEBUG_RESULT_STOP     (2U)
#define KEY_TASK2_DEBUG_RESULT_PUSHERR  (3U)
#define KEY_TASK2_DEBUG_RESULT_STARTERR (4U)

#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
#define KEY_TASK4_LIGHT_TEST_PERIOD_MS  (1000U)
#define KEY_TASK4_LIGHT_TEST_PATTERN_COUNT (7U)

static const dot_matrix_screen_pattern_enum g_task4_light_test_patterns[KEY_TASK4_LIGHT_TEST_PATTERN_COUNT] =
{
    DOT_MATRIX_SCREEN_PATTERN_LEFT_TURN,
    DOT_MATRIX_SCREEN_PATTERN_RIGHT_TURN,
    DOT_MATRIX_SCREEN_PATTERN_DOUBLE_FLASH,
    DOT_MATRIX_SCREEN_PATTERN_LOW_BEAM,
    DOT_MATRIX_SCREEN_PATTERN_HIGH_BEAM,
    DOT_MATRIX_SCREEN_PATTERN_FOG_LIGHT,
    DOT_MATRIX_SCREEN_PATTERN_INTERIOR_LIGHT
};

static uint8 g_task4_light_test_active = 0U;
static uint8 g_task4_light_test_owned = 0U;
static uint8 g_task4_light_test_pattern_index = 0U;
static uint32 g_task4_light_test_next_ms = 0U;

static uint8 Key_IsTask4LightTestPage(void);
static void Key_Task4LightTestStart(void);
static void Key_Task4LightTestAllOn(void);
static void Key_Task4LightTestTask(void);
#endif

static const key_task2_debug_cmd_t g_task2_debug_go_cmds[KEY_TASK2_DEBUG_GO_COUNT] =
{
    {KEY_TASK2_CMD_DOOR1_LEFT_PASS,  "G1L"},
    {KEY_TASK2_CMD_DOOR1_PASS,       "G1"},
    {KEY_TASK2_CMD_DOOR2_PASS,       "G2"},
    {KEY_TASK2_CMD_DOOR3_PASS,       "G3"},
    {KEY_TASK2_CMD_DOOR3_RIGHT_PASS, "G3R"},
};

static const key_task2_debug_cmd_t g_task2_debug_ret_cmds[KEY_TASK2_DEBUG_RET_COUNT] =
{
    {KEY_TASK2_CMD_DOOR1_RIGHT_RET, "G1R"},
    {KEY_TASK2_CMD_DOOR1_RET,       "G1"},
    {KEY_TASK2_CMD_DOOR2_RET,       "G2"},
    {KEY_TASK2_CMD_DOOR3_RET,       "G3"},
    {KEY_TASK2_CMD_DOOR3_LEFT_RET,  "G3L"},
};

static const key_task2_debug_cmd_t g_task2_debug_action_cmds[KEY_TASK2_DEBUG_ACTION_COUNT] =
{
    {KEY_TASK2_CMD_FORWARD_10M,   "FWD"},
    {KEY_TASK2_CMD_SNAKE_FORWARD, "SNK"},
};

static uint8 g_task2_debug_go_index = 2U;
static uint8 g_task2_debug_ret_index = 2U;
static uint8 g_task2_debug_action_index = 0U;
static uint8 g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_READY;

static const key_task2_debug_cmd_t *Key_Task2DebugGoItem(void)
{
    if (g_task2_debug_go_index >= KEY_TASK2_DEBUG_GO_COUNT)
    {
        g_task2_debug_go_index = 0U;
    }

    return &g_task2_debug_go_cmds[g_task2_debug_go_index];
}

static const key_task2_debug_cmd_t *Key_Task2DebugRetItem(void)
{
    if (g_task2_debug_ret_index >= KEY_TASK2_DEBUG_RET_COUNT)
    {
        g_task2_debug_ret_index = 0U;
    }

    return &g_task2_debug_ret_cmds[g_task2_debug_ret_index];
}

static const key_task2_debug_cmd_t *Key_Task2DebugActionItem(void)
{
    if (g_task2_debug_action_index >= KEY_TASK2_DEBUG_ACTION_COUNT)
    {
        g_task2_debug_action_index = 0U;
    }

    return &g_task2_debug_action_cmds[g_task2_debug_action_index];
}

static uint8 Key_Task2DebugIsTaskBusy(void)
{
    return ((g_task_ctrl_id == 2U) &&
            (ctrl_task2_sequence_is_busy() != 0U)) ? 1U : 0U;
}

uint8 Key_Task2DebugGetGoCmd(void)
{
    return Key_Task2DebugGoItem()->cmd;
}

const char *Key_Task2DebugGetGoName(void)
{
    return Key_Task2DebugGoItem()->name;
}

uint8 Key_Task2DebugGetRetCmd(void)
{
    return Key_Task2DebugRetItem()->cmd;
}

const char *Key_Task2DebugGetRetName(void)
{
    return Key_Task2DebugRetItem()->name;
}

uint8 Key_Task2DebugGetActionCmd(void)
{
    return Key_Task2DebugActionItem()->cmd;
}

const char *Key_Task2DebugGetActionName(void)
{
    return Key_Task2DebugActionItem()->name;
}

const char *Key_Task2DebugGetLastName(void)
{
    switch (g_task2_debug_last_result)
    {
        case KEY_TASK2_DEBUG_RESULT_START:
            return "START";
        case KEY_TASK2_DEBUG_RESULT_STOP:
            return "STOP";
        case KEY_TASK2_DEBUG_RESULT_PUSHERR:
            return "PUSHERR";
        case KEY_TASK2_DEBUG_RESULT_STARTERR:
            return "STARTERR";
        default:
            return "READY";
    }
}

uint8 Key_Task2DebugIsBusy(void)
{
    return Key_Task2DebugIsTaskBusy();
}

static uint8 Key_Task2DebugPushCmd(uint8 cmd)
{
    return (ctrl_task2_sequence_push(cmd) != 0U) ? 1U : 0U;
}

static uint8 Key_Task2DebugQueueSelected(void)
{
    uint8 ok = 1U;

    if (Key_Task2DebugPushCmd(Key_Task2DebugGetGoCmd()) == 0U)
    {
        ok = 0U;
    }
    if (Key_Task2DebugPushCmd(Key_Task2DebugGetActionCmd()) == 0U)
    {
        ok = 0U;
    }
    if (Key_Task2DebugPushCmd(Key_Task2DebugGetRetCmd()) == 0U)
    {
        ok = 0U;
    }

    return ok;
}

static void Key_Task2DebugNextGo(void)
{
    g_task2_debug_go_index = (uint8)((g_task2_debug_go_index + 1U) % KEY_TASK2_DEBUG_GO_COUNT);
    g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_READY;
    printf("[TASK2_DBG] go=%s cmd=%u\r\n",
           Key_Task2DebugGetGoName(),
           (unsigned)Key_Task2DebugGetGoCmd());
    Gui_Refersh_Bool = ZF_TRUE;
}

static void Key_Task2DebugNextRet(void)
{
    g_task2_debug_ret_index = (uint8)((g_task2_debug_ret_index + 1U) % KEY_TASK2_DEBUG_RET_COUNT);
    g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_READY;
    printf("[TASK2_DBG] ret=%s cmd=%u\r\n",
           Key_Task2DebugGetRetName(),
           (unsigned)Key_Task2DebugGetRetCmd());
    Gui_Refersh_Bool = ZF_TRUE;
}

static void Key_Task2DebugNextAction(void)
{
    g_task2_debug_action_index = (uint8)((g_task2_debug_action_index + 1U) % KEY_TASK2_DEBUG_ACTION_COUNT);
    g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_READY;
    printf("[TASK2_DBG] action=%s cmd=%u\r\n",
           Key_Task2DebugGetActionName(),
           (unsigned)Key_Task2DebugGetActionCmd());
    Gui_Refersh_Bool = ZF_TRUE;
}

static void Key_Task2DebugLaunchOrStop(void)
{
    if (Key_Task2DebugIsTaskBusy() != 0U)
    {
        ctrl_task_stop();
        g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_STOP;
        printf("[TASK2_DBG] stop\r\n");
        Gui_Refersh_Bool = ZF_TRUE;
        return;
    }

    ctrl_task_start(2U);
    ctrl_task2_sequence_set_enable(1U);

    if (Key_Task2DebugQueueSelected() == 0U)
    {
        g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_PUSHERR;
        ctrl_task_stop();
        printf("[TASK2_DBG] start failed seq=GO>%s>RET go=%u act=%u ret=%u\r\n",
               Key_Task2DebugGetActionName(),
               (unsigned)Key_Task2DebugGetGoCmd(),
               (unsigned)Key_Task2DebugGetActionCmd(),
               (unsigned)Key_Task2DebugGetRetCmd());
    }
    else
    {
        ctrl_task2_sequence_update();
        if (ctrl_task2_sequence_is_busy() == 0U)
        {
            g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_STARTERR;
            ctrl_task_stop();
            printf("[TASK2_DBG] start failed no motion seq=GO>%s>RET go=%u act=%u ret=%u\r\n",
                   Key_Task2DebugGetActionName(),
                   (unsigned)Key_Task2DebugGetGoCmd(),
                   (unsigned)Key_Task2DebugGetActionCmd(),
                   (unsigned)Key_Task2DebugGetRetCmd());
        }
        else
        {
            g_task2_debug_last_result = KEY_TASK2_DEBUG_RESULT_START;
            printf("[TASK2_DBG] start seq=GO>%s>RET go=%s/%u act=%u ret=%s/%u\r\n",
                   Key_Task2DebugGetActionName(),
                   Key_Task2DebugGetGoName(),
                   (unsigned)Key_Task2DebugGetGoCmd(),
                   (unsigned)Key_Task2DebugGetActionCmd(),
                   Key_Task2DebugGetRetName(),
                   (unsigned)Key_Task2DebugGetRetCmd());
        }
    }

    Gui_Refersh_Bool = ZF_TRUE;
}


static const char *Key_TargetSrcName(uint8 target_src)
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

static uint8 Key_DebounceRelease(uint8 raw,
                                uint8 *candidate,
                                uint8 *stable,
                                uint32 *candidate_since_ms,
                                uint32 now_ms)
{
    uint8 previous;

    if (raw != *candidate)
    {
        *candidate = raw;
        *candidate_since_ms = now_ms;
        return 0U;
    }

    if ((raw == *stable) ||
        ((uint32)(now_ms - *candidate_since_ms) < KEY_DEBOUNCE_MS))
    {
        return 0U;
    }

    previous = *stable;
    *stable = raw;
    return ((previous != 0U) && (raw == 0U)) ? 1U : 0U;
}


void Key_Init(void)
{
    xl9555_app_init();
    Key_Num = 0U;
    g_key_scan_armed = 0U;
    g_key_startup_release_since_ms = 0U;
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
}

void Beep_Deal(void)
{
    if (BUZZER_NUM == 0U)
    {
        return;
    }

    BUZZER_NUM--;
    gpio_set_level(BUZZER_PIN, (BUZZER_NUM != 0U) ? GPIO_HIGH : GPIO_LOW);
}

void Toggle_Active(void)
{
    (void)xl9555_app_get_switch(TOGGLE1_XL9555);
    (void)xl9555_app_get_switch(TOGGLE2_XL9555);
}

void Key_Active(void)
{
    uint8 active_key = Key_Num;

    if (active_key != 0U)
    {
        g_key_last_event_num = active_key;
        g_key_last_event_time_ms = system_getval_ms();
        g_key_event_count++;
    }

    if (Key_Task3RecordAnyKeyReplay(active_key) != 0U)
    {
        active_key = 0U;
    }

    switch (active_key)
    {
        case 1:
            printf("Key Num: 1\r\n");
#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
            if (Key_IsTask4LightTestPage() != 0U)
            {
                Key_Task4LightTestAllOn();
            }
            else
#endif
            if (Key_IsTask2PointPage() != 0U)
            {
                Key_Task2DebugNextGo();
            }
            else if (Key_IsPathTaskPage() != 0U)
            {
                Key_PathLoadFlash();
            }
            else if (Page_Num == InfoPage)
            {
                ui_task_prev();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (((Page_Num == SetPage) || (Page_Num == ParamPage)) &&
                     (Key_IsTask1UiOverridePage() == 0U) &&
                     (Key_IsTask2PointPage() == 0U) &&
                     (Key_IsTask3CameraMirrorPage() == 0U))
            {
                UI_KeyChange_Callback(0);
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPSPage))
            {
                UI_Task3CameraSnapshotCycleSize();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((Page_Num == GPSPage) && (ui_task_get() != 3U))
            {
                UI_Image_AddPoint(gnss.latitude, gnss.longitude);
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPS_SetPage))
            {
                UI_Task3CameraTestStart(DATA_LOGGER_CAMERA_TEST_MODE_STATIC);
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (Page_Num == GPS_SetPage)
            {
                if (ui_task_get() == 1U)
                {
                    UI_GpsTest_Start();
                }
                else
                {
                    UI_Image_Callback(0);
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            break;

        case 2:
            printf("Key Num: 2\r\n");
#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
            if (Key_IsTask4LightTestPage() != 0U)
            {
                Key_Task4LightTestStart();
            }
            else
#endif
            if (Key_IsTask2PointPage() != 0U)
            {
                Key_Task2DebugNextRet();
            }
            else if ((ui_task_get() == 3U) && (Page_Num == SetPage))
            {
                if (ctrl_task3_follow_is_active() != 0U)
                {
                    ctrl_task3_follow_finish_request();
                    printf("[TASK3_FOLLOW] K2 finish\r\n");
                }
                else if (ctrl_task_is_running() != 0U)
                {
                    ctrl_task_stop();
                    printf("[TASK3_FOLLOW] K2 stop task3\r\n");
                }
                else
                {
                    uint8 ret = ctrl_task3_follow_start();
                    printf("[TASK3_FOLLOW] K2 start ret=%u\r\n",
                           (unsigned)ret);
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (Key_IsPathTaskPage() != 0U)
            {
                Key_PathRecordToggle();
            }
            else if (Page_Num == InfoPage)
            {
                ui_task_next();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (((Page_Num == SetPage) || (Page_Num == ParamPage)) &&
                     (Key_IsTask1UiOverridePage() == 0U) &&
                     (Key_IsTask2PointPage() == 0U) &&
                     (Key_IsTask3CameraMirrorPage() == 0U))
            {
                UI_KeyChange_Callback(1);
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPSPage))
            {
                UI_Task3CameraSnapshotRequest();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((Page_Num == GPSPage) && (ui_task_get() != 3U))
            {
                UI_Image_RemoveLastPoint();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPS_SetPage))
            {
                UI_Task3CameraTestStart(DATA_LOGGER_CAMERA_TEST_MODE_MOTION);
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (Page_Num == GPS_SetPage)
            {
                if (ui_task_get() == 1U)
                {
                    UI_GpsTest_Stop();
                }
                else
                {
                    UI_Image_Callback(1);
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            break;

        case 3:
            printf("Key Num: 3\r\n");
            if (Key_IsTask2PointPage() != 0U)
            {
                Key_Task2DebugNextAction();
            }
            else if ((ui_task_get() == 3U) && (Page_Num == SetPage))
            {
                if (ctrl_task3_follow_is_active() != 0U)
                {
                    ctrl_task3_follow_finish_request();
                    printf("[TASK3_FOLLOW] K3 test finish\r\n");
                }
                else if (ctrl_task_is_running() != 0U)
                {
                    ctrl_task_stop();
                    printf("[TASK3_FOLLOW] K3 stop task3\r\n");
                }
                else
                {
                    uint8 ret = ctrl_task3_follow_test_start();
                    uint8 capture_ret = 255U;

                    if (ret == 0U)
                    {
                        capture_ret = UI_Task3FollowTestCaptureStart();
                        if (capture_ret != 0U)
                        {
                            ctrl_task3_follow_finish_request();
                        }
                    }
                    printf("[TASK3_FOLLOW] K3 test start ret=%u capture=%u\r\n",
                           (unsigned)ret,
                           (unsigned)capture_ret);
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPSPage))
            {
                UI_Task3CameraCaptureModeToggle();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPS_SetPage))
            {
                if (ctrl_task3_follow_is_active() != 0U)
                {
                    ctrl_task3_follow_finish_request();
                    printf("[TASK3_FOLLOW] page5 K3 test finish\r\n");
                }
                else if (ctrl_task_is_running() != 0U)
                {
                    ctrl_task_stop();
                    printf("[TASK3_FOLLOW] page5 K3 stop task3\r\n");
                }
                else
                {
                    uint8 ret = ctrl_task3_follow_test_start();
                    uint8 capture_ret = 255U;

                    if (ret == 0U)
                    {
                        capture_ret = UI_Task3FollowTestCaptureStart();
                        if (capture_ret != 0U)
                        {
                            ctrl_task3_follow_finish_request();
                        }
                    }
                    printf("[TASK3_FOLLOW] page5 K3 test start ret=%u capture=%u\r\n",
                           (unsigned)ret,
                           (unsigned)capture_ret);
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 2U) && (Page_Num == InfoPage))
            {
                if (guimai_voice_is_recording())
                {
                    guimai_voice_stop_record();
                    printf("[GUIMAI] Key3 stop record\r\n");
                }
                else
                {
                    guimai_voice_start_record();
                    printf("[GUIMAI] Key3 start record\r\n");
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (Page_Num == InfoPage)
            {
                uint8 next_target_src = TARGET_SRC_TASK;

                if (g_pid_target_src == TARGET_SRC_TASK)
                {
                    next_target_src = TARGET_SRC_PEDAL;
                }
                else if (g_pid_target_src == TARGET_SRC_PEDAL)
                {
                    next_target_src = TARGET_SRC_REMOTE;
                }
                else if (g_pid_target_src == TARGET_SRC_REMOTE)
                {
                    next_target_src = TARGET_SRC_PWM_TEST;
                }
                else if (g_pid_target_src == TARGET_SRC_PWM_TEST)
                {
                    next_target_src = TARGET_SRC_STEER_SCAN;
                }
                else
                {
                    next_target_src = TARGET_SRC_TASK;
                }

                if (next_target_src != TARGET_SRC_TASK)
                {
                    ctrl_task_pause_for_external_source();
                }
                g_pid_target_src = next_target_src;
                g_pid_target_use_pedal =
                    (g_pid_target_src == TARGET_SRC_PEDAL) ? 1U : 0U;
                if (next_target_src != TARGET_SRC_STEER_SCAN)
                {
                    (void)ctrl_logger_restart_task_src(0U, next_target_src);
                }
                if (g_pid_target_src == TARGET_SRC_PWM_TEST)
                {
                    ctrl_pwm_test_reset();
                }
                else if (g_pid_target_src == TARGET_SRC_REMOTE)
                {
#if YAOKONG_REMOTE_ENABLE
                    yaokong_remote_entry_reset();
#endif
                }
                printf("[CTRL] target source -> %s\r\n", Key_TargetSrcName(g_pid_target_src));
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 1U) && (Page_Num == SetPage))
            {
                ctrl_task1_reverse_toggle();
                printf("[TASK1] replay dir -> %s\r\n",
                       (ctrl_task1_reverse_get() != 0U) ? "BACK" : "FWD");
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((Page_Num == SetPage) &&
                     (Key_IsTask1UiOverridePage() == 0U) &&
                     (Key_IsTask3CameraMirrorPage() == 0U))
            {
                ui_task_state_t *state = ui_task_curr_state();
                program_select++;
                if (program_select > 2)
                {
                    program_select = 1;
                }
                state->program_select = program_select;
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (Page_Num == GPS_SetPage)
            {
                if (ui_task_get() == 1U)
                {
                    UI_GpsTest_Reset();
                }
                else
                {
                    UI_Image_SelectNext();
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            break;

        case 4:
            printf("Key Num: 4\r\n");
            if (Key_IsTask2PointPage() != 0U)
            {
                Key_Task2DebugLaunchOrStop();
            }
            else if ((Page_Num == GPS_SetPage) && (ui_task_get() == 1U))
            {
                UI_GpsTest_MarkPoint();
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((ui_task_get() == 3U) && (Page_Num == GPS_SetPage))
            {
                if (ctrl_task3_follow_is_active() != 0U)
                {
                    ctrl_task3_follow_finish_request();
                    printf("[TASK3_FOLLOW] page5 K4 test finish\r\n");
                }
                else
                {
                    UI_Task3CameraTestStop();
                }
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if ((Page_Num == InfoPage) && (ui_task_get() == GUIMAI_MUSIC_KEY_TASK_ID))
            {
                uint8 music_ret = guimai_music_toggle_first();
                printf("[KEY4] task2 music toggle ret=%u\r\n", (unsigned)music_ret);
                Gui_Refersh_Bool = ZF_TRUE;
            }
            else if (Key_IsPathTaskPage() != 0U)
            {
                Key_PathClearRam();
            }
            else if (Page_Num == InfoPage)
            {
                if (g_pid_target_src == TARGET_SRC_PWM_TEST)
                {
                    ctrl_pwm_test_step();
                    printf("[CTRL] PWMTEST duty=%d\r\n",
                           g_ctrl_cmd.pwm_duty);
                    Gui_Refersh_Bool = ZF_TRUE;
                }
                else if (g_pid_target_src == TARGET_SRC_STEER_SCAN)
                {
                    ctrl_steer_scan_start();
                    printf("[CTRL] STEER SCAN restart\r\n");
                    Gui_Refersh_Bool = ZF_TRUE;
                }
                else if (g_pid_target_src == TARGET_SRC_TASK)
                {
                    if (ctrl_task_is_running())
                    {
                        ctrl_task_stop();
                    }
                    else
                    {
                        ctrl_task_start(ui_task_get());
                    }
                    Gui_Refersh_Bool = ZF_TRUE;
                }
            }
            else if ((Page_Num == SetPage) &&
                     (Key_IsTask1UiOverridePage() == 0U) &&
                     (Key_IsTask3CameraMirrorPage() == 0U))
            {
                ui_task_state_t *state = ui_task_curr_state();
                if (program_select <= 1)
                {
                    program_select = 2;
                }
                else
                {
                    program_select--;
                }
                state->program_select = program_select;
                Gui_Refersh_Bool = ZF_TRUE;
            }
            break;

        case 5:
            printf("Key Num: 5\r\n");
            Page_Num = (UI_Page)(((uint8)Page_Num + 1U) % 5U);
            Gui_Refersh_Bool = ZF_TRUE;
            break;

        default:
            break;
    }

    Key_Num = 0;
#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
    Key_Task4LightTestTask();
#endif
}

void Key_KM4_Deal(void)
{
    Key_Num = 0;
}

void Key_Test_Print(void)
{
    Key_scan();
}

void Key_scan(void)
{
    Key_status raw;
    uint32 now_ms = system_getval_ms();

    Key_Num = 0U;
    raw.key1 = xl9555_app_get_key(KEY1_XL9555);
    raw.key2 = xl9555_app_get_key(KEY2_XL9555);
    raw.key3 = xl9555_app_get_key(KEY3_XL9555);
    raw.key4 = xl9555_app_get_key(KEY4_XL9555);
    raw.key5 = xl9555_app_get_key(KEY5_XL9555);

    if (g_key_scan_armed == 0U)
    {
        if ((raw.key1 != 0U) || (raw.key2 != 0U) || (raw.key3 != 0U) ||
            (raw.key4 != 0U) || (raw.key5 != 0U))
        {
            g_key_startup_release_since_ms = 0U;
            return;
        }

        if (g_key_startup_release_since_ms == 0U)
        {
            g_key_startup_release_since_ms = now_ms;
            return;
        }

        if ((uint32)(now_ms - g_key_startup_release_since_ms) <
            KEY_STARTUP_RELEASE_MS)
        {
            return;
        }

        Status.key1 = 0U;
        Status.key2 = 0U;
        Status.key3 = 0U;
        Status.key4 = 0U;
        Status.key5 = 0U;
        L_status.key1 = 0U;
        L_status.key2 = 0U;
        L_status.key3 = 0U;
        L_status.key4 = 0U;
        L_status.key5 = 0U;
        g_key_scan_armed = 1U;
        return;
    }

    if (Key_DebounceRelease(raw.key1,
                            &L_status.key1,
                            &Status.key1,
                            &g_key_candidate_since_ms[0],
                            now_ms) != 0U)
    {
        Key_Num = 1U;
    }
    if (Key_DebounceRelease(raw.key2,
                            &L_status.key2,
                            &Status.key2,
                            &g_key_candidate_since_ms[1],
                            now_ms) != 0U)
    {
        Key_Num = 2U;
    }
    if (Key_DebounceRelease(raw.key3,
                            &L_status.key3,
                            &Status.key3,
                            &g_key_candidate_since_ms[2],
                            now_ms) != 0U)
    {
        Key_Num = 3U;
    }
    if (Key_DebounceRelease(raw.key4,
                            &L_status.key4,
                            &Status.key4,
                            &g_key_candidate_since_ms[3],
                            now_ms) != 0U)
    {
        Key_Num = 4U;
    }
    if (Key_DebounceRelease(raw.key5,
                            &L_status.key5,
                            &Status.key5,
                            &g_key_candidate_since_ms[4],
                            now_ms) != 0U)
    {
        Key_Num = 5U;
    }
}

static uint8 Key_IsPathTaskPage(void)
{
    uint8 task_id = ui_task_get();

    if (task_id == 1U)
    {
        return (Page_Num == SetPage) ? 1U : 0U;
    }

    return ((task_id == 3U) && (Page_Num == ParamPage)) ? 1U : 0U;
}

static uint8 Key_IsTask2PointPage(void)
{
    return ((ui_task_get() == 2U) && (Page_Num == ParamPage)) ? 1U : 0U;
}

static uint8 Key_IsTask3CameraMirrorPage(void)
{
    return ((ui_task_get() == 3U) && (Page_Num == SetPage)) ? 1U : 0U;
}

static uint8 Key_IsTask1UiOverridePage(void)
{
    return ((ui_task_get() == 1U) &&
            ((Page_Num == SetPage) || (Page_Num == ParamPage))) ? 1U : 0U;
}

#if TLD7002_PROJECT_DOT_MATRIX_ENABLE
static uint8 Key_IsTask4LightTestPage(void)
{
    return ((ui_task_get() == 4U) && (Page_Num == ParamPage)) ? 1U : 0U;
}

static void Key_Task4LightTestStart(void)
{
    g_task4_light_test_active = 1U;
    g_task4_light_test_owned = 1U;
    g_task4_light_test_pattern_index = 0U;
    dot_matrix_screen_show_pattern(g_task4_light_test_patterns[0]);
    g_task4_light_test_next_ms = system_getval_ms() + KEY_TASK4_LIGHT_TEST_PERIOD_MS;
    printf("[T4_LED_TEST] loop start\r\n");
}

static void Key_Task4LightTestAllOn(void)
{
    g_task4_light_test_active = 0U;
    g_task4_light_test_owned = 1U;
    g_task4_light_test_pattern_index = 0U;
    g_task4_light_test_next_ms = 0U;
    dot_matrix_screen_show_pattern(DOT_MATRIX_SCREEN_PATTERN_ALL_ON);
    printf("[T4_LED_TEST] all on\r\n");
}

static void Key_Task4LightTestTask(void)
{
    uint32 now_ms;

    if (Key_IsTask4LightTestPage() == 0U)
    {
        if (g_task4_light_test_owned != 0U)
        {
            g_task4_light_test_active = 0U;
            g_task4_light_test_owned = 0U;
            g_task4_light_test_pattern_index = 0U;
            g_task4_light_test_next_ms = 0U;
            dot_matrix_screen_show_pattern(DOT_MATRIX_SCREEN_PATTERN_BLANK);
            printf("[T4_LED_TEST] stop and blank\r\n");
        }
        return;
    }

    if (g_task4_light_test_active == 0U)
    {
        return;
    }

    now_ms = system_getval_ms();
    if ((int32)(now_ms - g_task4_light_test_next_ms) < 0)
    {
        return;
    }

    g_task4_light_test_pattern_index =
        (uint8)((g_task4_light_test_pattern_index + 1U) % KEY_TASK4_LIGHT_TEST_PATTERN_COUNT);
    dot_matrix_screen_show_pattern(g_task4_light_test_patterns[g_task4_light_test_pattern_index]);
    g_task4_light_test_next_ms = now_ms + KEY_TASK4_LIGHT_TEST_PERIOD_MS;
}
#endif

static const char *Key_PathFileForTask(uint8 task_id)
{
    if (task_id == 1U)
    {
        return NAV_PATH_TASK1_FILE;
    }
    if (task_id == 2U)
    {
        return NAV_PATH_TASK2_FILE;
    }

    return NAV_PATH_TASK3_FILE;
}

static uint8 Key_ShouldRecordFullNavLogger(uint8 task_id)
{
    return ((task_id == 1U) || (task_id == 3U)) ? 1U : 0U;
}

static uint8 Key_Task3RecordAnyKeyReplay(uint8 active_key)
{
    nav_path_state_t path_state;

    if ((active_key == 0U) ||
        (ui_task_get() != 3U))
    {
        return 0U;
    }

    nav_control_path_get_state(&path_state);
    if ((path_state.active == 0U) ||
        (path_state.mode != NAV_PATH_MODE_RECORD))
    {
        if ((ctrl_task3_follow_is_active() == 0U) &&
            (ctrl_task_is_running() != 0U) &&
            (g_task_ctrl_id == 3U))
        {
            printf("[TASK3_KEY] K%u ignored: task3 already running\r\n",
                   (unsigned)active_key);
            return 1U;
        }
        return 0U;
    }

    printf("[TASK3_KEY] K%u stop/save/replay as CH6\r\n",
           (unsigned)active_key);
    ctrl_task3_remote_ch6_request();
    Gui_Refersh_Bool = ZF_TRUE;
    return 1U;
}

static void Key_PathLoadFlash(void)
{
    uint8 task_id = ui_task_get();
    uint8 ret;

    if (nav_control_path_sd_save_is_busy() != 0U)
    {
        printf("[NAV_PATH] task%u load flash blocked: sd save pending.\r\n",
               (unsigned)task_id);
        Gui_Refersh_Bool = ZF_TRUE;
        return;
    }

    ret = nav_control_path_load_from_flash_task(task_id);
    printf("[NAV_PATH] task%u load flash ret=%u\r\n", (unsigned)task_id, (unsigned)ret);
    Gui_Refersh_Bool = ZF_TRUE;
}

static void Key_PathRecordToggle(void)
{
    nav_path_state_t path_state;
    uint8 task_id = ui_task_get();
    uint8 sd_req_ret;
    uint8 ret_flash;
    uint8 logger_restart_ret = 255U;

    nav_control_path_get_state(&path_state);
    if ((path_state.active != 0U) && (path_state.mode == NAV_PATH_MODE_RECORD))
    {
        nav_ins_state_t ins_state;
        uint8 ref_ready;

        nav_control_path_stop();
        if ((Key_ShouldRecordFullNavLogger(task_id) != 0U) &&
            (data_logger_is_busy() != 0U))
        {
            data_logger_stop();
        }
        if (task_id == 3U)
        {
            (void)nav_control_path_force_last_sample_yaw(KEY_TASK3_RECORD_STOP_YAW_DEG);
        }
        ret_flash = nav_control_path_save_to_flash_task(task_id);
        nav_control_path_get_state(&path_state);
        nav_ins_get_state(&ins_state);
        ref_ready = IMU_AHRS_IsReferenceReady();
        sd_req_ret = nav_control_path_request_save_to_sd(task_id,
                                                         Key_PathFileForTask(task_id),
                                                         ret_flash,
                                                         path_state.last_flash_load_result,
                                                         ref_ready,
                                                         ins_state.valid,
                                                         &ins_state);
        printf("[NAV_PATH] task%u stop: samples=%u sd_req=%u flash=%u\r\n",
               (unsigned)task_id,
               (unsigned)path_state.sample_count,
               (unsigned)sd_req_ret,
               (unsigned)ret_flash);
        printf("[NAV_PATH] task%u async sd pending=%u ref=%u ins=%u\r\n",
               (unsigned)task_id,
               (unsigned)nav_control_path_sd_save_is_busy(),
               (unsigned)ref_ready,
               (unsigned)ins_state.valid);
        printf("[NAV_PATH] task%u logger stop state=%u err=%u\r\n",
               (unsigned)task_id,
               (unsigned)data_logger_get_state(),
               (unsigned)data_logger_get_last_error());
    }
    else
    {
        if (nav_control_path_sd_save_is_busy() != 0U)
        {
            printf("[NAV_PATH] task%u record blocked: sd save pending.\r\n",
                   (unsigned)task_id);
            Gui_Refersh_Bool = ZF_TRUE;
            return;
        }

        if ((path_state.valid != 0U) || (path_state.sample_count != 0U))
        {
            printf("[NAV_PATH] task%u record blocked: clear RAM with K4 first. samples=%u\r\n",
                   (unsigned)task_id,
                   (unsigned)path_state.sample_count);
            Gui_Refersh_Bool = ZF_TRUE;
            return;
        }

        (void)nav_control_path_start_record(NAV_PATH_DEFAULT_INTERVAL_M);
        if (Key_ShouldRecordFullNavLogger(task_id) != 0U)
        {
            logger_restart_ret =
                data_logger_force_restart_task_src(task_id,
                                                   DATA_LOGGER_PROFILE_FULL_NAV,
                                                   TARGET_SRC_TASK);
            printf("[NAV_PATH] task%u record logger ret=%u\r\n",
                   (unsigned)task_id,
                   (unsigned)logger_restart_ret);
        }
        printf("[NAV_PATH] task%u start record interval=%.3f\r\n",
               (unsigned)task_id,
               (double)NAV_PATH_DEFAULT_INTERVAL_M);
    }

    Gui_Refersh_Bool = ZF_TRUE;
}

static void Key_PathClearRam(void)
{
    nav_path_state_t path_state;
    uint8 task_id = ui_task_get();

    nav_control_path_get_state(&path_state);
    if ((path_state.active != 0U) && (path_state.mode == NAV_PATH_MODE_RECORD))
    {
        printf("[NAV_PATH] clear blocked: stop record first.\r\n");
    }
    else if (nav_control_path_sd_save_is_busy() != 0U)
    {
        printf("[NAV_PATH] clear blocked: sd save pending.\r\n");
    }
    else if ((path_state.active != 0U) && (path_state.mode == NAV_PATH_MODE_REPLAY))
    {
        printf("[NAV_PATH] clear blocked: stop replay first.\r\n");
    }
    else if ((path_state.valid != 0U) &&
             (path_state.last_sd_result != 0U) &&
             (path_state.last_flash_result != 0U) &&
             (path_state.last_flash_load_result != 0U))
    {
        printf("[NAV_PATH] clear blocked: path is not saved/loaded safely.\r\n");
    }
    else
    {
        nav_control_path_clear();
        printf("[NAV_PATH] task%u RAM path cleared.\r\n", (unsigned)task_id);
    }

    Gui_Refersh_Bool = ZF_TRUE;
}
