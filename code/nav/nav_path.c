#include "nav_path.h"

#include "ff.h"
#include "sd_simple.h"
#include "data_logger.h"
#include "zf_driver_flash.h"
#include <math.h>
#include "stdio.h"
#include "string.h"

#define NAV_PATH_POS_SCALE_CM          (100.0f)
#define NAV_PATH_YAW_SCALE_CDEG        (100.0f)
#define NAV_PATH_KAPPA_SCALE_MILLI     (1000.0f)
#define NAV_PATH_SPEED_SCALE_CMPS      (100.0f)
#define NAV_PATH_HEADER                "NAVPATH_V2"
#define NAV_PATH_FLASH_MAGIC           (0x4E504154UL)
#define NAV_PATH_FLASH_VERSION         (2UL)
#define NAV_PATH_FLASH_SECTION         (0U)
#define NAV_PATH_FLASH_PAGE_COUNT      (13U)
#define NAV_PATH_FLASH_TASK3_FOLLOW_PAGE_COUNT (17U)
#define NAV_PATH_FLASH_DATA_OFFSET     (4U)
#define NAV_PATH_FLASH_WORDS_PER_SAMPLE (3U)
#define NAV_PATH_SAVE_NOT_TRIED        (255U)
#define NAV_PATH_SAVE_PENDING          (254U)
#define NAV_PATH_ASYNC_FILENAME_SIZE   (64U)
#define NAV_PATH_FLASH_TASK2_BASE      (60U)
#define NAV_PATH_FLASH_TASK1_BASE      (80U)
#define NAV_PATH_FLASH_TASK3_BASE      (110U)
#define NAV_PATH_FLASH_TASK3_FOLLOW_BASE (40U)
#if ((NAV_PATH_FLASH_TASK3_FOLLOW_BASE + NAV_PATH_FLASH_TASK3_FOLLOW_PAGE_COUNT) > \
     NAV_PATH_FLASH_TASK2_BASE)
#error "Task3 follow Flash pages overlap Task2 path storage"
#endif
#define NAV_PATH_TASK1_SD_RETRY_MS     (10000U)
#define NAV_PATH_FS_NO_MOUNT           (0U)
#define NAV_PATH_FS_ALLOW_MOUNT        (1U)
#define NAV_PATH_REPLAY_PROJECT_BACK_SAMPLES (8U)
#define NAV_PATH_REPLAY_PROJECT_FORWARD_SAMPLES (32U)
#define NAV_PATH_REPLAY_MIN_SEG_LEN2_M (0.000025f)
#define NAV_PATH_REPLAY_FINISH_INDEX_TOL_SAMPLES (4U)
#define NAV_PATH_REPLAY_DIR_CHANGE_SCAN_M (2.80f)
#define NAV_PATH_REPLAY_DIR_CHANGE_STATIONARY_SWITCH_M (0.12f)
#define NAV_PATH_REPLAY_DIR_CHANGE_STATIONARY_MIN_SAMPLES (6U)
#define NAV_PATH_REPLAY_FINISH_REACH_M (0.30f)
#define NAV_PATH_REPLAY_FINISH_OVERRUN_M (2.0f)
#define NAV_PATH_REPLAY_FINISH_EARLY_REACH_M (0.18f)
#define NAV_PATH_REPLAY_FINISH_EARLY_LAT_M (0.08f)
#define NAV_PATH_REPLAY_FINISH_EARLY_YAW_DEG (8.0f)
#define NAV_PATH_REPLAY_FINISH_EARLY_PROGRESS_M (0.45f)
#define NAV_PATH_REPLAY_FINISH_TERMINAL_REACH_M (0.34f)
#define NAV_PATH_REPLAY_FINISH_TERMINAL_LAT_M (0.28f)
#define NAV_PATH_REPLAY_FINISH_TERMINAL_YAW_DEG (28.0f)
#define NAV_PATH_REPLAY_FINISH_TERMINAL_SPEED_MPS (0.12f)
#define NAV_PATH_REPLAY_FINISH_CONFIRM_COUNT (4U)
#define NAV_PATH_REPLAY_FINISH_MAX_SPEED_MPS (0.12f)
#define NAV_PATH_REPLAY_PREVIEW_BASE_M (0.34f)
#define NAV_PATH_REPLAY_PREVIEW_SPEED_GAIN_S (0.46f)
#define NAV_PATH_REPLAY_PREVIEW_MIN_M (0.24f)
#define NAV_PATH_REPLAY_PREVIEW_MAX_M (1.16f)
#define NAV_PATH_REPLAY_PREVIEW_REVERSE_MAX_M (0.56f)
#define NAV_PATH_REPLAY_PREVIEW_FIXED_INS_MAX_M (0.58f)
#define NAV_PATH_REPLAY_FIXED_INS_MATCH_SPEED_MPS (0.70f)
#define NAV_PATH_REPLAY_PREVIEW_TIGHT_KAPPA_1PM (0.60f)
#define NAV_PATH_REPLAY_PREVIEW_MID_KAPPA_1PM (0.35f)
#define NAV_PATH_REPLAY_PREVIEW_SOFT_KAPPA_1PM (0.15f)
#define NAV_PATH_REPLAY_PREVIEW_TIGHT_MAX_M (0.72f)
#define NAV_PATH_REPLAY_PREVIEW_MID_MAX_M (0.90f)
#define NAV_PATH_REPLAY_PREVIEW_SOFT_MAX_M (1.05f)
#define NAV_PATH_REPLAY_KAPPA_AHEAD_BASE_M (1.32f)
#define NAV_PATH_REPLAY_KAPPA_AHEAD_SPEED_GAIN_S (0.20f)
#define NAV_PATH_REPLAY_KAPPA_AHEAD_MIN_M  (1.20f)
#define NAV_PATH_REPLAY_KAPPA_AHEAD_MAX_M  (2.20f)
#define NAV_PATH_REPLAY_KAPPA_AHEAD_REVERSE_MAX_M (1.40f)
#define NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_BASE_M (0.65f)
#define NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_SPEED_GAIN_S (0.18f)
#define NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_MIN_M (0.65f)
#define NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_MAX_M (1.35f)
#define NAV_PATH_REPLAY_KAPPA_ARC_M (0.48f)
#define NAV_PATH_REPLAY_KAPPA_ARC_MIN_M (0.16f)
#define NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM (1.60f)
#define NAV_PATH_REPLAY_KAPPA_REV_MIN_1PM (0.22f)
#define NAV_PATH_REPLAY_KAPPA_REV_PRODUCT_1PM2 (-0.045f)
#define NAV_PATH_REPLAY_KAPPA_REV_EXTRA_AHEAD_M (0.45f)
#define NAV_PATH_REPLAY_MOTION_AHEAD_M (1.20f)
#define NAV_PATH_REPLAY_PREVIEW_YAW_MAX_DELTA_DEG (70.0f)
#define NAV_PATH_REPLAY_PREVIEW_FIXED_INS_YAW_MAX_DELTA_DEG (38.0f)
#define NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_NEAR_M (0.45f)
#define NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_FINAL_M (0.16f)
#define NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_NEAR_LIMIT_DEG (25.0f)
#define NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_FINAL_LIMIT_DEG (8.0f)
#define NAV_PATH_REPLAY_PROJECT_MAX_FRAME_JUMP (24U)
#define NAV_PATH_REPLAY_PROJECT_YAW_JUMP_DEG (48.0f)
#define NAV_PATH_REPLAY_PROJECT_DIR_CHANGE_MAX_FRAME_JUMP (90U)
#define NAV_PATH_REPLAY_PROJECT_DIR_CHANGE_YAW_JUMP_DEG (118.0f)
#define NAV_PATH_REPLAY_PROJECT_STATIONARY_SWITCH_SCORE_MARGIN (0.16f)
#define NAV_PATH_REPLAY_PROJECT_FIXED_INS_MAX_FRAME_JUMP (12U)
#define NAV_PATH_REPLAY_PROJECT_FIXED_INS_YAW_JUMP_DEG (48.0f)
#define NAV_PATH_REPLAY_PROJECT_LAST_BACK_SAMPLES (10U)
#define NAV_PATH_REPLAY_PROJECT_LAST_FORWARD_SAMPLES (34U)
#define NAV_PATH_REPLAY_PROJECT_LAST_FIXED_INS_BACK_SAMPLES (8U)
#define NAV_PATH_REPLAY_PROJECT_LAST_FIXED_INS_FORWARD_SAMPLES (18U)
#define NAV_PATH_REPLAY_PROJECT_EXPECTED_ARC_WEIGHT (0.18f)
#define NAV_PATH_REPLAY_PROJECT_FIXED_INS_EXPECTED_ARC_WEIGHT (0.42f)
#define NAV_PATH_REPLAY_PROJECT_LAST_ARC_WEIGHT (0.24f)
#define NAV_PATH_REPLAY_PROJECT_FIXED_INS_LAST_ARC_WEIGHT (0.62f)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_BACK_SAMPLES (20U)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_FORWARD_SAMPLES (90U)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_MAX_FRAME_JUMP (90U)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_YAW_JUMP_DEG (120.0f)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_EXPECTED_ARC_WEIGHT (0.035f)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_LAST_ARC_WEIGHT (0.050f)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_YAW_WEIGHT (0.42f)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_BODY_FRONT_MIN_M (0.05f)
#define NAV_PATH_REPLAY_PROJECT_RESCUE_BODY_BACK_WEIGHT (6.0f)
#define NAV_PATH_REPLAY_PROGRESS_SYNC_DEADBAND_M (0.18f)
#define NAV_PATH_REPLAY_PROGRESS_SYNC_HARD_M (1.10f)
#define NAV_PATH_REPLAY_PROGRESS_SYNC_BLEND (0.45f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_KAPPA_MIN_1PM (0.46f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_LAT_M (0.55f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_YAW_DEG (48.0f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_LAT_YAW_M (0.36f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_LAT_YAW_DEG (35.0f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_LAT_M (0.38f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_YAW_DEG (34.0f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_LAT_YAW_M (0.24f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_LAT_YAW_DEG (26.0f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_LAT_M (0.30f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_YAW_DEG (20.0f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_LAT_YAW_M (0.18f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_LAT_YAW_DEG (20.0f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_SCALE (0.20f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_SCALE (0.45f)
#define NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_SCALE (0.72f)
#define NAV_PATH_REPLAY_PROGRESS_HARD_HOLD_SCALE (0.04f)
#define NAV_PATH_REPLAY_PROGRESS_HOLD_SCALE (0.12f)
#define NAV_PATH_REPLAY_PROGRESS_SLOW_SCALE (0.35f)
#define NAV_PATH_REPLAY_PROGRESS_SOFT_SCALE (0.68f)
#define NAV_PATH_REPLAY_PROGRESS_FIXED_INS_HOLD_SCALE (0.24f)
#define NAV_PATH_REPLAY_PROGRESS_FIXED_INS_SLOW_SCALE (0.38f)
#define NAV_PATH_REPLAY_PROGRESS_FIXED_INS_SOFT_SCALE (0.62f)
#define NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_START_M (1.20f)
#define NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_FULL_M (0.35f)
#define NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_FADE_M (0.20f)
#define NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_CUTOFF_M (0.35f)
static int16 g_nav_path_x_cm[NAV_PATH_MAX_SAMPLES];
static int16 g_nav_path_y_cm[NAV_PATH_MAX_SAMPLES];
static int16 g_nav_path_yaw_cdeg[NAV_PATH_MAX_SAMPLES];
static int16 g_nav_path_kappa_milli[NAV_PATH_MAX_SAMPLES];
static int16 g_nav_path_speed_cmps[NAV_PATH_MAX_SAMPLES];
static nav_path_state_t g_nav_path_state = {0};
static float g_nav_path_record_next_m = 0.0f;
static float g_nav_path_base_distance_m = 0.0f;
static float g_nav_path_base_x_m = 0.0f;
static float g_nav_path_base_y_m = 0.0f;
static float g_nav_path_progress_m = 0.0f;
static float g_nav_path_last_distance_m = 0.0f;
static float g_nav_path_record_yaw_origin_deg = 0.0f;
static uint8 g_nav_path_base_valid = 0U;
static uint8 g_nav_path_last_distance_valid = 0U;
static uint8 g_nav_path_record_yaw_origin_valid = 0U;
static uint8 g_nav_path_replay_reverse = 0U;
static uint8 g_nav_path_replay_backward = 0U;
static uint8 g_nav_path_replay_align_pending = 0U;
static uint8 g_nav_path_replay_fixed_ins = 0U;
static uint8 g_nav_path_replay_progress_gate = 0U;
static uint8 g_nav_path_replay_rescue_project = 0U;
static uint8 g_nav_path_replay_rescue_anchor_valid = 0U;
static uint16 g_nav_path_replay_rescue_anchor_index = 0U;
static uint8 g_nav_path_replay_project_last_valid = 0U;
static uint16 g_nav_path_replay_project_last_index = 0U;
static float g_nav_path_replay_project_last_yaw_deg = 0.0f;
static uint8 g_nav_path_finish_confirm_count = 0U;
static float g_nav_path_replay_anchor_x_m = 0.0f;
static float g_nav_path_replay_anchor_y_m = 0.0f;
static float g_nav_path_replay_yaw_offset_deg = 0.0f;
static FATFS g_nav_path_fatfs;
static volatile uint8 g_nav_path_mounted = 0U;
static uint8 g_nav_path_save_status_inited = 0U;
static volatile uint8 g_nav_path_task1_dir_ready = 0U;
static uint8 g_nav_path_task1_dir_try_valid = 0U;
static uint32 g_nav_path_task1_dir_last_try_ms = 0U;
static volatile uint8 g_nav_path_task1_dir_request = 0U;
static volatile uint8 g_nav_path_sd_save_request = 0U;
static volatile uint8 g_nav_path_sd_save_active = 0U;
static uint8 g_nav_path_sd_save_task_id = 0U;
static uint8 g_nav_path_sd_save_flash_result = NAV_PATH_SAVE_NOT_TRIED;
static uint8 g_nav_path_sd_save_flash_load_result = NAV_PATH_SAVE_NOT_TRIED;
static uint8 g_nav_path_sd_save_ref_ready = 0U;
static uint8 g_nav_path_sd_save_ins_valid = 0U;
static nav_ins_state_t g_nav_path_sd_save_ins_state = {0};
static char g_nav_path_sd_save_filename[NAV_PATH_ASYNC_FILENAME_SIZE] = {0};

static float NavPath_ReplayYawForIndex(uint16 index);
static uint16 NavPath_PreviewIndexFromRef(uint16 ref_index, float preview_m);
static float NavPath_KappaAheadDistanceM(uint16 ref_index, float speed_mps);
static float NavPath_YawKappaAheadDistanceM(float speed_mps);
static float NavPath_LocalAheadKappaFromIndex(uint16 ref_index, float ahead_m);
static float NavPath_AbsF32(float value);
static void NavPath_DirChangeInfo(uint16 ref_index,
                                  float max_scan_m,
                                  float *out_distance_m,
                                  uint16 *out_samples);

static void NavPath_MemorySync(void)
{
    __dsync();
}

static float NavPath_Wrap180(float angle_deg)
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

static int16 NavPath_ClampInt16(int32 value)
{
    if (value > 32767)
    {
        value = 32767;
    }
    else if (value < -32768)
    {
        value = -32768;
    }
    return (int16)value;
}

static int16 NavPath_FloatToInt16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled >= 0.0f)
    {
        scaled += 0.5f;
    }
    else
    {
        scaled -= 0.5f;
    }
    return NavPath_ClampInt16((int32)scaled);
}

static float NavPath_XFromStore(int16 x_cm)
{
    return (float)x_cm / NAV_PATH_POS_SCALE_CM;
}

static float NavPath_YFromStore(int16 y_cm)
{
    return (float)y_cm / NAV_PATH_POS_SCALE_CM;
}

static float NavPath_YawFromStore(int16 yaw_cdeg)
{
    return (float)yaw_cdeg / NAV_PATH_YAW_SCALE_CDEG;
}

static float NavPath_KappaFromStore(int16 kappa_milli)
{
    return (float)kappa_milli / NAV_PATH_KAPPA_SCALE_MILLI;
}

static float NavPath_SpeedFromStore(int16 speed_cmps)
{
    return (float)speed_cmps / NAV_PATH_SPEED_SCALE_CMPS;
}

static int16 NavPath_YawToStore(float yaw_deg)
{
    return NavPath_FloatToInt16(NavPath_Wrap180(yaw_deg), NAV_PATH_YAW_SCALE_CDEG);
}

static int16 NavPath_KappaToStore(float kappa_1pm)
{
    return NavPath_FloatToInt16(kappa_1pm, NAV_PATH_KAPPA_SCALE_MILLI);
}

static const char *NavPath_FileOrDefault(const char *filename)
{
    if ((filename == NULL) || (filename[0] == '\0'))
    {
        return NAV_PATH_DEFAULT_FILE;
    }

    return filename;
}

static uint8 NavPath_EnsureDir(const char *dir)
{
    FRESULT fr;

    fr = f_mkdir(dir);
    if ((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[NAV_PATH] f_mkdir failed, fr=%d dir=%s\r\n", (int)fr, dir);
        return 1U;
    }

    return 0U;
}

static uint8 NavPath_IsTask1Path(const char *path)
{
    return (strncmp(path, NAV_PATH_TASK1_DIR, strlen(NAV_PATH_TASK1_DIR)) == 0) ? 1U : 0U;
}

static uint8 NavPath_EnsureDirForPath(const char *path)
{
    if (NavPath_IsTask1Path(path) != 0U)
    {
        return NavPath_EnsureDir(NAV_PATH_TASK1_DIR);
    }

    if (NavPath_EnsureDir(NAV_PATH_ROOT_DIR) != 0U)
    {
        return 1U;
    }

    return 0U;
}

static const char *NavPath_RecordLogFileForTask(uint8 task_id)
{
    return (task_id == 1U) ? NAV_PATH_TASK1_RECORD_LOG_FILE : NAV_PATH_RECORD_LOG_FILE;
}

static const char *NavPath_RecordDiagFileForTask(uint8 task_id)
{
    return (task_id == 1U) ? NAV_PATH_TASK1_RECORD_DIAG_FILE : NAV_PATH_RECORD_DIAG_FILE;
}

static uint32 NavPath_FlashBaseForTask(uint8 task_id)
{
    if (task_id == 1U)
    {
        return NAV_PATH_FLASH_TASK1_BASE;
    }
    if (task_id == 2U)
    {
        return NAV_PATH_FLASH_TASK2_BASE;
    }
    if (task_id == NAV_PATH_STORAGE_TASK3_FOLLOW_ID)
    {
        return NAV_PATH_FLASH_TASK3_FOLLOW_BASE;
    }

    return NAV_PATH_FLASH_TASK3_BASE;
}

static uint16 NavPath_FlashPageCountForTask(uint8 task_id)
{
    return (task_id == NAV_PATH_STORAGE_TASK3_FOLLOW_ID) ?
           NAV_PATH_FLASH_TASK3_FOLLOW_PAGE_COUNT :
           NAV_PATH_FLASH_PAGE_COUNT;
}

static uint16 NavPath_FlashMaxSamplesForTask(uint8 task_id)
{
    uint32 page_count = NavPath_FlashPageCountForTask(task_id);
    uint32 word_capacity =
        ((uint32)EEPROM_PAGE_LENGTH * page_count) -
        NAV_PATH_FLASH_DATA_OFFSET;

    return (uint16)(word_capacity / NAV_PATH_FLASH_WORDS_PER_SAMPLE);
}

static uint8 NavPath_MountSd(void)
{
    FRESULT fr;
    uint8 ret;

    if (g_nav_path_mounted != 0U)
    {
        return 0U;
    }

    ret = sd_simple_init();
    if (ret != 0U)
    {
        printf("[NAV_PATH] sd init failed, ret=%u\r\n", ret);
        return 1U;
    }

    fr = f_mount(&g_nav_path_fatfs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("[NAV_PATH] f_mount failed, fr=%d\r\n", (int)fr);
        return 2U;
    }

    g_nav_path_mounted = 1U;
    return 0U;
}

static uint8 NavPath_EnsureFs(uint8 allow_mount)
{
    if (g_nav_path_mounted != 0U)
    {
        return 0U;
    }

    if (allow_mount == 0U)
    {
        nav_path_request_task1_sd_dir();
        return 1U;
    }

    return NavPath_MountSd();
}

static uint8 NavPath_ReadLine(FIL *file, char *line, UINT line_size)
{
    UINT read_len;
    UINT used = 0U;
    char ch;
    FRESULT fr;

    if ((file == NULL) || (line == NULL) || (line_size < 2U))
    {
        return 0U;
    }

    while (used < (line_size - 1U))
    {
        fr = f_read(file, &ch, 1U, &read_len);
        if ((fr != FR_OK) || (read_len == 0U))
        {
            break;
        }

        if (ch == '\n')
        {
            break;
        }
        if (ch != '\r')
        {
            line[used++] = ch;
        }
    }

    line[used] = '\0';
    return (used > 0U) ? 1U : 0U;
}

static void NavPath_ClearFlashBuffer(void)
{
    uint16 i;

    for (i = 0U; i < EEPROM_PAGE_LENGTH; i++)
    {
        flash_union_buffer[i].uint32_type = 0U;
    }
}

static uint32 NavPath_PackInt16Pair(int16 lo, int16 hi)
{
    return ((uint32)((uint16)hi) << 16) | (uint32)((uint16)lo);
}

static int16 NavPath_UnpackInt16(uint32 packed, uint8 high_half)
{
    uint16 raw;

    if (high_half != 0U)
    {
        raw = (uint16)((packed >> 16) & 0xFFFFU);
    }
    else
    {
        raw = (uint16)(packed & 0xFFFFU);
    }

    return (int16)raw;
}

static uint32 NavPath_PackSampleWord(uint16 index, uint8 word_offset)
{
    if (word_offset == 0U)
    {
        return NavPath_PackInt16Pair(g_nav_path_x_cm[index], g_nav_path_y_cm[index]);
    }
    if (word_offset == 1U)
    {
        return NavPath_PackInt16Pair(g_nav_path_yaw_cdeg[index], g_nav_path_kappa_milli[index]);
    }
    return ((uint32)((uint16)g_nav_path_speed_cmps[index]) & 0xFFFFU);
}

static void NavPath_UnpackSampleWord(uint16 index, uint8 word_offset, uint32 packed)
{
    if (word_offset == 0U)
    {
        g_nav_path_x_cm[index] = NavPath_UnpackInt16(packed, 0U);
        g_nav_path_y_cm[index] = NavPath_UnpackInt16(packed, 1U);
    }
    else if (word_offset == 1U)
    {
        g_nav_path_yaw_cdeg[index] = NavPath_UnpackInt16(packed, 0U);
        g_nav_path_kappa_milli[index] = NavPath_UnpackInt16(packed, 1U);
    }
    else
    {
        g_nav_path_speed_cmps[index] = NavPath_UnpackInt16(packed, 0U);
    }
}

static int8 NavPath_MotionDirFromIndex(uint16 index)
{
    uint16 from_index;
    uint16 to_index;
    float from_x_m;
    float from_y_m;
    float to_x_m;
    float to_y_m;
    float dx_m;
    float dy_m;
    float yaw_rad;
    float forward_x;
    float forward_y;
    float dot_m;
    float speed_mps;

    if (g_nav_path_state.sample_count < 2U)
    {
        return 1;
    }

    if (index == 0U)
    {
        from_index = 0U;
        to_index = 1U;
    }
    else
    {
        from_index = (uint16)(index - 1U);
        to_index = index;
    }

    from_x_m = NavPath_XFromStore(g_nav_path_x_cm[from_index]);
    from_y_m = NavPath_YFromStore(g_nav_path_y_cm[from_index]);
    to_x_m = NavPath_XFromStore(g_nav_path_x_cm[to_index]);
    to_y_m = NavPath_YFromStore(g_nav_path_y_cm[to_index]);
    dx_m = to_x_m - from_x_m;
    dy_m = to_y_m - from_y_m;

    yaw_rad = NavPath_YawFromStore(g_nav_path_yaw_cdeg[index]) * NAV_PI / 180.0f;
    forward_x = sinf(yaw_rad);
    forward_y = cosf(yaw_rad);
    dot_m = dx_m * forward_x + dy_m * forward_y;
    if (dot_m < -0.005f)
    {
        return -1;
    }
    if (dot_m > 0.005f)
    {
        return 1;
    }

    speed_mps = NavPath_SpeedFromStore(g_nav_path_speed_cmps[index]);
    return (speed_mps < -0.02f) ? -1 : 1;
}

static int8 NavPath_SegmentMotionDir(uint16 start_index)
{
    if ((uint16)(start_index + 1U) < g_nav_path_state.sample_count)
    {
        return NavPath_MotionDirFromIndex((uint16)(start_index + 1U));
    }

    return NavPath_MotionDirFromIndex(start_index);
}

static void NavPath_GetReplayPoint(uint16 index, float *out_x_m, float *out_y_m)
{
    float x_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
    float y_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
    float rel_x_m = x_m - g_nav_path_replay_anchor_x_m;
    float rel_y_m = y_m - g_nav_path_replay_anchor_y_m;
    float yaw_rad = g_nav_path_replay_yaw_offset_deg * NAV_PI / 180.0f;
    float sin_yaw = sinf(yaw_rad);
    float cos_yaw = cosf(yaw_rad);

    if (out_x_m != NULL)
    {
        *out_x_m = rel_x_m * cos_yaw + rel_y_m * sin_yaw;
    }
    if (out_y_m != NULL)
    {
        *out_y_m = -rel_x_m * sin_yaw + rel_y_m * cos_yaw;
    }
}

static float NavPath_ReplayKappaFromIndex(uint16 index)
{
    float kappa_1pm;

    if (index >= g_nav_path_state.sample_count)
    {
        index = (g_nav_path_state.sample_count > 0U) ?
                (uint16)(g_nav_path_state.sample_count - 1U) : 0U;
    }

    kappa_1pm = NavPath_KappaFromStore(g_nav_path_kappa_milli[index]);
    if (g_nav_path_replay_reverse != 0U)
    {
        kappa_1pm = -kappa_1pm;
    }
    return kappa_1pm;
}

static float NavPath_ReplayYawFlipDeg(void)
{
    return (g_nav_path_replay_reverse != g_nav_path_replay_backward) ?
           180.0f : 0.0f;
}

static float NavPath_ReplayArcKappaFromIndex(uint16 ref_index, float arc_m)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 start_index = ref_index;
    uint16 end_index = ref_index;
    uint16 index = ref_index;
    int8 ref_motion_dir;
    float distance_m = 0.0f;
    float start_yaw_deg;
    float end_yaw_deg;
    float dyaw_deg;
    float kappa_1pm;

    if ((sample_count < 2U) || (ref_index >= sample_count))
    {
        return NavPath_ReplayKappaFromIndex(ref_index);
    }

    ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    while (distance_m < arc_m)
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;

        if (g_nav_path_replay_reverse != 0U)
        {
            if (index == 0U)
            {
                break;
            }
            next_index = (uint16)(index - 1U);
        }
        else
        {
            if ((uint16)(index + 1U) >= sample_count)
            {
                break;
            }
            next_index = (uint16)(index + 1U);
        }

        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            break;
        }

        x0_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
        y0_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
        x1_m = NavPath_XFromStore(g_nav_path_x_cm[next_index]);
        y1_m = NavPath_YFromStore(g_nav_path_y_cm[next_index]);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;
        end_index = index;
    }

    if ((end_index == start_index) || (distance_m < NAV_PATH_REPLAY_KAPPA_ARC_MIN_M))
    {
        return NavPath_ReplayKappaFromIndex(ref_index);
    }

    start_yaw_deg = NavPath_ReplayYawForIndex(start_index);
    end_yaw_deg = NavPath_ReplayYawForIndex(end_index);
    dyaw_deg = NavPath_Wrap180(end_yaw_deg - start_yaw_deg);
    kappa_1pm = (dyaw_deg * NAV_PI / 180.0f) / distance_m;

    if (NavPath_AbsF32(kappa_1pm) > NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM)
    {
        kappa_1pm = (kappa_1pm >= 0.0f) ?
                    NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM :
                    -NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM;
    }

    return kappa_1pm;
}

static uint8 NavPath_ReplayKappaReverseAhead(uint16 ref_index, float speed_mps)
{
    float ref_kappa_1pm;
    float ahead_kappa_1pm;
    float ahead_m;
    uint16 ahead_index;

    if (g_nav_path_state.sample_count < 2U)
    {
        return 0U;
    }

    ref_kappa_1pm =
        NavPath_ReplayArcKappaFromIndex(ref_index, NAV_PATH_REPLAY_KAPPA_ARC_M);
    if (NavPath_AbsF32(ref_kappa_1pm) < NAV_PATH_REPLAY_KAPPA_REV_MIN_1PM)
    {
        return 0U;
    }

    ahead_m = NavPath_KappaAheadDistanceM(ref_index, speed_mps) +
              NAV_PATH_REPLAY_KAPPA_REV_EXTRA_AHEAD_M;
    ahead_index = NavPath_PreviewIndexFromRef(ref_index, ahead_m);
    ahead_kappa_1pm =
        NavPath_ReplayArcKappaFromIndex(ahead_index,
                                        NAV_PATH_REPLAY_KAPPA_ARC_M);
    if (NavPath_AbsF32(ahead_kappa_1pm) < NAV_PATH_REPLAY_KAPPA_REV_MIN_1PM)
    {
        return 0U;
    }

    return ((ref_kappa_1pm * ahead_kappa_1pm) <
            NAV_PATH_REPLAY_KAPPA_REV_PRODUCT_1PM2) ? 1U : 0U;
}

static void NavPath_SetRefFromIndex(uint16 index)
{
    NavPath_GetReplayPoint(index,
                           &g_nav_path_state.ref_x_m,
                           &g_nav_path_state.ref_y_m);
    g_nav_path_state.ref_yaw_deg = NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[index]) +
                                                    NavPath_ReplayYawFlipDeg() +
                                                    g_nav_path_replay_yaw_offset_deg);
    g_nav_path_state.ref_kappa_1pm =
        NavPath_ReplayArcKappaFromIndex(index, NAV_PATH_REPLAY_KAPPA_ARC_M);
    g_nav_path_state.ref_kappa_ahead_1pm = g_nav_path_state.ref_kappa_1pm;
    g_nav_path_state.ref_kappa_yaw_1pm = g_nav_path_state.ref_kappa_1pm;
    g_nav_path_state.ref_speed_mps = NavPath_SpeedFromStore(g_nav_path_speed_cmps[index]);
    g_nav_path_state.ref_speed_ahead_mps = g_nav_path_state.ref_speed_mps;
    g_nav_path_state.ref_motion_dir = NavPath_MotionDirFromIndex(index);
    g_nav_path_state.ref_motion_dir_ahead = g_nav_path_state.ref_motion_dir;
    NavPath_DirChangeInfo(index,
                          NAV_PATH_REPLAY_DIR_CHANGE_SCAN_M,
                          &g_nav_path_state.ref_dir_change_distance_m,
                          &g_nav_path_state.ref_dir_change_samples);
}

static float NavPath_ReplayAnchorYawDeg(void)
{
    uint16 index = 0U;
    float yaw_deg;

    if ((g_nav_path_replay_reverse != 0U) && (g_nav_path_state.sample_count > 0U))
    {
        index = g_nav_path_state.sample_count - 1U;
    }

    yaw_deg = NavPath_YawFromStore(g_nav_path_yaw_cdeg[index]);
    if (NavPath_ReplayYawFlipDeg() > 0.0f)
    {
        yaw_deg = NavPath_Wrap180(yaw_deg + 180.0f);
    }

    return yaw_deg;
}

static void NavPath_UpdateReplayAlignment(float current_yaw_deg)
{
    if (g_nav_path_replay_align_pending != 0U)
    {
        g_nav_path_replay_yaw_offset_deg = NavPath_Wrap180(current_yaw_deg - NavPath_ReplayAnchorYawDeg());
        g_nav_path_replay_align_pending = 0U;
    }
}

static float NavPath_LateralError(float x_m, float y_m)
{
    float yaw_rad = g_nav_path_state.ref_yaw_deg * NAV_PI / 180.0f;
    float forward_x = sinf(yaw_rad);
    float forward_y = cosf(yaw_rad);
    float dx = x_m - g_nav_path_base_x_m - g_nav_path_state.ref_x_m;
    float dy = y_m - g_nav_path_base_y_m - g_nav_path_state.ref_y_m;

    return forward_x * dy - forward_y * dx;
}

static float NavPath_AbsF32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float NavPath_ClipF32(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static float NavPath_ReplayProgressScale(float speed_mps)
{
    float lat_abs_m;
    float yaw_abs_deg;
    uint8 reverse_kappa_ahead;

    if ((g_nav_path_state.mode != NAV_PATH_MODE_REPLAY) ||
        (g_nav_path_replay_progress_gate == 0U) ||
        (g_nav_path_state.valid == 0U))
    {
        return 1.0f;
    }

    lat_abs_m = NavPath_AbsF32(g_nav_path_state.lateral_error_m);
    yaw_abs_deg = NavPath_AbsF32(g_nav_path_state.yaw_error_deg);
    reverse_kappa_ahead =
        NavPath_ReplayKappaReverseAhead(g_nav_path_state.replay_index,
                                        speed_mps);

    if (g_nav_path_replay_fixed_ins != 0U)
    {
        if ((lat_abs_m > 1.20f) ||
            (yaw_abs_deg > 80.0f) ||
            ((lat_abs_m > 0.75f) && (yaw_abs_deg > 45.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_FIXED_INS_HOLD_SCALE;
        }
        if ((lat_abs_m > 0.75f) ||
            (yaw_abs_deg > 55.0f) ||
            ((lat_abs_m > 0.50f) && (yaw_abs_deg > 32.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_FIXED_INS_SLOW_SCALE;
        }
        if ((lat_abs_m > 0.45f) ||
            (yaw_abs_deg > 35.0f) ||
            ((lat_abs_m > 0.32f) && (yaw_abs_deg > 24.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_FIXED_INS_SOFT_SCALE;
        }
    }
    else
    {
        if (reverse_kappa_ahead != 0U)
        {
            if ((lat_abs_m > 0.70f) ||
                (yaw_abs_deg > 48.0f) ||
                ((lat_abs_m > 0.38f) && (yaw_abs_deg > 30.0f)))
            {
                return NAV_PATH_REPLAY_PROGRESS_HARD_HOLD_SCALE;
            }
            if ((lat_abs_m > 0.46f) ||
                (yaw_abs_deg > 34.0f) ||
                ((lat_abs_m > 0.28f) && (yaw_abs_deg > 22.0f)))
            {
                return NAV_PATH_REPLAY_PROGRESS_HOLD_SCALE;
            }
            if ((lat_abs_m > 0.30f) ||
                (yaw_abs_deg > 24.0f) ||
                ((lat_abs_m > 0.20f) && (yaw_abs_deg > 16.0f)))
            {
                return NAV_PATH_REPLAY_PROGRESS_SLOW_SCALE;
            }
        }
        if ((NavPath_AbsF32(g_nav_path_state.ref_kappa_1pm) >=
             NAV_PATH_REPLAY_PROGRESS_CURVE_KAPPA_MIN_1PM) ||
            (NavPath_AbsF32(g_nav_path_state.ref_kappa_ahead_1pm) >=
             NAV_PATH_REPLAY_PROGRESS_CURVE_KAPPA_MIN_1PM))
        {
            if ((lat_abs_m > NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_LAT_M) ||
                (yaw_abs_deg > NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_YAW_DEG) ||
                ((lat_abs_m > NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_LAT_YAW_M) &&
                 (yaw_abs_deg > NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_LAT_YAW_DEG)))
            {
                return NAV_PATH_REPLAY_PROGRESS_CURVE_HOLD_SCALE;
            }
            if ((lat_abs_m > NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_LAT_M) ||
                (yaw_abs_deg > NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_YAW_DEG) ||
                ((lat_abs_m > NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_LAT_YAW_M) &&
                 (yaw_abs_deg > NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_LAT_YAW_DEG)))
            {
                return NAV_PATH_REPLAY_PROGRESS_CURVE_SLOW_SCALE;
            }
            if ((lat_abs_m > NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_LAT_M) ||
                (yaw_abs_deg > NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_YAW_DEG) ||
                ((lat_abs_m > NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_LAT_YAW_M) &&
                 (yaw_abs_deg > NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_LAT_YAW_DEG)))
            {
                return NAV_PATH_REPLAY_PROGRESS_CURVE_SOFT_SCALE;
            }
        }
        if ((lat_abs_m > 1.25f) ||
            (yaw_abs_deg > 96.0f) ||
            ((lat_abs_m > 0.90f) && (yaw_abs_deg > 62.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_HARD_HOLD_SCALE;
        }
        if ((lat_abs_m > 0.95f) ||
            (yaw_abs_deg > 72.0f) ||
            ((lat_abs_m > 0.68f) && (yaw_abs_deg > 46.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_HOLD_SCALE;
        }
        if ((lat_abs_m > 0.72f) ||
            (yaw_abs_deg > 52.0f) ||
            ((lat_abs_m > 0.52f) && (yaw_abs_deg > 34.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_SLOW_SCALE;
        }
        if ((lat_abs_m > 0.52f) ||
            ((lat_abs_m > 0.38f) && (yaw_abs_deg > 24.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_SOFT_SCALE;
        }
        if ((lat_abs_m > 1.85f) ||
            (yaw_abs_deg > 138.0f) ||
            ((lat_abs_m > 1.12f) && (yaw_abs_deg > 80.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_HARD_HOLD_SCALE;
        }
        if ((lat_abs_m > 1.25f) ||
            (yaw_abs_deg > 108.0f) ||
            ((lat_abs_m > 0.88f) && (yaw_abs_deg > 60.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_HOLD_SCALE;
        }
        if ((lat_abs_m > 0.95f) ||
            (yaw_abs_deg > 84.0f) ||
            ((lat_abs_m > 0.72f) && (yaw_abs_deg > 46.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_SLOW_SCALE;
        }
        if ((lat_abs_m > 0.82f) ||
            ((lat_abs_m > 0.62f) && (yaw_abs_deg > 36.0f)))
        {
            return NAV_PATH_REPLAY_PROGRESS_SOFT_SCALE;
        }
    }

    return 1.0f;
}

static float NavPath_PreviewDistanceM(float speed_mps, float kappa_1pm)
{
    float speed_abs_mps = NavPath_AbsF32(speed_mps);
    float kappa_abs_1pm = NavPath_AbsF32(kappa_1pm);
    float preview_m = NAV_PATH_REPLAY_PREVIEW_BASE_M +
                      NAV_PATH_REPLAY_PREVIEW_SPEED_GAIN_S * speed_abs_mps;
    float max_preview_m = NAV_PATH_REPLAY_PREVIEW_MAX_M;

    if (kappa_abs_1pm > NAV_PATH_REPLAY_PREVIEW_TIGHT_KAPPA_1PM)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_TIGHT_MAX_M;
    }
    else if (kappa_abs_1pm > NAV_PATH_REPLAY_PREVIEW_MID_KAPPA_1PM)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_MID_MAX_M;
    }
    else if (kappa_abs_1pm > NAV_PATH_REPLAY_PREVIEW_SOFT_KAPPA_1PM)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_SOFT_MAX_M;
    }

    if (((g_nav_path_replay_backward != 0U) ||
         (g_nav_path_state.ref_motion_dir < 0)) &&
        (max_preview_m > NAV_PATH_REPLAY_PREVIEW_REVERSE_MAX_M))
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_REVERSE_MAX_M;
    }
    if ((g_nav_path_replay_fixed_ins != 0U) &&
        (max_preview_m > NAV_PATH_REPLAY_PREVIEW_FIXED_INS_MAX_M))
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_FIXED_INS_MAX_M;
    }

    return NavPath_ClipF32(preview_m,
                           NAV_PATH_REPLAY_PREVIEW_MIN_M,
                           max_preview_m);
}

static uint16 NavPath_PreviewIndexFromRef(uint16 ref_index, float preview_m)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 index = ref_index;
    int8 ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    float distance_m = 0.0f;

    if ((sample_count < 2U) || (ref_index >= sample_count))
    {
        return ref_index;
    }

    while (distance_m < preview_m)
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;

        if (g_nav_path_replay_reverse != 0U)
        {
            if (index == 0U)
            {
                break;
            }
            next_index = (uint16)(index - 1U);
        }
        else
        {
            if ((uint16)(index + 1U) >= sample_count)
            {
                break;
            }
            next_index = (uint16)(index + 1U);
        }

        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            break;
        }

        x0_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
        y0_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
        x1_m = NavPath_XFromStore(g_nav_path_x_cm[next_index]);
        y1_m = NavPath_YFromStore(g_nav_path_y_cm[next_index]);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;
    }

    return index;
}

static uint16 NavPath_PreviewIndexAnyMotionFromRef(uint16 ref_index, float preview_m)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 index = ref_index;
    float distance_m = 0.0f;

    if ((sample_count < 2U) || (ref_index >= sample_count))
    {
        return ref_index;
    }

    while (distance_m < preview_m)
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;

        if (g_nav_path_replay_reverse != 0U)
        {
            if (index == 0U)
            {
                break;
            }
            next_index = (uint16)(index - 1U);
        }
        else
        {
            if ((uint16)(index + 1U) >= sample_count)
            {
                break;
            }
            next_index = (uint16)(index + 1U);
        }

        NavPath_GetReplayPoint(index, &x0_m, &y0_m);
        NavPath_GetReplayPoint(next_index, &x1_m, &y1_m);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;
    }

    return index;
}

static void NavPath_DirChangeInfo(uint16 ref_index,
                                  float max_scan_m,
                                  float *out_distance_m,
                                  uint16 *out_samples)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 index = ref_index;
    uint16 samples = 0U;
    int8 ref_motion_dir;
    float distance_m = 0.0f;

    if (out_distance_m != NULL)
    {
        *out_distance_m = max_scan_m;
    }
    if (out_samples != NULL)
    {
        *out_samples = 0U;
    }
    if ((sample_count < 2U) || (ref_index >= sample_count))
    {
        return;
    }

    ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    while (distance_m < max_scan_m)
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;

        if (g_nav_path_replay_reverse != 0U)
        {
            if (index == 0U)
            {
                break;
            }
            next_index = (uint16)(index - 1U);
        }
        else
        {
            if ((uint16)(index + 1U) >= sample_count)
            {
                break;
            }
            next_index = (uint16)(index + 1U);
        }

        NavPath_GetReplayPoint(index, &x0_m, &y0_m);
        NavPath_GetReplayPoint(next_index, &x1_m, &y1_m);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        samples++;

        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            if (out_distance_m != NULL)
            {
                *out_distance_m = distance_m;
            }
            if (out_samples != NULL)
            {
                *out_samples = samples;
            }
            return;
        }

        index = next_index;
    }

    if (out_distance_m != NULL)
    {
        *out_distance_m = max_scan_m;
    }
    if (out_samples != NULL)
    {
        *out_samples = samples;
    }
}

static float NavPath_KappaAheadDistanceM(uint16 ref_index, float speed_mps)
{
    float speed_abs_mps = NavPath_AbsF32(speed_mps);
    float ahead_m = NAV_PATH_REPLAY_KAPPA_AHEAD_BASE_M +
                    NAV_PATH_REPLAY_KAPPA_AHEAD_SPEED_GAIN_S * speed_abs_mps;

    if (ahead_m < NAV_PATH_REPLAY_KAPPA_AHEAD_MIN_M)
    {
        ahead_m = NAV_PATH_REPLAY_KAPPA_AHEAD_MIN_M;
    }
    if (ahead_m > NAV_PATH_REPLAY_KAPPA_AHEAD_MAX_M)
    {
        ahead_m = NAV_PATH_REPLAY_KAPPA_AHEAD_MAX_M;
    }
    if ((NavPath_MotionDirFromIndex(ref_index) < 0) &&
        (ahead_m > NAV_PATH_REPLAY_KAPPA_AHEAD_REVERSE_MAX_M))
    {
        ahead_m = NAV_PATH_REPLAY_KAPPA_AHEAD_REVERSE_MAX_M;
    }

    return ahead_m;
}

static float NavPath_YawKappaAheadDistanceM(float speed_mps)
{
    float ahead_m = NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_BASE_M +
                    NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_SPEED_GAIN_S *
                    NavPath_AbsF32(speed_mps);

    if (ahead_m < NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_MIN_M)
    {
        ahead_m = NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_MIN_M;
    }
    if (ahead_m > NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_MAX_M)
    {
        ahead_m = NAV_PATH_REPLAY_YAW_KAPPA_AHEAD_MAX_M;
    }

    return ahead_m;
}

static float NavPath_LocalAheadKappaFromIndex(uint16 ref_index, float ahead_m)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 index = ref_index;
    int8 ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    float distance_m = 0.0f;
    float best_kappa_1pm =
        NavPath_ReplayArcKappaFromIndex(ref_index, NAV_PATH_REPLAY_KAPPA_ARC_M);
    float best_abs_kappa_1pm = NavPath_AbsF32(best_kappa_1pm);

    while ((sample_count >= 2U) && (distance_m < ahead_m))
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;
        float kappa_1pm;
        float abs_kappa_1pm;

        if (g_nav_path_replay_reverse != 0U)
        {
            if (index == 0U)
            {
                break;
            }
            next_index = (uint16)(index - 1U);
        }
        else
        {
            if ((uint16)(index + 1U) >= sample_count)
            {
                break;
            }
            next_index = (uint16)(index + 1U);
        }

        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            break;
        }

        NavPath_GetReplayPoint(index, &x0_m, &y0_m);
        NavPath_GetReplayPoint(next_index, &x1_m, &y1_m);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;

        kappa_1pm = NavPath_ReplayArcKappaFromIndex(index,
                                                    NAV_PATH_REPLAY_KAPPA_ARC_M);
        abs_kappa_1pm = NavPath_AbsF32(kappa_1pm);
        if (abs_kappa_1pm > best_abs_kappa_1pm)
        {
            best_abs_kappa_1pm = abs_kappa_1pm;
            best_kappa_1pm = kappa_1pm;
        }
    }

    return best_kappa_1pm;
}

static void NavPath_ApplyYawKappa(uint16 ref_index, float speed_mps)
{
    float ahead_m = NavPath_YawKappaAheadDistanceM(speed_mps);

    g_nav_path_state.ref_kappa_yaw_1pm =
        NavPath_LocalAheadKappaFromIndex(ref_index, ahead_m);
}

static void NavPath_ApplyAheadKappa(uint16 ref_index, float speed_mps)
{
    float ahead_m = NavPath_KappaAheadDistanceM(ref_index, speed_mps);
    float reverse_ahead_m = ahead_m + NAV_PATH_REPLAY_KAPPA_REV_EXTRA_AHEAD_M;
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 index = ref_index;
    uint16 ahead_index = NavPath_PreviewIndexFromRef(ref_index, ahead_m);
    int8 ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    float distance_m = 0.0f;
    float best_abs_kappa_1pm =
        NavPath_AbsF32(NavPath_ReplayArcKappaFromIndex(ref_index,
                                                       NAV_PATH_REPLAY_KAPPA_ARC_M));
    float best_kappa_1pm =
        NavPath_ReplayArcKappaFromIndex(ref_index, NAV_PATH_REPLAY_KAPPA_ARC_M);
    float ref_kappa_1pm = best_kappa_1pm;
    float ref_abs_kappa_1pm = best_abs_kappa_1pm;
    uint8 opposite_valid = 0U;
    float opposite_abs_kappa_1pm = 0.0f;
    float opposite_kappa_1pm = 0.0f;

    while ((sample_count >= 2U) && (distance_m < reverse_ahead_m))
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;
        float kappa_1pm;
        float abs_kappa_1pm;

        if (g_nav_path_replay_reverse != 0U)
        {
            if (index == 0U)
            {
                break;
            }
            next_index = (uint16)(index - 1U);
        }
        else
        {
            if ((uint16)(index + 1U) >= sample_count)
            {
                break;
            }
            next_index = (uint16)(index + 1U);
        }

        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            break;
        }

        NavPath_GetReplayPoint(index, &x0_m, &y0_m);
        NavPath_GetReplayPoint(next_index, &x1_m, &y1_m);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;

        kappa_1pm = NavPath_ReplayArcKappaFromIndex(index,
                                                    NAV_PATH_REPLAY_KAPPA_ARC_M);
        abs_kappa_1pm = NavPath_AbsF32(kappa_1pm);
        if ((ref_abs_kappa_1pm >= NAV_PATH_REPLAY_KAPPA_REV_MIN_1PM) &&
            (abs_kappa_1pm >= NAV_PATH_REPLAY_KAPPA_REV_MIN_1PM) &&
            ((ref_kappa_1pm * kappa_1pm) <
             NAV_PATH_REPLAY_KAPPA_REV_PRODUCT_1PM2) &&
            ((opposite_valid == 0U) ||
             (abs_kappa_1pm > opposite_abs_kappa_1pm)))
        {
            opposite_valid = 1U;
            opposite_abs_kappa_1pm = abs_kappa_1pm;
            opposite_kappa_1pm = kappa_1pm;
        }
        if ((distance_m <= ahead_m) &&
            (abs_kappa_1pm > best_abs_kappa_1pm))
        {
            best_abs_kappa_1pm = abs_kappa_1pm;
            best_kappa_1pm = kappa_1pm;
        }
    }

    if (best_abs_kappa_1pm <=
        NavPath_AbsF32(NavPath_ReplayArcKappaFromIndex(ahead_index,
                                                       NAV_PATH_REPLAY_KAPPA_ARC_M)))
    {
        best_kappa_1pm =
            NavPath_ReplayArcKappaFromIndex(ahead_index,
                                            NAV_PATH_REPLAY_KAPPA_ARC_M);
    }
    if (opposite_valid != 0U)
    {
        best_kappa_1pm = opposite_kappa_1pm;
    }

    g_nav_path_state.ref_kappa_ahead_1pm = best_kappa_1pm;
}

static void NavPath_ApplyAheadMotion(uint16 ref_index)
{
    uint16 ahead_index = NavPath_PreviewIndexAnyMotionFromRef(ref_index,
                                                              NAV_PATH_REPLAY_MOTION_AHEAD_M);

    g_nav_path_state.ref_speed_ahead_mps =
        NavPath_SpeedFromStore(g_nav_path_speed_cmps[ahead_index]);
    g_nav_path_state.ref_motion_dir_ahead =
        NavPath_MotionDirFromIndex(ahead_index);
}

static float NavPath_ReplayRemainingFromIndexM(uint16 ref_index)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 finish_index;
    uint16 index_delta;

    if ((sample_count == 0U) ||
        (g_nav_path_state.sample_interval_m <= 0.0f))
    {
        return 0.0f;
    }

    finish_index = (g_nav_path_replay_reverse != 0U) ?
                   0U : (uint16)(sample_count - 1U);
    if (ref_index > finish_index)
    {
        index_delta = (uint16)(ref_index - finish_index);
    }
    else
    {
        index_delta = (uint16)(finish_index - ref_index);
    }

    return (float)index_delta * g_nav_path_state.sample_interval_m;
}

static float NavPath_ReverseFinalYawBlend(float remaining_m)
{
    float blend;
    float lat_abs_m;
    float lat_fade;

    if (remaining_m >= NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_START_M)
    {
        return 0.0f;
    }
    if (remaining_m <= NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_FULL_M)
    {
        blend = 1.0f;
    }
    else
    {
        blend = (NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_START_M - remaining_m) /
                (NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_START_M -
                 NAV_PATH_REPLAY_REVERSE_FINAL_YAW_BLEND_FULL_M);
    }

    lat_abs_m = NavPath_AbsF32(g_nav_path_state.lateral_error_m);
    if (lat_abs_m <= NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_FADE_M)
    {
        return blend;
    }
    if (lat_abs_m >= NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_CUTOFF_M)
    {
        return 0.0f;
    }

    lat_fade = (NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_CUTOFF_M - lat_abs_m) /
               (NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_CUTOFF_M -
                NAV_PATH_REPLAY_REVERSE_FINAL_YAW_LAT_FADE_M);
    return blend * lat_fade;
}

static void NavPath_ApplyPreviewYaw(float x_m,
                                    float y_m,
                                    uint16 ref_index,
                                    float speed_mps)
{
    float preview_m = NavPath_PreviewDistanceM(speed_mps,
                                               g_nav_path_state.ref_kappa_1pm);
    uint16 preview_index = NavPath_PreviewIndexFromRef(ref_index, preview_m);
    float target_yaw_deg;
    float record_yaw_deg = g_nav_path_state.ref_yaw_deg;
    float preview_yaw_delta_deg;
    float preview_yaw_limit_deg = NAV_PATH_REPLAY_PREVIEW_YAW_MAX_DELTA_DEG;
    float final_yaw_blend = 0.0f;
    float preview_x_m;
    float preview_y_m;
    float vehicle_x_m;
    float vehicle_y_m;
    float dx_m;
    float dy_m;

    if (preview_index == ref_index)
    {
        return;
    }

    NavPath_GetReplayPoint(preview_index, &preview_x_m, &preview_y_m);
    vehicle_x_m = x_m - g_nav_path_base_x_m;
    vehicle_y_m = y_m - g_nav_path_base_y_m;
    dx_m = preview_x_m - vehicle_x_m;
    dy_m = preview_y_m - vehicle_y_m;
    if ((dx_m * dx_m + dy_m * dy_m) >= NAV_PATH_REPLAY_MIN_SEG_LEN2_M)
    {
        target_yaw_deg = NavPath_Wrap180(atan2f(dx_m, dy_m) * 180.0f / NAV_PI);
        if ((g_nav_path_replay_backward != 0U) ||
            (g_nav_path_state.ref_motion_dir < 0))
        {
            target_yaw_deg = NavPath_Wrap180(target_yaw_deg + 180.0f);
        }
    }
    else
    {
        target_yaw_deg = NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[preview_index]) +
                                         NavPath_ReplayYawFlipDeg() +
                                         g_nav_path_replay_yaw_offset_deg);
    }

    preview_yaw_delta_deg = NavPath_Wrap180(target_yaw_deg - record_yaw_deg);
    if (g_nav_path_replay_fixed_ins != 0U)
    {
        preview_yaw_limit_deg =
            NAV_PATH_REPLAY_PREVIEW_FIXED_INS_YAW_MAX_DELTA_DEG;
    }
    if (g_nav_path_state.ref_dir_change_distance_m <=
        NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_FINAL_M)
    {
        if (preview_yaw_limit_deg >
            NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_FINAL_LIMIT_DEG)
        {
            preview_yaw_limit_deg =
                NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_FINAL_LIMIT_DEG;
        }
    }
    else if (g_nav_path_state.ref_dir_change_distance_m <=
             NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_NEAR_M)
    {
        if (preview_yaw_limit_deg >
            NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_NEAR_LIMIT_DEG)
        {
            preview_yaw_limit_deg =
                NAV_PATH_REPLAY_PREVIEW_YAW_DIR_CHANGE_NEAR_LIMIT_DEG;
        }
    }
    if (NavPath_AbsF32(preview_yaw_delta_deg) > preview_yaw_limit_deg)
    {
        target_yaw_deg = record_yaw_deg;
    }

    if ((g_nav_path_replay_backward != 0U) ||
        (g_nav_path_state.ref_motion_dir < 0))
    {
        final_yaw_blend =
            NavPath_ReverseFinalYawBlend(NavPath_ReplayRemainingFromIndexM(ref_index));
        if (final_yaw_blend > 0.0f)
        {
            target_yaw_deg = NavPath_Wrap180(target_yaw_deg +
                             final_yaw_blend *
                             NavPath_Wrap180(record_yaw_deg - target_yaw_deg));
        }
    }

    g_nav_path_state.ref_yaw_deg = NavPath_Wrap180(target_yaw_deg);
}

static float NavPath_ReplayYawForIndex(uint16 index)
{
    return NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[index]) +
                           NavPath_ReplayYawFlipDeg() +
                           g_nav_path_replay_yaw_offset_deg);
}

static float NavPath_PreviewDistanceFixedInsMatchM(float kappa_1pm,
                                                   int8 motion_dir)
{
    float speed_abs_mps = NAV_PATH_REPLAY_FIXED_INS_MATCH_SPEED_MPS;
    float kappa_abs_1pm = NavPath_AbsF32(kappa_1pm);
    float preview_m = NAV_PATH_REPLAY_PREVIEW_BASE_M +
                      NAV_PATH_REPLAY_PREVIEW_SPEED_GAIN_S * speed_abs_mps;
    float max_preview_m = NAV_PATH_REPLAY_PREVIEW_MAX_M;

    if (kappa_abs_1pm > NAV_PATH_REPLAY_PREVIEW_TIGHT_KAPPA_1PM)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_TIGHT_MAX_M;
    }
    else if (kappa_abs_1pm > NAV_PATH_REPLAY_PREVIEW_MID_KAPPA_1PM)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_MID_MAX_M;
    }
    else if (kappa_abs_1pm > NAV_PATH_REPLAY_PREVIEW_SOFT_KAPPA_1PM)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_SOFT_MAX_M;
    }

    if ((motion_dir < 0) &&
        (max_preview_m > NAV_PATH_REPLAY_PREVIEW_REVERSE_MAX_M))
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_REVERSE_MAX_M;
    }
    if (max_preview_m > NAV_PATH_REPLAY_PREVIEW_FIXED_INS_MAX_M)
    {
        max_preview_m = NAV_PATH_REPLAY_PREVIEW_FIXED_INS_MAX_M;
    }

    return NavPath_ClipF32(preview_m,
                           NAV_PATH_REPLAY_PREVIEW_MIN_M,
                           max_preview_m);
}

static float NavPath_ReverseFixedInsArcKappaFromIndex(uint16 ref_index,
                                                      float arc_m)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 start_index = ref_index;
    uint16 end_index = ref_index;
    uint16 index = ref_index;
    int8 ref_motion_dir;
    float distance_m = 0.0f;
    float start_yaw_deg;
    float end_yaw_deg;
    float dyaw_deg;
    float kappa_1pm;

    if (sample_count == 0U)
    {
        return 0.0f;
    }
    if (ref_index >= sample_count)
    {
        ref_index = (uint16)(sample_count - 1U);
        start_index = ref_index;
        end_index = ref_index;
        index = ref_index;
    }
    if (sample_count < 2U)
    {
        return -NavPath_KappaFromStore(g_nav_path_kappa_milli[ref_index]);
    }

    ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    while (distance_m < arc_m)
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;

        if (index == 0U)
        {
            break;
        }
        next_index = (uint16)(index - 1U);
        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            break;
        }

        x0_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
        y0_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
        x1_m = NavPath_XFromStore(g_nav_path_x_cm[next_index]);
        y1_m = NavPath_YFromStore(g_nav_path_y_cm[next_index]);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;
        end_index = index;
    }

    if ((end_index == start_index) ||
        (distance_m < NAV_PATH_REPLAY_KAPPA_ARC_MIN_M))
    {
        return -NavPath_KappaFromStore(g_nav_path_kappa_milli[ref_index]);
    }

    start_yaw_deg =
        NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[start_index]) +
                        180.0f);
    end_yaw_deg =
        NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[end_index]) +
                        180.0f);
    dyaw_deg = NavPath_Wrap180(end_yaw_deg - start_yaw_deg);
    kappa_1pm = (dyaw_deg * NAV_PI / 180.0f) / distance_m;

    if (NavPath_AbsF32(kappa_1pm) > NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM)
    {
        kappa_1pm = (kappa_1pm >= 0.0f) ?
                    NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM :
                    -NAV_PATH_REPLAY_KAPPA_ARC_LIMIT_1PM;
    }

    return kappa_1pm;
}

static uint16 NavPath_PreviewIndexReverseFixedInsFromRef(uint16 ref_index,
                                                        float preview_m)
{
    uint16 index = ref_index;
    int8 ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    float distance_m = 0.0f;

    if ((g_nav_path_state.sample_count < 2U) ||
        (ref_index >= g_nav_path_state.sample_count))
    {
        return ref_index;
    }

    while (distance_m < preview_m)
    {
        uint16 next_index;
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float dx_m;
        float dy_m;

        if (index == 0U)
        {
            break;
        }
        next_index = (uint16)(index - 1U);
        if (NavPath_MotionDirFromIndex(next_index) != ref_motion_dir)
        {
            break;
        }

        x0_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
        y0_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
        x1_m = NavPath_XFromStore(g_nav_path_x_cm[next_index]);
        y1_m = NavPath_YFromStore(g_nav_path_y_cm[next_index]);
        dx_m = x1_m - x0_m;
        dy_m = y1_m - y0_m;
        distance_m += sqrtf(dx_m * dx_m + dy_m * dy_m);
        index = next_index;
    }

    return index;
}

static float NavPath_ReverseFixedInsMatchPreviewYaw(uint16 ref_index,
                                                    float vehicle_x_m,
                                                    float vehicle_y_m)
{
    float record_yaw_deg =
        NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[ref_index]) +
                        180.0f);
    float ref_kappa_1pm =
        NavPath_ReverseFixedInsArcKappaFromIndex(ref_index,
                                                NAV_PATH_REPLAY_KAPPA_ARC_M);
    int8 ref_motion_dir = NavPath_MotionDirFromIndex(ref_index);
    float preview_m =
        NavPath_PreviewDistanceFixedInsMatchM(ref_kappa_1pm, ref_motion_dir);
    uint16 preview_index =
        NavPath_PreviewIndexReverseFixedInsFromRef(ref_index, preview_m);
    float target_yaw_deg;
    float preview_x_m;
    float preview_y_m;
    float dx_m;
    float dy_m;
    float preview_yaw_delta_deg;

    if (preview_index == ref_index)
    {
        return record_yaw_deg;
    }

    preview_x_m = NavPath_XFromStore(g_nav_path_x_cm[preview_index]);
    preview_y_m = NavPath_YFromStore(g_nav_path_y_cm[preview_index]);
    dx_m = preview_x_m - vehicle_x_m;
    dy_m = preview_y_m - vehicle_y_m;
    if ((dx_m * dx_m + dy_m * dy_m) >= NAV_PATH_REPLAY_MIN_SEG_LEN2_M)
    {
        target_yaw_deg =
            NavPath_Wrap180(atan2f(dx_m, dy_m) * 180.0f / NAV_PI);
        if (ref_motion_dir < 0)
        {
            target_yaw_deg = NavPath_Wrap180(target_yaw_deg + 180.0f);
        }
    }
    else
    {
        target_yaw_deg =
            NavPath_Wrap180(NavPath_YawFromStore(g_nav_path_yaw_cdeg[preview_index]) +
                            180.0f);
    }

    preview_yaw_delta_deg = NavPath_Wrap180(target_yaw_deg - record_yaw_deg);
    if (NavPath_AbsF32(preview_yaw_delta_deg) >
        NAV_PATH_REPLAY_PREVIEW_FIXED_INS_YAW_MAX_DELTA_DEG)
    {
        target_yaw_deg = record_yaw_deg;
    }

    return NavPath_Wrap180(target_yaw_deg);
}

static void NavPath_ProjectReplayWindow(uint16 start_index,
                                        uint16 end_index,
                                        int8 expected_motion_dir,
                                        float vehicle_x_m,
                                        float vehicle_y_m,
                                        float vehicle_yaw_deg,
                                        uint16 expected_index,
                                        uint8 use_continuity_gate,
                                        uint8 *best_valid,
                                        uint16 *best_index,
                                        float *best_x_m,
                                        float *best_y_m,
                                        float *best_score)
{
    uint16 i;
    uint16 sample_count = g_nav_path_state.sample_count;
    float interval_m = g_nav_path_state.sample_interval_m;

    if ((sample_count < 2U) ||
        (best_valid == NULL) ||
        (best_index == NULL) ||
        (best_x_m == NULL) ||
        (best_y_m == NULL) ||
        (best_score == NULL))
    {
        return;
    }

    if (end_index >= (uint16)(sample_count - 1U))
    {
        end_index = (uint16)(sample_count - 2U);
    }
    if (start_index > end_index)
    {
        return;
    }

    for (i = start_index; i <= end_index; i++)
    {
        float x0_m;
        float y0_m;
        float x1_m;
        float y1_m;
        float seg_x_m;
        float seg_y_m;
        float seg_len2_m;
        float t;
        float proj_x_m;
        float proj_y_m;
        float dx_m;
        float dy_m;
        float dist2_m;
        float expected_arc_m;
        float score;
        uint16 candidate_index;
        uint16 expected_delta;
        uint16 last_delta = 0U;

        if (NavPath_SegmentMotionDir(i) != expected_motion_dir)
        {
            continue;
        }

        NavPath_GetReplayPoint(i, &x0_m, &y0_m);
        NavPath_GetReplayPoint((uint16)(i + 1U), &x1_m, &y1_m);
        seg_x_m = x1_m - x0_m;
        seg_y_m = y1_m - y0_m;
        seg_len2_m = seg_x_m * seg_x_m + seg_y_m * seg_y_m;
        if (seg_len2_m < NAV_PATH_REPLAY_MIN_SEG_LEN2_M)
        {
            continue;
        }

        t = ((vehicle_x_m - x0_m) * seg_x_m +
             (vehicle_y_m - y0_m) * seg_y_m) / seg_len2_m;
        if (t < 0.0f)
        {
            t = 0.0f;
        }
        else if (t > 1.0f)
        {
            t = 1.0f;
        }

        candidate_index = (t >= 0.5f) ? (uint16)(i + 1U) : i;
        if (NavPath_MotionDirFromIndex(candidate_index) != expected_motion_dir)
        {
            if (NavPath_MotionDirFromIndex((uint16)(i + 1U)) ==
                expected_motion_dir)
            {
                candidate_index = (uint16)(i + 1U);
            }
            else if (NavPath_MotionDirFromIndex(i) != expected_motion_dir)
            {
                continue;
            }
            else
            {
                candidate_index = i;
            }
        }

        if (use_continuity_gate != 0U)
        {
            uint16 frame_index_delta;
            uint16 max_frame_jump;
            float yaw_jump_deg;
            float candidate_yaw_deg = NavPath_ReplayYawForIndex(candidate_index);

            if (candidate_index > g_nav_path_replay_project_last_index)
            {
                frame_index_delta =
                    (uint16)(candidate_index - g_nav_path_replay_project_last_index);
            }
            else
            {
                frame_index_delta =
                    (uint16)(g_nav_path_replay_project_last_index - candidate_index);
            }

            max_frame_jump = (g_nav_path_replay_fixed_ins != 0U) ?
                             NAV_PATH_REPLAY_PROJECT_FIXED_INS_MAX_FRAME_JUMP :
                             NAV_PATH_REPLAY_PROJECT_MAX_FRAME_JUMP;
            yaw_jump_deg = (g_nav_path_replay_fixed_ins != 0U) ?
                           NAV_PATH_REPLAY_PROJECT_FIXED_INS_YAW_JUMP_DEG :
                           NAV_PATH_REPLAY_PROJECT_YAW_JUMP_DEG;
            if (g_nav_path_replay_rescue_project != 0U)
            {
                max_frame_jump = NAV_PATH_REPLAY_PROJECT_RESCUE_MAX_FRAME_JUMP;
                yaw_jump_deg = NAV_PATH_REPLAY_PROJECT_RESCUE_YAW_JUMP_DEG;
            }
            if (NavPath_MotionDirFromIndex(candidate_index) !=
                NavPath_MotionDirFromIndex(g_nav_path_replay_project_last_index))
            {
                if (max_frame_jump < NAV_PATH_REPLAY_PROJECT_DIR_CHANGE_MAX_FRAME_JUMP)
                {
                    max_frame_jump = NAV_PATH_REPLAY_PROJECT_DIR_CHANGE_MAX_FRAME_JUMP;
                }
                if (yaw_jump_deg < NAV_PATH_REPLAY_PROJECT_DIR_CHANGE_YAW_JUMP_DEG)
                {
                    yaw_jump_deg = NAV_PATH_REPLAY_PROJECT_DIR_CHANGE_YAW_JUMP_DEG;
                }
            }

            if ((frame_index_delta > max_frame_jump) ||
                (NavPath_AbsF32(NavPath_Wrap180(candidate_yaw_deg -
                                                g_nav_path_replay_project_last_yaw_deg)) >
                 yaw_jump_deg))
            {
                continue;
            }
        }

        proj_x_m = x0_m + t * seg_x_m;
        proj_y_m = y0_m + t * seg_y_m;
        dx_m = vehicle_x_m - proj_x_m;
        dy_m = vehicle_y_m - proj_y_m;
        dist2_m = dx_m * dx_m + dy_m * dy_m;

        if (candidate_index > expected_index)
        {
            expected_delta = (uint16)(candidate_index - expected_index);
        }
        else
        {
            expected_delta = (uint16)(expected_index - candidate_index);
        }
        expected_arc_m = interval_m * (float)expected_delta;
        score = dist2_m +
                ((g_nav_path_replay_rescue_project != 0U) ?
                 NAV_PATH_REPLAY_PROJECT_RESCUE_EXPECTED_ARC_WEIGHT :
                 ((g_nav_path_replay_fixed_ins != 0U) ?
                 NAV_PATH_REPLAY_PROJECT_FIXED_INS_EXPECTED_ARC_WEIGHT :
                  NAV_PATH_REPLAY_PROJECT_EXPECTED_ARC_WEIGHT)) *
                expected_arc_m * expected_arc_m;
        if (use_continuity_gate != 0U)
        {
            float last_arc_m;

            if (candidate_index > g_nav_path_replay_project_last_index)
            {
                last_delta = (uint16)(candidate_index -
                                      g_nav_path_replay_project_last_index);
            }
            else
            {
                last_delta = (uint16)(g_nav_path_replay_project_last_index -
                                      candidate_index);
            }

            last_arc_m = interval_m * (float)last_delta;
            score += ((g_nav_path_replay_rescue_project != 0U) ?
                      NAV_PATH_REPLAY_PROJECT_RESCUE_LAST_ARC_WEIGHT :
                      ((g_nav_path_replay_fixed_ins != 0U) ?
                      NAV_PATH_REPLAY_PROJECT_FIXED_INS_LAST_ARC_WEIGHT :
                       NAV_PATH_REPLAY_PROJECT_LAST_ARC_WEIGHT)) *
                     last_arc_m * last_arc_m;
        }
        if (g_nav_path_replay_rescue_project != 0U)
        {
            float yaw_rad = vehicle_yaw_deg * NAV_PI / 180.0f;
            float target_dx_m = proj_x_m - vehicle_x_m;
            float target_dy_m = proj_y_m - vehicle_y_m;
            float forward_dot_m = target_dx_m * sinf(yaw_rad) +
                                  target_dy_m * cosf(yaw_rad);
            float yaw_err_deg =
                NavPath_AbsF32(NavPath_Wrap180(vehicle_yaw_deg -
                                               NavPath_ReplayYawForIndex(candidate_index)));
            float yaw_norm = yaw_err_deg / 45.0f;
            if (forward_dot_m <
                NAV_PATH_REPLAY_PROJECT_RESCUE_BODY_FRONT_MIN_M)
            {
                float behind_m =
                    NAV_PATH_REPLAY_PROJECT_RESCUE_BODY_FRONT_MIN_M -
                    forward_dot_m;
                score += NAV_PATH_REPLAY_PROJECT_RESCUE_BODY_BACK_WEIGHT *
                         behind_m * behind_m;
            }
            score += NAV_PATH_REPLAY_PROJECT_RESCUE_YAW_WEIGHT *
                     yaw_norm * yaw_norm;
        }

        if ((*best_valid == 0U) || (score < *best_score))
        {
            *best_valid = 1U;
            *best_score = score;
            *best_x_m = proj_x_m;
            *best_y_m = proj_y_m;
            *best_index = candidate_index;
        }
    }
}

static uint8 NavPath_ProjectReplayRef(float x_m,
                                      float y_m,
                                      float yaw_deg,
                                      uint16 expected_index,
                                      uint16 *out_index,
                                      float *out_x_m,
                                      float *out_y_m)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 lower_pad;
    uint16 upper_pad;
    uint16 start_index;
    uint16 end_index;
    uint8 best_valid = 0U;
    uint16 best_index = expected_index;
    int8 expected_motion_dir;
    float vehicle_x_m = x_m - g_nav_path_base_x_m;
    float vehicle_y_m = y_m - g_nav_path_base_y_m;
    float vehicle_yaw_deg = NavPath_Wrap180(yaw_deg);
    float best_x_m = 0.0f;
    float best_y_m = 0.0f;
    float best_score = 0.0f;

    if ((sample_count < 2U) ||
        (out_index == NULL) ||
        (out_x_m == NULL) ||
        (out_y_m == NULL))
    {
        return 0U;
    }

    if (expected_index >= sample_count)
    {
        expected_index = (uint16)(sample_count - 1U);
    }
    expected_motion_dir = NavPath_MotionDirFromIndex(expected_index);

    if (g_nav_path_replay_rescue_project != 0U)
    {
        lower_pad = NAV_PATH_REPLAY_PROJECT_RESCUE_BACK_SAMPLES;
        upper_pad = NAV_PATH_REPLAY_PROJECT_RESCUE_FORWARD_SAMPLES;
    }
    else if (g_nav_path_replay_reverse != 0U)
    {
        lower_pad = NAV_PATH_REPLAY_PROJECT_FORWARD_SAMPLES;
        upper_pad = NAV_PATH_REPLAY_PROJECT_BACK_SAMPLES;
    }
    else
    {
        lower_pad = NAV_PATH_REPLAY_PROJECT_BACK_SAMPLES;
        upper_pad = NAV_PATH_REPLAY_PROJECT_FORWARD_SAMPLES;
    }

    start_index = (expected_index > lower_pad) ?
                  (uint16)(expected_index - lower_pad) : 0U;
    if (start_index >= (uint16)(sample_count - 1U))
    {
        start_index = (uint16)(sample_count - 2U);
    }

    if (((uint32)expected_index + (uint32)upper_pad) >=
        (uint32)(sample_count - 1U))
    {
        end_index = (uint16)(sample_count - 2U);
    }
    else
    {
        end_index = (uint16)(expected_index + upper_pad);
    }
    if (end_index < start_index)
    {
        end_index = start_index;
    }
    if ((g_nav_path_replay_rescue_project != 0U) &&
        (g_nav_path_replay_rescue_anchor_valid != 0U))
    {
        uint16 anchor_start_index;
        uint16 anchor_end_index;

        anchor_start_index =
            (g_nav_path_replay_rescue_anchor_index > lower_pad) ?
            (uint16)(g_nav_path_replay_rescue_anchor_index - lower_pad) : 0U;
        if (((uint32)g_nav_path_replay_rescue_anchor_index +
             (uint32)upper_pad) >= (uint32)(sample_count - 1U))
        {
            anchor_end_index = (uint16)(sample_count - 2U);
        }
        else
        {
            anchor_end_index =
                (uint16)(g_nav_path_replay_rescue_anchor_index + upper_pad);
        }
        if (anchor_start_index >= (uint16)(sample_count - 1U))
        {
            anchor_start_index = (uint16)(sample_count - 2U);
        }
        if (anchor_end_index < anchor_start_index)
        {
            anchor_end_index = anchor_start_index;
        }
        if (start_index < anchor_start_index)
        {
            start_index = anchor_start_index;
        }
        if (start_index > anchor_end_index)
        {
            start_index = anchor_end_index;
        }
        if (end_index > anchor_end_index)
        {
            end_index = anchor_end_index;
        }
        if (end_index < start_index)
        {
            end_index = start_index;
        }
    }

    NavPath_ProjectReplayWindow(start_index,
                                end_index,
                                expected_motion_dir,
                                vehicle_x_m,
                                vehicle_y_m,
                                vehicle_yaw_deg,
                                expected_index,
                                g_nav_path_replay_project_last_valid,
                                &best_valid,
                                &best_index,
                                &best_x_m,
                                &best_y_m,
                                &best_score);

    if (g_nav_path_replay_project_last_valid != 0U)
    {
        uint16 last_index = g_nav_path_replay_project_last_index;
        uint16 last_back_pad;
        uint16 last_forward_pad;
        uint16 last_start_index;
        uint16 last_end_index;
        int8 last_motion_dir;

        if (last_index >= sample_count)
        {
            last_index = (uint16)(sample_count - 1U);
        }

        if (g_nav_path_replay_rescue_project != 0U)
        {
            last_back_pad = NAV_PATH_REPLAY_PROJECT_RESCUE_BACK_SAMPLES;
            last_forward_pad = NAV_PATH_REPLAY_PROJECT_RESCUE_FORWARD_SAMPLES;
        }
        else if (g_nav_path_replay_fixed_ins != 0U)
        {
            last_back_pad = NAV_PATH_REPLAY_PROJECT_LAST_FIXED_INS_BACK_SAMPLES;
            last_forward_pad =
                NAV_PATH_REPLAY_PROJECT_LAST_FIXED_INS_FORWARD_SAMPLES;
        }
        else
        {
            last_back_pad = NAV_PATH_REPLAY_PROJECT_LAST_BACK_SAMPLES;
            last_forward_pad = NAV_PATH_REPLAY_PROJECT_LAST_FORWARD_SAMPLES;
        }

        if (g_nav_path_replay_reverse != 0U)
        {
            uint16 tmp = last_back_pad;
            last_back_pad = last_forward_pad;
            last_forward_pad = tmp;
        }

        last_start_index = (last_index > last_back_pad) ?
                           (uint16)(last_index - last_back_pad) : 0U;
        if (last_start_index >= (uint16)(sample_count - 1U))
        {
            last_start_index = (uint16)(sample_count - 2U);
        }
        if (((uint32)last_index + (uint32)last_forward_pad) >=
            (uint32)(sample_count - 1U))
        {
            last_end_index = (uint16)(sample_count - 2U);
        }
        else
        {
            last_end_index = (uint16)(last_index + last_forward_pad);
        }
        if (last_end_index < last_start_index)
        {
            last_end_index = last_start_index;
        }
        if ((g_nav_path_replay_rescue_project != 0U) &&
            (g_nav_path_replay_rescue_anchor_valid != 0U))
        {
            uint16 anchor_start_index =
                (g_nav_path_replay_rescue_anchor_index >
                 NAV_PATH_REPLAY_PROJECT_RESCUE_BACK_SAMPLES) ?
                (uint16)(g_nav_path_replay_rescue_anchor_index -
                         NAV_PATH_REPLAY_PROJECT_RESCUE_BACK_SAMPLES) : 0U;
            uint16 anchor_end_index;

            if (((uint32)g_nav_path_replay_rescue_anchor_index +
                 (uint32)NAV_PATH_REPLAY_PROJECT_RESCUE_FORWARD_SAMPLES) >=
                (uint32)(sample_count - 1U))
            {
                anchor_end_index = (uint16)(sample_count - 2U);
            }
            else
            {
                anchor_end_index =
                    (uint16)(g_nav_path_replay_rescue_anchor_index +
                             NAV_PATH_REPLAY_PROJECT_RESCUE_FORWARD_SAMPLES);
            }
            if (anchor_start_index >= (uint16)(sample_count - 1U))
            {
                anchor_start_index = (uint16)(sample_count - 2U);
            }
            if (anchor_end_index < anchor_start_index)
            {
                anchor_end_index = anchor_start_index;
            }
            if (last_start_index < anchor_start_index)
            {
                last_start_index = anchor_start_index;
            }
            if (last_start_index > anchor_end_index)
            {
                last_start_index = anchor_end_index;
            }
            if (last_end_index > anchor_end_index)
            {
                last_end_index = anchor_end_index;
            }
            if (last_end_index < last_start_index)
            {
                last_end_index = last_start_index;
            }
        }

        last_motion_dir = NavPath_MotionDirFromIndex(last_index);
        NavPath_ProjectReplayWindow(last_start_index,
                                    last_end_index,
                                    last_motion_dir,
                                    vehicle_x_m,
                                    vehicle_y_m,
                                    vehicle_yaw_deg,
                                    expected_index,
                                    1U,
                                    &best_valid,
                                    &best_index,
                                    &best_x_m,
                                    &best_y_m,
                                    &best_score);
    }

    {
        float dir_change_distance_m;
        uint16 dir_change_samples;

        NavPath_DirChangeInfo(expected_index,
                              NAV_PATH_REPLAY_DIR_CHANGE_SCAN_M,
                              &dir_change_distance_m,
                              &dir_change_samples);
        if ((g_nav_path_replay_rescue_project == 0U) &&
            (dir_change_distance_m <=
             NAV_PATH_REPLAY_DIR_CHANGE_STATIONARY_SWITCH_M) &&
            (dir_change_samples >=
             NAV_PATH_REPLAY_DIR_CHANGE_STATIONARY_MIN_SAMPLES))
        {
            uint16 alt_index;
            uint16 alt_start_index;
            uint16 alt_end_index;
            uint16 low_index;
            uint16 high_index;
            uint8 alt_valid = 0U;
            uint16 alt_best_index = expected_index;
            float alt_best_x_m = 0.0f;
            float alt_best_y_m = 0.0f;
            float alt_best_score = 0.0f;
            int8 alt_motion_dir;

            if (g_nav_path_replay_reverse != 0U)
            {
                alt_index = (expected_index > dir_change_samples) ?
                            (uint16)(expected_index - dir_change_samples) : 0U;
            }
            else if (((uint32)expected_index + (uint32)dir_change_samples) >=
                     (uint32)sample_count)
            {
                alt_index = (uint16)(sample_count - 1U);
            }
            else
            {
                alt_index = (uint16)(expected_index + dir_change_samples);
            }

            low_index = (alt_index < expected_index) ? alt_index : expected_index;
            high_index = (alt_index > expected_index) ? alt_index : expected_index;
            alt_start_index = (low_index > NAV_PATH_REPLAY_PROJECT_BACK_SAMPLES) ?
                              (uint16)(low_index - NAV_PATH_REPLAY_PROJECT_BACK_SAMPLES) : 0U;
            if (alt_start_index >= (uint16)(sample_count - 1U))
            {
                alt_start_index = (uint16)(sample_count - 2U);
            }
            if (((uint32)high_index + (uint32)NAV_PATH_REPLAY_PROJECT_FORWARD_SAMPLES) >=
                (uint32)(sample_count - 1U))
            {
                alt_end_index = (uint16)(sample_count - 2U);
            }
            else
            {
                alt_end_index =
                    (uint16)(high_index + NAV_PATH_REPLAY_PROJECT_FORWARD_SAMPLES);
            }
            if (alt_end_index < alt_start_index)
            {
                alt_end_index = alt_start_index;
            }
            alt_motion_dir = NavPath_MotionDirFromIndex(alt_index);
            NavPath_ProjectReplayWindow(alt_start_index,
                                        alt_end_index,
                                        alt_motion_dir,
                                        vehicle_x_m,
                                        vehicle_y_m,
                                        vehicle_yaw_deg,
                                        alt_index,
                                        g_nav_path_replay_project_last_valid,
                                        &alt_valid,
                                        &alt_best_index,
                                        &alt_best_x_m,
                                        &alt_best_y_m,
                                        &alt_best_score);
            if ((alt_valid != 0U) &&
                ((best_valid == 0U) ||
                 (alt_best_score <=
                  (best_score + NAV_PATH_REPLAY_PROJECT_STATIONARY_SWITCH_SCORE_MARGIN))))
            {
                best_valid = 1U;
                best_index = alt_best_index;
                best_x_m = alt_best_x_m;
                best_y_m = alt_best_y_m;
                best_score = alt_best_score;
            }
        }
    }

    if (best_valid == 0U)
    {
        return 0U;
    }

    *out_index = best_index;
    *out_x_m = best_x_m;
    *out_y_m = best_y_m;
    g_nav_path_replay_project_last_valid = 1U;
    g_nav_path_replay_project_last_index = best_index;
    g_nav_path_replay_project_last_yaw_deg = NavPath_ReplayYawForIndex(best_index);
    return 1U;
}

static void NavPath_SyncReplayProgressToIndex(uint16 ref_index,
                                              float *path_distance_m,
                                              uint8 *distance_complete)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    float target_progress_m;
    float progress_err_m;
    float path_len_m;

    if ((sample_count == 0U) ||
        (g_nav_path_state.sample_interval_m <= 0.0f) ||
        (path_distance_m == NULL))
    {
        return;
    }

    if (ref_index >= sample_count)
    {
        ref_index = (uint16)(sample_count - 1U);
    }

    if (g_nav_path_replay_reverse != 0U)
    {
        target_progress_m =
            (float)((uint16)(sample_count - 1U) - ref_index) *
            g_nav_path_state.sample_interval_m;
    }
    else
    {
        target_progress_m = (float)ref_index * g_nav_path_state.sample_interval_m;
    }

    progress_err_m = target_progress_m - g_nav_path_progress_m;
    if (NavPath_AbsF32(progress_err_m) <=
        NAV_PATH_REPLAY_PROGRESS_SYNC_DEADBAND_M)
    {
        return;
    }

    if ((g_nav_path_replay_progress_gate != 0U) &&
        ((NavPath_AbsF32(g_nav_path_state.lateral_error_m) > 0.55f) ||
         (NavPath_AbsF32(g_nav_path_state.yaw_error_deg) > 42.0f)))
    {
        if (progress_err_m > 0.18f)
        {
            progress_err_m = 0.18f;
        }
        else if (progress_err_m < -0.18f)
        {
            progress_err_m = -0.18f;
        }
        g_nav_path_progress_m +=
            progress_err_m * NAV_PATH_REPLAY_PROGRESS_SYNC_BLEND;
    }
    else if (NavPath_AbsF32(progress_err_m) >=
             NAV_PATH_REPLAY_PROGRESS_SYNC_HARD_M)
    {
        g_nav_path_progress_m = target_progress_m;
    }
    else
    {
        g_nav_path_progress_m +=
            progress_err_m * NAV_PATH_REPLAY_PROGRESS_SYNC_BLEND;
    }

    path_len_m = (float)(sample_count - 1U) *
                 g_nav_path_state.sample_interval_m;
    if (g_nav_path_progress_m < 0.0f)
    {
        g_nav_path_progress_m = 0.0f;
    }
    if (g_nav_path_progress_m > path_len_m)
    {
        g_nav_path_progress_m = path_len_m;
    }

    *path_distance_m = g_nav_path_progress_m;
    g_nav_path_state.progress_m = g_nav_path_progress_m;
    if ((distance_complete != NULL) && (g_nav_path_progress_m < path_len_m))
    {
        *distance_complete = 0U;
    }
}

static uint8 NavPath_ReplayFinishReady(float path_distance_m,
                                       float x_m,
                                       float y_m,
                                       uint16 ref_index,
                                       uint8 distance_complete,
                                       float speed_mps)
{
    uint16 sample_count = g_nav_path_state.sample_count;
    uint16 finish_index;
    uint8 index_close = 0U;
    uint8 regular_finish_allowed = distance_complete;
    float vehicle_x_m;
    float vehicle_y_m;
    float finish_x_m;
    float finish_y_m;
    float dx_m;
    float dy_m;
    float dist2_m;
    float path_len_m;
    float progress_remaining_m;
    float lat_abs_m;
    float yaw_abs_deg;
    float speed_abs_mps;

    if ((sample_count == 0U) ||
        (g_nav_path_state.sample_interval_m <= 0.0f))
    {
        return 0U;
    }

    finish_index = (g_nav_path_replay_reverse != 0U) ?
                   0U : (uint16)(sample_count - 1U);
    if (g_nav_path_replay_reverse != 0U)
    {
        index_close = (ref_index <= NAV_PATH_REPLAY_FINISH_INDEX_TOL_SAMPLES) ? 1U : 0U;
    }
    else if ((uint16)(ref_index + NAV_PATH_REPLAY_FINISH_INDEX_TOL_SAMPLES) >= finish_index)
    {
        index_close = 1U;
    }

    path_len_m = (float)(sample_count - 1U) * g_nav_path_state.sample_interval_m;
    if ((index_close == 0U) &&
        (path_distance_m < (path_len_m + NAV_PATH_REPLAY_FINISH_OVERRUN_M)))
    {
        regular_finish_allowed = 0U;
    }

    vehicle_x_m = x_m - g_nav_path_base_x_m;
    vehicle_y_m = y_m - g_nav_path_base_y_m;
    NavPath_GetReplayPoint(finish_index, &finish_x_m, &finish_y_m);
    dx_m = vehicle_x_m - finish_x_m;
    dy_m = vehicle_y_m - finish_y_m;
    dist2_m = dx_m * dx_m + dy_m * dy_m;
    speed_abs_mps = NavPath_AbsF32(speed_mps);

    if ((index_close != 0U) &&
        (dist2_m <= (NAV_PATH_REPLAY_FINISH_TERMINAL_REACH_M *
                    NAV_PATH_REPLAY_FINISH_TERMINAL_REACH_M)) &&
        (NavPath_AbsF32(g_nav_path_state.lateral_error_m) <=
         NAV_PATH_REPLAY_FINISH_TERMINAL_LAT_M) &&
        (NavPath_AbsF32(g_nav_path_state.yaw_error_deg) <=
         NAV_PATH_REPLAY_FINISH_TERMINAL_YAW_DEG) &&
        (speed_abs_mps <= NAV_PATH_REPLAY_FINISH_TERMINAL_SPEED_MPS))
    {
        if (g_nav_path_finish_confirm_count < NAV_PATH_REPLAY_FINISH_CONFIRM_COUNT)
        {
            g_nav_path_finish_confirm_count++;
        }
        return (g_nav_path_finish_confirm_count >=
                NAV_PATH_REPLAY_FINISH_CONFIRM_COUNT) ? 1U : 0U;
    }

    if (speed_abs_mps > NAV_PATH_REPLAY_FINISH_MAX_SPEED_MPS)
    {
        g_nav_path_finish_confirm_count = 0U;
        return 0U;
    }

    if ((regular_finish_allowed != 0U) &&
        (dist2_m <= (NAV_PATH_REPLAY_FINISH_REACH_M *
                    NAV_PATH_REPLAY_FINISH_REACH_M)))
    {
        g_nav_path_finish_confirm_count = 0U;
        return 1U;
    }

    progress_remaining_m = path_len_m - path_distance_m;
    if (progress_remaining_m < 0.0f)
    {
        progress_remaining_m = 0.0f;
    }
    lat_abs_m = NavPath_AbsF32(g_nav_path_state.lateral_error_m);
    yaw_abs_deg = NavPath_AbsF32(g_nav_path_state.yaw_error_deg);
    if ((distance_complete == 0U) &&
        (index_close != 0U) &&
        (progress_remaining_m <= NAV_PATH_REPLAY_FINISH_EARLY_PROGRESS_M) &&
        (dist2_m <= (NAV_PATH_REPLAY_FINISH_EARLY_REACH_M *
                    NAV_PATH_REPLAY_FINISH_EARLY_REACH_M)) &&
        (lat_abs_m <= NAV_PATH_REPLAY_FINISH_EARLY_LAT_M) &&
        (yaw_abs_deg <= NAV_PATH_REPLAY_FINISH_EARLY_YAW_DEG))
    {
        if (g_nav_path_finish_confirm_count < NAV_PATH_REPLAY_FINISH_CONFIRM_COUNT)
        {
            g_nav_path_finish_confirm_count++;
        }
        return (g_nav_path_finish_confirm_count >=
                NAV_PATH_REPLAY_FINISH_CONFIRM_COUNT) ? 1U : 0U;
    }

    g_nav_path_finish_confirm_count = 0U;
    return 0U;
}

static void NavPath_RecordSample(float x_m, float y_m, float yaw_deg, float speed_mps)
{
    uint16 index = g_nav_path_state.sample_count;
    float record_yaw_deg;
    float yaw_origin_rad;
    float sin_origin;
    float cos_origin;
    float record_x_m;
    float record_y_m;

    if (index >= NAV_PATH_MAX_SAMPLES)
    {
        return;
    }

    if (g_nav_path_record_yaw_origin_valid == 0U)
    {
        g_nav_path_record_yaw_origin_deg = yaw_deg;
        g_nav_path_record_yaw_origin_valid = 1U;
    }
    record_yaw_deg = NavPath_Wrap180(yaw_deg - g_nav_path_record_yaw_origin_deg);
    yaw_origin_rad = g_nav_path_record_yaw_origin_deg * NAV_PI / 180.0f;
    sin_origin = sinf(yaw_origin_rad);
    cos_origin = cosf(yaw_origin_rad);
    record_x_m = cos_origin * x_m - sin_origin * y_m;
    record_y_m = sin_origin * x_m + cos_origin * y_m;

    g_nav_path_x_cm[index] = NavPath_FloatToInt16(record_x_m, NAV_PATH_POS_SCALE_CM);
    g_nav_path_y_cm[index] = NavPath_FloatToInt16(record_y_m, NAV_PATH_POS_SCALE_CM);
    g_nav_path_yaw_cdeg[index] = NavPath_YawToStore(record_yaw_deg);
    g_nav_path_speed_cmps[index] = NavPath_FloatToInt16(speed_mps, NAV_PATH_SPEED_SCALE_CMPS);

    if (index == 0U)
    {
        g_nav_path_kappa_milli[index] = 0;
    }
    else
    {
        float last_yaw_deg = NavPath_YawFromStore(g_nav_path_yaw_cdeg[index - 1U]);
        float dyaw_rad = NavPath_Wrap180(record_yaw_deg - last_yaw_deg) * NAV_PI / 180.0f;
        float kappa_1pm = dyaw_rad / g_nav_path_state.sample_interval_m;
        int16 kappa_store = NavPath_KappaToStore(kappa_1pm);
        g_nav_path_kappa_milli[index - 1U] = kappa_store;
        g_nav_path_kappa_milli[index] = kappa_store;
    }

    g_nav_path_state.sample_count++;
    g_nav_path_state.valid = 1U;
}

void nav_path_init(void)
{
    nav_path_clear();
}

void nav_path_clear(void)
{
    uint8 last_sd_result = g_nav_path_state.last_sd_result;
    uint8 last_flash_result = g_nav_path_state.last_flash_result;
    uint8 last_flash_load_result = g_nav_path_state.last_flash_load_result;

    if (g_nav_path_save_status_inited == 0U)
    {
        last_sd_result = NAV_PATH_SAVE_NOT_TRIED;
        last_flash_result = NAV_PATH_SAVE_NOT_TRIED;
        last_flash_load_result = NAV_PATH_SAVE_NOT_TRIED;
        g_nav_path_save_status_inited = 1U;
    }

    memset(g_nav_path_x_cm, 0, sizeof(g_nav_path_x_cm));
    memset(g_nav_path_y_cm, 0, sizeof(g_nav_path_y_cm));
    memset(g_nav_path_yaw_cdeg, 0, sizeof(g_nav_path_yaw_cdeg));
    memset(g_nav_path_kappa_milli, 0, sizeof(g_nav_path_kappa_milli));
    memset(g_nav_path_speed_cmps, 0, sizeof(g_nav_path_speed_cmps));
    memset(&g_nav_path_state, 0, sizeof(g_nav_path_state));
    g_nav_path_state.mode = NAV_PATH_MODE_IDLE;
    g_nav_path_state.sample_interval_m = NAV_PATH_DEFAULT_INTERVAL_M;
    g_nav_path_state.ref_motion_dir = 1;
    g_nav_path_state.last_sd_result = last_sd_result;
    g_nav_path_state.last_flash_result = last_flash_result;
    g_nav_path_state.last_flash_load_result = last_flash_load_result;
    g_nav_path_record_next_m = 0.0f;
    g_nav_path_base_distance_m = 0.0f;
    g_nav_path_base_x_m = 0.0f;
    g_nav_path_base_y_m = 0.0f;
    g_nav_path_progress_m = 0.0f;
    g_nav_path_last_distance_m = 0.0f;
    g_nav_path_record_yaw_origin_deg = 0.0f;
    g_nav_path_base_valid = 0U;
    g_nav_path_last_distance_valid = 0U;
    g_nav_path_record_yaw_origin_valid = 0U;
    g_nav_path_replay_reverse = 0U;
    g_nav_path_replay_backward = 0U;
    g_nav_path_replay_align_pending = 0U;
    g_nav_path_replay_fixed_ins = 0U;
    g_nav_path_replay_progress_gate = 0U;
    g_nav_path_replay_rescue_project = 0U;
    g_nav_path_replay_rescue_anchor_valid = 0U;
    g_nav_path_replay_rescue_anchor_index = 0U;
    g_nav_path_replay_project_last_valid = 0U;
    g_nav_path_replay_project_last_index = 0U;
    g_nav_path_replay_project_last_yaw_deg = 0.0f;
    g_nav_path_finish_confirm_count = 0U;
    g_nav_path_replay_anchor_x_m = 0.0f;
    g_nav_path_replay_anchor_y_m = 0.0f;
    g_nav_path_replay_yaw_offset_deg = 0.0f;
}

uint8 nav_path_start_record(float sample_interval_m)
{
    if (nav_path_sd_save_is_busy() != 0U)
    {
        return 9U;
    }

    nav_path_clear();

    if (sample_interval_m <= 0.0f)
    {
        sample_interval_m = NAV_PATH_DEFAULT_INTERVAL_M;
    }

    g_nav_path_state.mode = NAV_PATH_MODE_RECORD;
    g_nav_path_state.active = 1U;
    g_nav_path_state.finished = 0U;
    g_nav_path_state.valid = 0U;
    g_nav_path_state.last_sd_result = NAV_PATH_SAVE_NOT_TRIED;
    g_nav_path_state.last_flash_result = NAV_PATH_SAVE_NOT_TRIED;
    g_nav_path_state.sample_interval_m = sample_interval_m;
    g_nav_path_record_next_m = 0.0f;
    g_nav_path_progress_m = 0.0f;
    g_nav_path_last_distance_m = 0.0f;
    g_nav_path_record_yaw_origin_deg = 0.0f;
    g_nav_path_base_valid = 0U;
    g_nav_path_last_distance_valid = 0U;
    g_nav_path_record_yaw_origin_valid = 0U;
    g_nav_path_finish_confirm_count = 0U;
    return 0U;
}

static uint8 NavPath_StartReplayFromStep(uint8 reverse,
                                         uint8 backward,
                                         uint8 align_yaw,
                                         uint16 start_step,
                                         uint8 fixed_ins)
{
    uint16 start_index;
    uint16 last_index;

    if (g_nav_path_state.sample_count == 0U)
    {
        return 1U;
    }

    last_index = (uint16)(g_nav_path_state.sample_count - 1U);
    if (start_step > last_index)
    {
        start_step = last_index;
    }

    g_nav_path_replay_reverse = (reverse != 0U) ? 1U : 0U;
    g_nav_path_replay_backward = ((reverse != 0U) && (backward != 0U)) ? 1U : 0U;
    g_nav_path_replay_align_pending = (align_yaw != 0U) ? 1U : 0U;
    g_nav_path_replay_fixed_ins = (fixed_ins != 0U) ? 1U : 0U;
    g_nav_path_replay_progress_gate = 0U;
    g_nav_path_replay_rescue_project = 0U;
    g_nav_path_replay_rescue_anchor_valid = 0U;
    g_nav_path_replay_rescue_anchor_index = 0U;
    g_nav_path_replay_project_last_valid = 0U;
    g_nav_path_replay_project_last_index = 0U;
    g_nav_path_replay_project_last_yaw_deg = 0.0f;
    g_nav_path_replay_yaw_offset_deg = 0.0f;
    start_index = (g_nav_path_replay_reverse != 0U) ?
                  (uint16)(last_index - start_step) : start_step;
    if (g_nav_path_replay_fixed_ins != 0U)
    {
        g_nav_path_replay_anchor_x_m = 0.0f;
        g_nav_path_replay_anchor_y_m = 0.0f;
    }
    else
    {
        g_nav_path_replay_anchor_x_m = NavPath_XFromStore(g_nav_path_x_cm[start_index]);
        g_nav_path_replay_anchor_y_m = NavPath_YFromStore(g_nav_path_y_cm[start_index]);
    }

    g_nav_path_state.mode = NAV_PATH_MODE_REPLAY;
    g_nav_path_state.active = 1U;
    g_nav_path_state.finished = 0U;
    g_nav_path_state.valid = 1U;
    g_nav_path_state.replay_index = start_index;
    NavPath_SetRefFromIndex(start_index);
    g_nav_path_replay_project_last_index = start_index;
    g_nav_path_replay_project_last_yaw_deg = g_nav_path_state.ref_yaw_deg;
    g_nav_path_replay_project_last_valid = 1U;
    g_nav_path_state.yaw_error_deg = 0.0f;
    g_nav_path_state.lateral_error_m = 0.0f;
    g_nav_path_progress_m = (float)start_step * g_nav_path_state.sample_interval_m;
    g_nav_path_last_distance_m = 0.0f;
    g_nav_path_base_valid = 0U;
    g_nav_path_last_distance_valid = 0U;
    return 0U;
}

static uint8 NavPath_StartReplay(uint8 reverse, uint8 backward, uint8 align_yaw)
{
    return NavPath_StartReplayFromStep(reverse, backward, align_yaw, 0U, 0U);
}

uint8 nav_path_start_replay(void)
{
    return NavPath_StartReplay(0U, 0U, 1U);
}

uint8 nav_path_start_replay_reverse(void)
{
    return NavPath_StartReplay(1U, 0U, 1U);
}

uint8 nav_path_start_replay_reverse_fixed_yaw(void)
{
    return NavPath_StartReplay(1U, 0U, 0U);
}

uint8 nav_path_start_replay_backward(void)
{
    return NavPath_StartReplay(1U, 1U, 1U);
}

uint8 nav_path_start_replay_backward_fixed_yaw(void)
{
    return NavPath_StartReplay(1U, 1U, 0U);
}

uint8 nav_path_start_replay_reverse_fixed_yaw_from_step(uint16 start_step)
{
    return NavPath_StartReplayFromStep(1U, 0U, 0U, start_step, 0U);
}

uint8 nav_path_start_replay_reverse_fixed_ins_from_step(uint16 start_step)
{
    return NavPath_StartReplayFromStep(1U, 0U, 0U, start_step, 1U);
}

void nav_path_set_replay_progress_gate(uint8 enable)
{
    g_nav_path_replay_progress_gate = (enable != 0U) ? 1U : 0U;
}

void nav_path_set_replay_rescue_project(uint8 enable)
{
    if (enable != 0U)
    {
        if ((g_nav_path_replay_rescue_project == 0U) ||
            (g_nav_path_replay_rescue_anchor_valid == 0U))
        {
            g_nav_path_replay_rescue_anchor_index =
                g_nav_path_state.replay_index;
            g_nav_path_replay_rescue_anchor_valid = 1U;
        }
        g_nav_path_replay_rescue_project = 1U;
    }
    else
    {
        g_nav_path_replay_rescue_project = 0U;
        g_nav_path_replay_rescue_anchor_valid = 0U;
        g_nav_path_replay_rescue_anchor_index = 0U;
    }
}

uint8 nav_path_find_reverse_fixed_ins_match(float x_m,
                                            float y_m,
                                            float yaw_deg,
                                            uint16 max_step,
                                            nav_path_replay_match_t *out_match)
{
    uint16 last_index;
    uint16 step;
    uint16 best_step = 0U;
    uint16 best_index = 0U;
    uint8 best_valid = 0U;
    float best_score = 0.0f;
    float best_ref_x_m = 0.0f;
    float best_ref_y_m = 0.0f;
    float best_ref_yaw_deg = 0.0f;
    float best_distance_m = 0.0f;
    float best_lateral_m = 0.0f;
    float best_longitudinal_m = 0.0f;
    float best_yaw_error_deg = 0.0f;

    if ((out_match == NULL) || (g_nav_path_state.sample_count == 0U))
    {
        return 1U;
    }

    memset(out_match, 0, sizeof(*out_match));
    last_index = (uint16)(g_nav_path_state.sample_count - 1U);
    if (max_step > last_index)
    {
        max_step = last_index;
    }

    yaw_deg = NavPath_Wrap180(yaw_deg);
    for (step = 0U; step <= max_step; step++)
    {
        uint16 index = (uint16)(last_index - step);
        float ref_x_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
        float ref_y_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
        float ref_yaw_deg =
            NavPath_ReverseFixedInsMatchPreviewYaw(index, x_m, y_m);
        float yaw_rad = ref_yaw_deg * NAV_PI / 180.0f;
        float forward_x = sinf(yaw_rad);
        float forward_y = cosf(yaw_rad);
        float dx_m = x_m - ref_x_m;
        float dy_m = y_m - ref_y_m;
        float lateral_m = forward_x * dy_m - forward_y * dx_m;
        float longitudinal_m = forward_x * dx_m + forward_y * dy_m;
        float distance_m = sqrtf(dx_m * dx_m + dy_m * dy_m);
        float yaw_error_deg = NavPath_Wrap180(yaw_deg - ref_yaw_deg);
        float yaw_abs_deg = NavPath_AbsF32(yaw_error_deg);
        float lateral_abs_m = NavPath_AbsF32(lateral_m);
        float longitudinal_abs_m = NavPath_AbsF32(longitudinal_m);
        float score = distance_m +
                      1.80f * lateral_abs_m +
                      0.32f * longitudinal_abs_m +
                      yaw_abs_deg * 0.040f;

        if ((best_valid == 0U) || (score < best_score))
        {
            best_valid = 1U;
            best_score = score;
            best_step = step;
            best_index = index;
            best_ref_x_m = ref_x_m;
            best_ref_y_m = ref_y_m;
            best_ref_yaw_deg = ref_yaw_deg;
            best_distance_m = distance_m;
            best_lateral_m = lateral_m;
            best_longitudinal_m = longitudinal_m;
            best_yaw_error_deg = yaw_error_deg;
        }
    }

    if (best_valid == 0U)
    {
        return 2U;
    }

    out_match->valid = 1U;
    out_match->start_step = best_step;
    out_match->replay_index = best_index;
    out_match->ref_x_m = best_ref_x_m;
    out_match->ref_y_m = best_ref_y_m;
    out_match->ref_yaw_deg = best_ref_yaw_deg;
    out_match->distance_m = best_distance_m;
    out_match->lateral_error_m = best_lateral_m;
    out_match->longitudinal_error_m = best_longitudinal_m;
    out_match->yaw_error_deg = best_yaw_error_deg;
    out_match->score = best_score;
    return 0U;
}

void nav_path_stop(void)
{
    g_nav_path_state.active = 0U;
    g_nav_path_state.mode = NAV_PATH_MODE_IDLE;
    g_nav_path_replay_align_pending = 0U;
    g_nav_path_replay_backward = 0U;
    g_nav_path_replay_fixed_ins = 0U;
    g_nav_path_replay_progress_gate = 0U;
    g_nav_path_replay_rescue_project = 0U;
    g_nav_path_replay_rescue_anchor_valid = 0U;
    g_nav_path_replay_rescue_anchor_index = 0U;
    g_nav_path_replay_project_last_valid = 0U;
    g_nav_path_replay_project_last_index = 0U;
    g_nav_path_replay_project_last_yaw_deg = 0.0f;
    g_nav_path_finish_confirm_count = 0U;
    g_nav_path_last_distance_valid = 0U;
}

void nav_path_update(float distance_m, float x_m, float y_m, float yaw_deg, float speed_mps)
{
    float path_distance_m;
    float distance_delta_m;
    uint32 index;

    if (g_nav_path_state.active == 0U)
    {
        return;
    }

    if (g_nav_path_base_valid == 0U)
    {
        g_nav_path_base_distance_m = distance_m;
        if (g_nav_path_state.mode == NAV_PATH_MODE_REPLAY)
        {
            if (g_nav_path_replay_fixed_ins != 0U)
            {
                g_nav_path_base_x_m = 0.0f;
                g_nav_path_base_y_m = 0.0f;
            }
            else
            {
                g_nav_path_base_x_m = x_m - g_nav_path_state.ref_x_m;
                g_nav_path_base_y_m = y_m - g_nav_path_state.ref_y_m;
            }
        }
        else
        {
            g_nav_path_base_x_m = x_m;
            g_nav_path_base_y_m = y_m;
            g_nav_path_progress_m = 0.0f;
        }
        g_nav_path_last_distance_m = distance_m;
        g_nav_path_base_valid = 1U;
        g_nav_path_last_distance_valid = 1U;
    }

    if (g_nav_path_last_distance_valid == 0U)
    {
        g_nav_path_last_distance_m = distance_m;
        g_nav_path_last_distance_valid = 1U;
    }

    distance_delta_m = distance_m - g_nav_path_last_distance_m;
    if (distance_delta_m < 0.0f)
    {
        distance_delta_m = -distance_delta_m;
    }
    if (g_nav_path_state.mode == NAV_PATH_MODE_REPLAY)
    {
        distance_delta_m *= NavPath_ReplayProgressScale(speed_mps);
    }
    g_nav_path_progress_m += distance_delta_m;
    g_nav_path_last_distance_m = distance_m;

    if ((g_nav_path_state.mode == NAV_PATH_MODE_RECORD) ||
        (g_nav_path_state.mode == NAV_PATH_MODE_REPLAY))
    {
        path_distance_m = g_nav_path_progress_m;
    }
    else if (g_nav_path_replay_backward != 0U)
    {
        path_distance_m = g_nav_path_base_distance_m - distance_m;
    }
    else
    {
        path_distance_m = distance_m - g_nav_path_base_distance_m;
    }
    g_nav_path_state.progress_m = path_distance_m;

    yaw_deg = NavPath_Wrap180(yaw_deg);

    if (g_nav_path_state.mode == NAV_PATH_MODE_RECORD)
    {
        while ((path_distance_m >= g_nav_path_record_next_m) &&
               (g_nav_path_state.sample_count < NAV_PATH_MAX_SAMPLES))
        {
            NavPath_RecordSample(x_m - g_nav_path_base_x_m,
                                 y_m - g_nav_path_base_y_m,
                                 yaw_deg,
                                 speed_mps);
            g_nav_path_record_next_m += g_nav_path_state.sample_interval_m;
        }

        if (g_nav_path_state.sample_count >= NAV_PATH_MAX_SAMPLES)
        {
            g_nav_path_state.finished = 1U;
            g_nav_path_state.active = 0U;
            g_nav_path_state.mode = NAV_PATH_MODE_IDLE;
        }
        return;
    }

    if (g_nav_path_state.mode == NAV_PATH_MODE_REPLAY)
    {
        uint32 step_index;
        uint8 distance_complete = 0U;
        uint16 ref_index;
        float projected_x_m;
        float projected_y_m;

        if ((g_nav_path_state.sample_count == 0U) ||
            (g_nav_path_state.sample_interval_m <= 0.0f))
        {
            g_nav_path_state.valid = 0U;
            return;
        }

        NavPath_UpdateReplayAlignment(yaw_deg);

        if (path_distance_m <= 0.0f)
        {
            step_index = 0U;
        }
        else
        {
            step_index = (uint32)((path_distance_m / g_nav_path_state.sample_interval_m) + 0.5f);
        }

        if (step_index >= (uint32)g_nav_path_state.sample_count)
        {
            step_index = (uint32)g_nav_path_state.sample_count - 1U;
            distance_complete = 1U;
        }

        if (g_nav_path_replay_reverse != 0U)
        {
            uint32 last_index = (uint32)g_nav_path_state.sample_count - 1U;
            index = (step_index >= last_index) ? 0U : (last_index - step_index);
        }
        else
        {
            index = step_index;
        }

        ref_index = (uint16)index;
        if (NavPath_ProjectReplayRef(x_m,
                                     y_m,
                                     yaw_deg,
                                     ref_index,
                                     &ref_index,
                                     &projected_x_m,
                                     &projected_y_m) != 0U)
        {
            NavPath_SetRefFromIndex(ref_index);
            g_nav_path_state.ref_x_m = projected_x_m;
            g_nav_path_state.ref_y_m = projected_y_m;
            NavPath_SyncReplayProgressToIndex(ref_index,
                                              &path_distance_m,
                                              &distance_complete);
        }
        else
        {
            if (g_nav_path_replay_project_last_valid != 0U)
            {
                if (g_nav_path_replay_fixed_ins != 0U)
                {
                    uint16 last_index = g_nav_path_replay_project_last_index;
                    uint16 max_jump =
                        NAV_PATH_REPLAY_PROJECT_FIXED_INS_MAX_FRAME_JUMP;

                    if (last_index >= g_nav_path_state.sample_count)
                    {
                        last_index = (uint16)(g_nav_path_state.sample_count - 1U);
                    }

                    if (ref_index > last_index)
                    {
                        uint16 delta = (uint16)(ref_index - last_index);
                        if (delta > max_jump)
                        {
                            ref_index = (uint16)(last_index + max_jump);
                        }
                    }
                    else
                    {
                        uint16 delta = (uint16)(last_index - ref_index);
                        if (delta > max_jump)
                        {
                            ref_index = (uint16)(last_index - max_jump);
                        }
                    }
                }
                else
                {
                    ref_index = g_nav_path_replay_project_last_index;
                }
                distance_complete = 0U;
                NavPath_SyncReplayProgressToIndex(ref_index,
                                                  &path_distance_m,
                                                  &distance_complete);
            }
            NavPath_SetRefFromIndex(ref_index);
            g_nav_path_replay_project_last_valid = 1U;
            g_nav_path_replay_project_last_index = ref_index;
            g_nav_path_replay_project_last_yaw_deg = g_nav_path_state.ref_yaw_deg;
        }

        g_nav_path_state.replay_index = ref_index;
        g_nav_path_state.lateral_error_m = NavPath_LateralError(x_m, y_m);
        NavPath_ApplyYawKappa(ref_index, speed_mps);
        NavPath_ApplyAheadKappa(ref_index, speed_mps);
        NavPath_ApplyAheadMotion(ref_index);
        NavPath_ApplyPreviewYaw(x_m, y_m, ref_index, speed_mps);
        g_nav_path_state.yaw_error_deg = NavPath_Wrap180(yaw_deg - g_nav_path_state.ref_yaw_deg);
        g_nav_path_state.finished = NavPath_ReplayFinishReady(path_distance_m,
                                                              x_m,
                                                              y_m,
                                                              ref_index,
                                                              distance_complete,
                                                              speed_mps);
        g_nav_path_state.valid = 1U;
    }
}

uint8 nav_path_save_to_sd(const char *filename)
{
    FIL file;
    FRESULT fr;
    UINT written;
    char line[96];
    uint16 i;
    const char *path = NavPath_FileOrDefault(filename);

    if (g_nav_path_state.sample_count == 0U)
    {
        g_nav_path_state.last_sd_result = 1U;
        return 1U;
    }

    if (NavPath_EnsureFs(NAV_PATH_FS_ALLOW_MOUNT) != 0U)
    {
        g_nav_path_state.last_sd_result = 2U;
        return 2U;
    }

    if (NavPath_EnsureDirForPath(path) != 0U)
    {
        g_nav_path_state.last_sd_result = 3U;
        return 3U;
    }

    fr = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        printf("[NAV_PATH] f_open write failed, fr=%d file=%s\r\n", (int)fr, path);
        g_nav_path_state.last_sd_result = 4U;
        return 4U;
    }

    sprintf(line, "%s,%.4f,%u\r\n", NAV_PATH_HEADER,
            (double)g_nav_path_state.sample_interval_m,
            (unsigned)g_nav_path_state.sample_count);
    fr = f_write(&file, line, (UINT)strlen(line), &written);
    if ((fr != FR_OK) || (written != (UINT)strlen(line)))
    {
        (void)f_close(&file);
        g_nav_path_state.last_sd_result = 5U;
        return 5U;
    }

    for (i = 0U; i < g_nav_path_state.sample_count; i++)
    {
        sprintf(line, "%u,%d,%d,%d,%d,%d\r\n",
                (unsigned)i,
                (int)g_nav_path_x_cm[i],
                (int)g_nav_path_y_cm[i],
                (int)g_nav_path_yaw_cdeg[i],
                (int)g_nav_path_kappa_milli[i],
                (int)g_nav_path_speed_cmps[i]);
        fr = f_write(&file, line, (UINT)strlen(line), &written);
        if ((fr != FR_OK) || (written != (UINT)strlen(line)))
        {
            (void)f_close(&file);
            g_nav_path_state.last_sd_result = 6U;
            return 6U;
        }
    }

    (void)f_sync(&file);
    (void)f_close(&file);
    printf("[NAV_PATH] saved: %s samples=%u interval=%.3f\r\n",
           path,
           (unsigned)g_nav_path_state.sample_count,
           (double)g_nav_path_state.sample_interval_m);
    g_nav_path_state.last_sd_result = 0U;
    return 0U;
}

uint8 nav_path_request_save_to_sd(uint8 task_id,
                                  const char *filename,
                                  uint8 flash_result,
                                  uint8 flash_load_result,
                                  uint8 ref_ready,
                                  uint8 ins_valid,
                                  const nav_ins_state_t *ins_state)
{
    const char *path = NavPath_FileOrDefault(filename);

    if (g_nav_path_state.sample_count == 0U)
    {
        g_nav_path_state.last_sd_result = 1U;
        return 1U;
    }

    if ((g_nav_path_sd_save_request != 0U) ||
        (g_nav_path_sd_save_active != 0U))
    {
        return 2U;
    }

    g_nav_path_sd_save_task_id = task_id;
    g_nav_path_sd_save_flash_result = flash_result;
    g_nav_path_sd_save_flash_load_result = flash_load_result;
    g_nav_path_sd_save_ref_ready = ref_ready;
    g_nav_path_sd_save_ins_valid = ins_valid;
    if (ins_state != NULL)
    {
        g_nav_path_sd_save_ins_state = *ins_state;
    }
    else
    {
        memset(&g_nav_path_sd_save_ins_state, 0, sizeof(g_nav_path_sd_save_ins_state));
    }
    strncpy(g_nav_path_sd_save_filename, path, sizeof(g_nav_path_sd_save_filename) - 1U);
    g_nav_path_sd_save_filename[sizeof(g_nav_path_sd_save_filename) - 1U] = '\0';
    g_nav_path_state.last_sd_result = NAV_PATH_SAVE_PENDING;
    NavPath_MemorySync();
    g_nav_path_sd_save_request = 1U;
    NavPath_MemorySync();
    return 0U;
}

uint8 nav_path_sd_save_is_busy(void)
{
    return ((g_nav_path_sd_save_request != 0U) ||
            (g_nav_path_sd_save_active != 0U)) ? 1U : 0U;
}

uint8 nav_path_force_last_sample_yaw(float yaw_deg)
{
    uint16 last_index;

    if (g_nav_path_state.sample_count == 0U)
    {
        return 1U;
    }

    last_index = (uint16)(g_nav_path_state.sample_count - 1U);
    g_nav_path_yaw_cdeg[last_index] = NavPath_YawToStore(yaw_deg);
    NavPath_MemorySync();
    return 0U;
}

static void NavPath_ProcessSdSaveRequest(void)
{
    uint8 task_id;
    uint8 flash_result;
    uint8 flash_load_result;
    uint8 ref_ready;
    uint8 ins_valid;
    uint8 ret_sd;
    uint8 diag_ret;
    uint8 status_ret;
    nav_ins_state_t ins_state;
    char filename[NAV_PATH_ASYNC_FILENAME_SIZE];

    if (g_nav_path_sd_save_request == 0U)
    {
        return;
    }

    if (data_logger_is_busy() != 0U)
    {
        return;
    }

    task_id = g_nav_path_sd_save_task_id;
    flash_result = g_nav_path_sd_save_flash_result;
    flash_load_result = g_nav_path_sd_save_flash_load_result;
    ref_ready = g_nav_path_sd_save_ref_ready;
    ins_valid = g_nav_path_sd_save_ins_valid;
    ins_state = g_nav_path_sd_save_ins_state;
    strncpy(filename, g_nav_path_sd_save_filename, sizeof(filename) - 1U);
    filename[sizeof(filename) - 1U] = '\0';

    g_nav_path_sd_save_active = 1U;
    g_nav_path_sd_save_request = 0U;
    NavPath_MemorySync();

    ret_sd = nav_path_save_to_sd(filename);
    diag_ret = nav_path_append_record_diag_to_sd(task_id,
                                                 "record_diag_stop",
                                                 &ins_state);
    status_ret = nav_path_append_status_to_sd(task_id,
                                              "stop",
                                              ret_sd,
                                              flash_result,
                                              flash_load_result,
                                              ref_ready,
                                              ins_valid,
                                              data_logger_get_state(),
                                              data_logger_get_last_error(),
                                              data_logger_get_task_id(),
                                              data_logger_get_profile(),
                                              data_logger_get_target_src());

    printf("[NAV_PATH] task%u async sd stop: sd=%u flash=%u diag=%u status=%u\r\n",
           (unsigned)task_id,
           (unsigned)ret_sd,
           (unsigned)flash_result,
           (unsigned)diag_ret,
           (unsigned)status_ret);

    g_nav_path_sd_save_active = 0U;
    NavPath_MemorySync();
}

uint8 nav_path_ensure_task1_sd_dir(void)
{
    uint32 now_ms;

    if (g_nav_path_task1_dir_ready != 0U)
    {
        return 0U;
    }

    now_ms = system_getval_ms();
    if ((g_nav_path_task1_dir_try_valid != 0U) &&
        ((now_ms - g_nav_path_task1_dir_last_try_ms) < NAV_PATH_TASK1_SD_RETRY_MS))
    {
        return 3U;
    }
    g_nav_path_task1_dir_try_valid = 1U;
    g_nav_path_task1_dir_last_try_ms = now_ms;

    if (NavPath_EnsureFs(NAV_PATH_FS_ALLOW_MOUNT) != 0U)
    {
        return 1U;
    }

    if (NavPath_EnsureDir(NAV_PATH_TASK1_DIR) != 0U)
    {
        return 2U;
    }

    g_nav_path_task1_dir_ready = 1U;
    printf("[NAV_PATH] task1 dir ready: %s\r\n", NAV_PATH_TASK1_DIR);
    return 0U;
}

void nav_path_request_task1_sd_dir(void)
{
    if (g_nav_path_task1_dir_ready == 0U)
    {
        g_nav_path_task1_dir_request = 1U;
    }
}

void nav_path_worker_task(uint32 now_ms)
{
    (void)now_ms;

    if ((g_nav_path_task1_dir_request != 0U) &&
        (g_nav_path_task1_dir_ready == 0U))
    {
        if (nav_path_ensure_task1_sd_dir() == 0U)
        {
            g_nav_path_task1_dir_request = 0U;
        }
    }

    NavPath_ProcessSdSaveRequest();
}

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
                                   uint8 logger_target_src)
{
    FIL file;
    FILINFO info;
    FRESULT fr;
    UINT written;
    char line[160];
    uint8 write_header = 0U;
    const char *event_text = (event != NULL) ? event : "";
    const char *log_file = NavPath_RecordLogFileForTask(task_id);

    if (NavPath_EnsureFs(NAV_PATH_FS_NO_MOUNT) != 0U)
    {
        return 1U;
    }

    if (NavPath_EnsureDirForPath(log_file) != 0U)
    {
        return 2U;
    }

    fr = f_stat(log_file, &info);
    if (fr == FR_NO_FILE)
    {
        write_header = 1U;
    }
    else if (fr != FR_OK)
    {
        return 3U;
    }

    fr = f_open(&file, log_file, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        return 4U;
    }

    if (write_header != 0U)
    {
        const char *header =
            "time_ms,task,event,samples,valid,active,mode,progress_m,interval_m,"
            "sd_ret,flash_ret,flash_load_ret,ref_ready,ins_valid,"
            "logger_state,logger_error,logger_task,logger_profile,logger_src\r\n";
        fr = f_write(&file, header, (UINT)strlen(header), &written);
        if ((fr != FR_OK) || (written != (UINT)strlen(header)))
        {
            (void)f_close(&file);
            return 5U;
        }
    }

    sprintf(line,
            "%lu,%u,%s,%u,%u,%u,%u,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
            (unsigned long)system_getval_ms(),
            (unsigned)task_id,
            event_text,
            (unsigned)g_nav_path_state.sample_count,
            (unsigned)g_nav_path_state.valid,
            (unsigned)g_nav_path_state.active,
            (unsigned)g_nav_path_state.mode,
            (double)g_nav_path_state.progress_m,
            (double)g_nav_path_state.sample_interval_m,
            (unsigned)sd_result,
            (unsigned)flash_result,
            (unsigned)flash_load_result,
            (unsigned)ref_ready,
            (unsigned)ins_valid,
            (unsigned)logger_state,
            (unsigned)logger_error,
            (unsigned)logger_task,
            (unsigned)logger_profile,
            (unsigned)logger_target_src);

    fr = f_write(&file, line, (UINT)strlen(line), &written);
    if ((fr != FR_OK) || (written != (UINT)strlen(line)))
    {
        (void)f_close(&file);
        return 6U;
    }

    (void)f_sync(&file);
    (void)f_close(&file);
    return 0U;
}

uint8 nav_path_append_record_diag_to_sd(uint8 task_id,
                                        const char *event,
                                        const nav_ins_state_t *ins_state)
{
    FIL file;
    FILINFO info;
    FRESULT fr;
    UINT written;
    char line[256];
    uint8 write_header = 0U;
    const char *event_text = (event != NULL) ? event : "";
    const char *log_file = NavPath_RecordDiagFileForTask(task_id);
    float start_x_m = 0.0f;
    float start_y_m = 0.0f;
    float start_yaw_deg = 0.0f;
    float start_speed_mps = 0.0f;
    float end_x_m = 0.0f;
    float end_y_m = 0.0f;
    float end_yaw_deg = 0.0f;
    float end_speed_mps = 0.0f;
    float start_end_dist_m = 0.0f;
    uint8 have_start = 0U;
    uint8 have_end = 0U;

    if (NavPath_EnsureFs(NAV_PATH_FS_NO_MOUNT) != 0U)
    {
        return 1U;
    }

    if (NavPath_EnsureDirForPath(log_file) != 0U)
    {
        return 2U;
    }

    fr = f_stat(log_file, &info);
    if (fr == FR_NO_FILE)
    {
        write_header = 1U;
    }
    else if (fr != FR_OK)
    {
        return 3U;
    }

    fr = f_open(&file, log_file, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        return 4U;
    }

    if (write_header != 0U)
    {
        const char *header =
            "time_ms,task,event,samples,progress_m,interval_m,"
            "path_start_x_m,path_start_y_m,path_start_yaw_deg,path_start_speed_mps,"
            "path_end_x_m,path_end_y_m,path_end_yaw_deg,path_end_speed_mps,"
            "path_start_end_dist_m,ins_x_m,ins_y_m,ins_yaw_deg,ins_distance_m\r\n";
        fr = f_write(&file, header, (UINT)strlen(header), &written);
        if ((fr != FR_OK) || (written != (UINT)strlen(header)))
        {
            (void)f_close(&file);
            return 5U;
        }
    }

    if (g_nav_path_state.sample_count != 0U)
    {
        NavPath_GetReplayPoint(0U, &start_x_m, &start_y_m);
        have_start = 1U;
        start_yaw_deg = NavPath_YawFromStore(g_nav_path_yaw_cdeg[0]);
        start_speed_mps = NavPath_SpeedFromStore(g_nav_path_speed_cmps[0]);
        NavPath_GetReplayPoint((uint16)(g_nav_path_state.sample_count - 1U),
                               &end_x_m,
                               &end_y_m);
        have_end = 1U;
        end_yaw_deg =
            NavPath_YawFromStore(g_nav_path_yaw_cdeg[g_nav_path_state.sample_count - 1U]);
        end_speed_mps =
            NavPath_SpeedFromStore(g_nav_path_speed_cmps[g_nav_path_state.sample_count - 1U]);
    }

    if ((have_start != 0U) && (have_end != 0U))
    {
        float dx_m = end_x_m - start_x_m;
        float dy_m = end_y_m - start_y_m;
        start_end_dist_m = sqrtf(dx_m * dx_m + dy_m * dy_m);
    }

    sprintf(line,
            "%lu,%u,%s,%u,%.3f,%.3f,"
            "%.3f,%.3f,%.2f,%.3f,"
            "%.3f,%.3f,%.2f,%.3f,"
            "%.3f,%.3f,%.3f,%.2f,%.3f\r\n",
            (unsigned long)system_getval_ms(),
            (unsigned)task_id,
            event_text,
            (unsigned)g_nav_path_state.sample_count,
            (double)g_nav_path_state.progress_m,
            (double)g_nav_path_state.sample_interval_m,
            (double)start_x_m,
            (double)start_y_m,
            (double)start_yaw_deg,
            (double)start_speed_mps,
            (double)end_x_m,
            (double)end_y_m,
            (double)end_yaw_deg,
            (double)end_speed_mps,
            (double)start_end_dist_m,
            (double)((ins_state != NULL) ? ins_state->x_m : 0.0f),
            (double)((ins_state != NULL) ? ins_state->y_m : 0.0f),
            (double)((ins_state != NULL) ? ins_state->yaw_deg : 0.0f),
            (double)((ins_state != NULL) ? ins_state->distance_m : 0.0f));

    fr = f_write(&file, line, (UINT)strlen(line), &written);
    if ((fr != FR_OK) || (written != (UINT)strlen(line)))
    {
        (void)f_close(&file);
        return 6U;
    }

    (void)f_sync(&file);
    (void)f_close(&file);
    return 0U;
}

uint8 nav_path_load_from_sd(const char *filename)
{
    FIL file;
    FRESULT fr;
    char line[128];
    char tag[16];
    float interval_m;
    unsigned expected_count;
    unsigned index;
    int x_cm;
    int y_cm;
    int yaw_cdeg;
    int kappa_milli;
    int speed_cmps;
    uint16 loaded = 0U;
    const char *path = NavPath_FileOrDefault(filename);

    if (nav_path_sd_save_is_busy() != 0U)
    {
        return 9U;
    }

    if (NavPath_EnsureFs(NAV_PATH_FS_NO_MOUNT) != 0U)
    {
        return 1U;
    }

    fr = f_open(&file, path, FA_READ);
    if (fr != FR_OK)
    {
        printf("[NAV_PATH] f_open read failed, fr=%d file=%s\r\n", (int)fr, path);
        return 2U;
    }

    if (NavPath_ReadLine(&file, line, (UINT)sizeof(line)) == 0U)
    {
        (void)f_close(&file);
        return 3U;
    }

    if (sscanf(line, "%15[^,],%f,%u", tag, &interval_m, &expected_count) != 3)
    {
        (void)f_close(&file);
        return 4U;
    }
    if ((strcmp(tag, NAV_PATH_HEADER) != 0) ||
        (interval_m <= 0.0f) ||
        (expected_count == 0U) ||
        (expected_count > NAV_PATH_MAX_SAMPLES))
    {
        (void)f_close(&file);
        return 5U;
    }

    nav_path_clear();
    g_nav_path_state.sample_interval_m = interval_m;

    while ((NavPath_ReadLine(&file, line, (UINT)sizeof(line)) != 0U) &&
           (loaded < NAV_PATH_MAX_SAMPLES))
    {
        if (sscanf(line, "%u,%d,%d,%d,%d,%d",
                   &index,
                   &x_cm,
                   &y_cm,
                   &yaw_cdeg,
                   &kappa_milli,
                   &speed_cmps) == 6)
        {
            if (index == (unsigned)loaded)
            {
                g_nav_path_x_cm[loaded] = NavPath_ClampInt16((int32)x_cm);
                g_nav_path_y_cm[loaded] = NavPath_ClampInt16((int32)y_cm);
                g_nav_path_yaw_cdeg[loaded] = NavPath_ClampInt16((int32)yaw_cdeg);
                g_nav_path_kappa_milli[loaded] = NavPath_ClampInt16((int32)kappa_milli);
                g_nav_path_speed_cmps[loaded] = NavPath_ClampInt16((int32)speed_cmps);
                loaded++;
            }
        }
    }

    (void)f_close(&file);

    if (loaded == 0U)
    {
        nav_path_clear();
        return 6U;
    }

    g_nav_path_state.sample_count = loaded;
    g_nav_path_state.valid = 1U;
    NavPath_SetRefFromIndex(0U);
    printf("[NAV_PATH] loaded: %s samples=%u interval=%.3f\r\n",
           path,
           (unsigned)loaded,
           (double)g_nav_path_state.sample_interval_m);
    return 0U;
}

uint8 nav_path_save_to_flash(void)
{
    return nav_path_save_to_flash_task(3U);
}

uint8 nav_path_save_to_flash_task(uint8 task_id)
{
    uint16 page;
    uint16 page_count = NavPath_FlashPageCountForTask(task_id);
    uint16 max_samples = NavPath_FlashMaxSamplesForTask(task_id);
    uint32 word_count;
    uint32 word_index = 0U;
    uint16 write_index;
    uint32 page_base = NavPath_FlashBaseForTask(task_id);

    if (g_nav_path_state.sample_count == 0U)
    {
        g_nav_path_state.last_flash_result = 1U;
        return 1U;
    }

    word_count = (uint32)g_nav_path_state.sample_count * NAV_PATH_FLASH_WORDS_PER_SAMPLE;
    if ((g_nav_path_state.sample_count > max_samples) ||
        ((word_count + NAV_PATH_FLASH_DATA_OFFSET) >
         ((uint32)EEPROM_PAGE_LENGTH * page_count)))
    {
        g_nav_path_state.last_flash_result = 2U;
        return 2U;
    }

    for (page = 0U; page < page_count; page++)
    {
        NavPath_ClearFlashBuffer();

        if (page == 0U)
        {
            flash_union_buffer[0].uint32_type = NAV_PATH_FLASH_MAGIC;
            flash_union_buffer[1].uint32_type = NAV_PATH_FLASH_VERSION;
            flash_union_buffer[2].uint32_type = (uint32)g_nav_path_state.sample_count;
            flash_union_buffer[3].float_type = g_nav_path_state.sample_interval_m;
            write_index = NAV_PATH_FLASH_DATA_OFFSET;
        }
        else
        {
            write_index = 0U;
        }

        while ((write_index < EEPROM_PAGE_LENGTH) && (word_index < word_count))
        {
            uint16 sample_index = (uint16)(word_index / NAV_PATH_FLASH_WORDS_PER_SAMPLE);
            uint8 word_offset = (uint8)(word_index % NAV_PATH_FLASH_WORDS_PER_SAMPLE);
            flash_union_buffer[write_index].uint32_type = NavPath_PackSampleWord(sample_index, word_offset);
            word_index++;
            write_index++;
        }

        if (flash_check(NAV_PATH_FLASH_SECTION, page_base + page))
        {
            flash_erase_page(NAV_PATH_FLASH_SECTION, page_base + page);
        }
        (void)flash_write_page_from_buffer(NAV_PATH_FLASH_SECTION, page_base + page);
    }

    g_nav_path_state.last_flash_result = 0U;
    printf("[NAV_PATH] flash saved: task=%u page=%u pages=%u count=%u interval=%.3f\r\n",
           (unsigned)task_id,
           (unsigned)page_base,
           (unsigned)page_count,
           (unsigned)g_nav_path_state.sample_count,
           (double)g_nav_path_state.sample_interval_m);
    return 0U;
}

uint8 nav_path_load_from_flash(void)
{
    return nav_path_load_from_flash_task(3U);
}

uint8 nav_path_load_from_flash_task(uint8 task_id)
{
    uint32 sample_count;
    uint32 word_count;
    uint32 word_index = 0U;
    float interval_m;
    uint16 page;
    uint16 read_index;
    uint16 page_count = NavPath_FlashPageCountForTask(task_id);
    uint16 max_samples = NavPath_FlashMaxSamplesForTask(task_id);
    uint32 page_base = NavPath_FlashBaseForTask(task_id);

    if (nav_path_sd_save_is_busy() != 0U)
    {
        g_nav_path_state.last_flash_load_result = 9U;
        return 9U;
    }

    if (0U == flash_check(NAV_PATH_FLASH_SECTION, page_base))
    {
        g_nav_path_state.last_flash_load_result = 1U;
        return 1U;
    }

    flash_read_page_to_buffer(NAV_PATH_FLASH_SECTION, page_base);
    if ((flash_union_buffer[0].uint32_type != NAV_PATH_FLASH_MAGIC) ||
        (flash_union_buffer[1].uint32_type != NAV_PATH_FLASH_VERSION))
    {
        g_nav_path_state.last_flash_load_result = 2U;
        return 2U;
    }

    sample_count = flash_union_buffer[2].uint32_type;
    interval_m = flash_union_buffer[3].float_type;
    if ((sample_count == 0U) ||
        (sample_count > (uint32)max_samples) ||
        (interval_m <= 0.0f))
    {
        g_nav_path_state.last_flash_load_result = 3U;
        return 3U;
    }

    word_count = sample_count * NAV_PATH_FLASH_WORDS_PER_SAMPLE;
    if ((word_count + NAV_PATH_FLASH_DATA_OFFSET) >
        ((uint32)EEPROM_PAGE_LENGTH * page_count))
    {
        g_nav_path_state.last_flash_load_result = 4U;
        return 4U;
    }

    nav_path_clear();
    g_nav_path_state.sample_interval_m = interval_m;

    for (page = 0U; page < page_count; page++)
    {
        flash_read_page_to_buffer(NAV_PATH_FLASH_SECTION, page_base + page);
        read_index = (page == 0U) ? NAV_PATH_FLASH_DATA_OFFSET : 0U;

        while ((read_index < EEPROM_PAGE_LENGTH) && (word_index < word_count))
        {
            uint16 sample_index = (uint16)(word_index / NAV_PATH_FLASH_WORDS_PER_SAMPLE);
            uint8 word_offset = (uint8)(word_index % NAV_PATH_FLASH_WORDS_PER_SAMPLE);
            NavPath_UnpackSampleWord(sample_index, word_offset, flash_union_buffer[read_index].uint32_type);
            word_index++;
            read_index++;
        }
    }

    g_nav_path_state.sample_count = (uint16)sample_count;
    g_nav_path_state.valid = (sample_count > 0U) ? 1U : 0U;
    if (sample_count > 0U)
    {
        NavPath_SetRefFromIndex(0U);
    }

    printf("[NAV_PATH] flash loaded: task=%u page=%u pages=%u count=%u interval=%.3f\r\n",
           (unsigned)task_id,
           (unsigned)page_base,
           (unsigned)page_count,
           (unsigned)g_nav_path_state.sample_count,
           (double)g_nav_path_state.sample_interval_m);
    g_nav_path_state.last_flash_load_result = (sample_count > 0U) ? 0U : 5U;
    return g_nav_path_state.last_flash_load_result;
}

void nav_path_get_state(nav_path_state_t *out_state)
{
    if (out_state != NULL)
    {
        *out_state = g_nav_path_state;
    }
}

uint16 nav_path_get_sample_count(void)
{
    return g_nav_path_state.sample_count;
}

float nav_path_get_sample_interval_m(void)
{
    return g_nav_path_state.sample_interval_m;
}

uint8 nav_path_get_replay_local_position(float x_m,
                                         float y_m,
                                         float *local_x_m,
                                         float *local_y_m)
{
    if ((local_x_m == NULL) ||
        (local_y_m == NULL) ||
        (g_nav_path_state.mode != NAV_PATH_MODE_REPLAY) ||
        (g_nav_path_base_valid == 0U))
    {
        return 1U;
    }

    *local_x_m = x_m - g_nav_path_base_x_m;
    *local_y_m = y_m - g_nav_path_base_y_m;
    return 0U;
}

uint8 nav_path_get_replay_point(uint16 index,
                                float *x_m,
                                float *y_m)
{
    if ((x_m == NULL) ||
        (y_m == NULL) ||
        (index >= g_nav_path_state.sample_count))
    {
        return 1U;
    }

    NavPath_GetReplayPoint(index, x_m, y_m);
    return 0U;
}

uint8 nav_path_get_yaw_deg(uint16 index, float *yaw_deg)
{
    if ((yaw_deg == NULL) || (index >= g_nav_path_state.sample_count))
    {
        return 1U;
    }

    *yaw_deg = NavPath_YawFromStore(g_nav_path_yaw_cdeg[index]);
    return 0U;
}

uint8 nav_path_get_sample(uint16 index,
                          float *x_m,
                          float *y_m,
                          float *yaw_deg,
                          float *kappa_1pm,
                          float *speed_mps)
{
    if (index >= g_nav_path_state.sample_count)
    {
        return 1U;
    }

    if (x_m != NULL)
    {
        *x_m = NavPath_XFromStore(g_nav_path_x_cm[index]);
    }
    if (y_m != NULL)
    {
        *y_m = NavPath_YFromStore(g_nav_path_y_cm[index]);
    }
    if (yaw_deg != NULL)
    {
        *yaw_deg = NavPath_YawFromStore(g_nav_path_yaw_cdeg[index]);
    }
    if (kappa_1pm != NULL)
    {
        *kappa_1pm = NavPath_KappaFromStore(g_nav_path_kappa_milli[index]);
    }
    if (speed_mps != NULL)
    {
        *speed_mps = NavPath_SpeedFromStore(g_nav_path_speed_cmps[index]);
    }
    return 0U;
}
