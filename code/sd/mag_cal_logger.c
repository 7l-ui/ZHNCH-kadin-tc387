#include "mag_cal_logger.h"

#include "ff.h"
#include "sd_simple.h"
#include "dici.h"
#include "stdio.h"
#include "string.h"

#define MAG_CAL_LOGGER_SAMPLE_PERIOD_MS  (20U)
#define MAG_CAL_LOGGER_SYNC_PERIOD_MS    (250U)
#define MAG_CAL_LOGGER_START_RETRY_MS    (1000U)
#define MAG_CAL_LOGGER_BUFFER_SIZE       (2048U)
#define MAG_CAL_LOGGER_LINE_SIZE         (256U)
#define MAG_CAL_LOGGER_QUEUE_SIZE        (128U)
#define MAG_CAL_LOGGER_MAX_FILES         (1000U)

#define MAG_CAL_LOGGER_STATE_IDLE        (0U)
#define MAG_CAL_LOGGER_STATE_STARTING    (1U)
#define MAG_CAL_LOGGER_STATE_RUNNING     (2U)
#define MAG_CAL_LOGGER_STATE_STOPPING    (3U)
#define MAG_CAL_LOGGER_STATE_FAILED      (4U)

typedef struct
{
    uint32 time_ms;
    float mag_x_raw;
    float mag_y_raw;
    float mag_z_raw;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float imu_pitch_deg;
    float imu_roll_deg;
    float imu_yaw_deg;
    float imu_rel_pitch_deg;
    float imu_rel_roll_deg;
    float imu_rel_yaw_deg;
    uint8 imu_ref_ready;
} mag_cal_logger_sample_t;

static const char *MAG_CAL_LOGGER_HEADER =
    "time_ms,mag_x_raw,mag_y_raw,mag_z_raw,acc_x_g,acc_y_g,acc_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,"
    "imu_pitch_deg,imu_roll_deg,imu_yaw_deg,imu_rel_pitch_deg,imu_rel_roll_deg,imu_rel_yaw_deg,imu_ref_ready\r\n";

static volatile mag_cal_logger_sample_t g_mag_cal_queue[MAG_CAL_LOGGER_QUEUE_SIZE];
static volatile uint16 g_mag_cal_queue_head;
static volatile uint16 g_mag_cal_queue_tail;
static volatile uint32 g_mag_cal_dropped_count;
static volatile uint32 g_mag_cal_last_sample_ms;
static volatile uint32 g_mag_cal_start_ms;
static volatile uint32 g_mag_cal_auto_stop_ms;
static volatile uint32 g_mag_cal_next_retry_ms;
static volatile uint8 g_mag_cal_state;
static volatile uint8 g_mag_cal_last_error;
static char g_mag_cal_filename[64];

#pragma section all "cpu1_dsram"
static FATFS g_mag_cal_fatfs;
static FIL g_mag_cal_file;
static uint8 g_mag_cal_mounted;
static uint8 g_mag_cal_file_open;
static uint32 g_mag_cal_last_sync_ms;
static char g_mag_cal_buffer[MAG_CAL_LOGGER_BUFFER_SIZE];
static char g_mag_cal_line[MAG_CAL_LOGGER_LINE_SIZE];
static uint32 g_mag_cal_buffer_used;
#pragma section all restore

static void mag_cal_logger_memory_sync(void)
{
    __dsync();
}

static uint16 mag_cal_logger_next_index(uint16 index)
{
    index++;
    if(index >= MAG_CAL_LOGGER_QUEUE_SIZE)
    {
        index = 0U;
    }
    return index;
}

static uint8 mag_cal_logger_time_before(uint32 a, uint32 b)
{
    return (((uint32)(a - b)) > 0x80000000UL) ? 1U : 0U;
}

static void mag_cal_logger_queue_reset(void)
{
    g_mag_cal_queue_head = 0U;
    g_mag_cal_queue_tail = 0U;
    g_mag_cal_dropped_count = 0U;
    g_mag_cal_last_sample_ms = 0U;
    g_mag_cal_start_ms = 0U;
    g_mag_cal_next_retry_ms = 0U;
    mag_cal_logger_memory_sync();
}

static uint8 mag_cal_logger_queue_push(const mag_cal_logger_sample_t *sample)
{
    uint16 head;
    uint16 next;

    if(sample == NULL)
    {
        return 1U;
    }

    head = g_mag_cal_queue_head;
    next = mag_cal_logger_next_index(head);
    if(next == g_mag_cal_queue_tail)
    {
        g_mag_cal_dropped_count++;
        return 2U;
    }

    g_mag_cal_queue[head] = *sample;
    mag_cal_logger_memory_sync();
    g_mag_cal_queue_head = next;
    return 0U;
}

static uint8 mag_cal_logger_queue_pop(mag_cal_logger_sample_t *sample)
{
    uint16 tail;

    if(sample == NULL)
    {
        return 1U;
    }

    tail = g_mag_cal_queue_tail;
    if(tail == g_mag_cal_queue_head)
    {
        return 1U;
    }

    *sample = g_mag_cal_queue[tail];
    mag_cal_logger_memory_sync();
    g_mag_cal_queue_tail = mag_cal_logger_next_index(tail);
    return 0U;
}

static uint8 mag_cal_logger_flush(void)
{
    FRESULT fr;
    UINT written;

    if((g_mag_cal_file_open == 0U) || (g_mag_cal_buffer_used == 0U))
    {
        return 0U;
    }

    fr = f_write(&g_mag_cal_file, g_mag_cal_buffer, (UINT)g_mag_cal_buffer_used, &written);
    if((fr != FR_OK) || (written != (UINT)g_mag_cal_buffer_used))
    {
        return 1U;
    }

    g_mag_cal_buffer_used = 0U;
    return 0U;
}

static uint8 mag_cal_logger_append(const char *text, uint32 len)
{
    if((text == NULL) || (len == 0U))
    {
        return 0U;
    }

    if(len > MAG_CAL_LOGGER_BUFFER_SIZE)
    {
        return 1U;
    }

    if((g_mag_cal_buffer_used + len) > MAG_CAL_LOGGER_BUFFER_SIZE)
    {
        if(mag_cal_logger_flush() != 0U)
        {
            return 1U;
        }
    }

    memcpy(&g_mag_cal_buffer[g_mag_cal_buffer_used], text, (size_t)len);
    g_mag_cal_buffer_used += len;
    return 0U;
}

static uint8 mag_cal_logger_make_filename(void)
{
    FILINFO info;
    FRESULT fr;
    uint16 i;

    for(i = 0U; i < MAG_CAL_LOGGER_MAX_FILES; i++)
    {
        sprintf(g_mag_cal_filename, "0:/CAL/MAG/MAG%03u.CSV", (unsigned)i);
        fr = f_stat(g_mag_cal_filename, &info);
        if(fr == FR_NO_FILE)
        {
            mag_cal_logger_memory_sync();
            return 0U;
        }
        if(fr != FR_OK)
        {
            return 1U;
        }
    }

    return 2U;
}

static uint8 mag_cal_logger_backend_init(void)
{
    FRESULT fr;
    uint8 ret;

    if(g_mag_cal_mounted != 0U)
    {
        return 0U;
    }

    ret = sd_simple_init();
    if(ret != 0U)
    {
        printf("[MAG_CAL] sd_simple_init failed, ret=%u\r\n", (unsigned)ret);
        return 1U;
    }

    fr = f_mount(&g_mag_cal_fatfs, "0:", 1);
    if(fr != FR_OK)
    {
        printf("[MAG_CAL] f_mount failed, fr=%d\r\n", (int)fr);
        return 2U;
    }
    g_mag_cal_mounted = 1U;

    fr = f_mkdir("0:/CAL");
    if((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[MAG_CAL] f_mkdir failed, fr=%d dir=0:/CAL\r\n", (int)fr);
        return 3U;
    }

    fr = f_mkdir("0:/CAL/MAG");
    if((fr != FR_OK) && (fr != FR_EXIST))
    {
        printf("[MAG_CAL] f_mkdir failed, fr=%d dir=0:/CAL/MAG\r\n", (int)fr);
        return 4U;
    }

    g_mag_cal_buffer_used = 0U;
    g_mag_cal_last_sync_ms = 0U;
    return 0U;
}

static void mag_cal_logger_backend_close(void)
{
    if(g_mag_cal_file_open != 0U)
    {
        (void)mag_cal_logger_flush();
        (void)f_sync(&g_mag_cal_file);
        (void)f_close(&g_mag_cal_file);
        g_mag_cal_file_open = 0U;
    }

    if(g_mag_cal_mounted != 0U)
    {
        (void)f_unmount("0:");
        g_mag_cal_mounted = 0U;
    }
}

static void mag_cal_logger_backend_fail(uint8 error)
{
    g_mag_cal_last_error = error;
    g_mag_cal_state = MAG_CAL_LOGGER_STATE_FAILED;
    mag_cal_logger_backend_close();
    mag_cal_logger_memory_sync();
}

static uint8 mag_cal_logger_backend_start(void)
{
    FRESULT fr;
    UINT written;
    uint8 ret;

    ret = mag_cal_logger_backend_init();
    if(ret != 0U)
    {
        return ret;
    }

    ret = mag_cal_logger_make_filename();
    if(ret != 0U)
    {
        return (uint8)(10U + ret);
    }

    fr = f_open(&g_mag_cal_file, g_mag_cal_filename, FA_CREATE_NEW | FA_WRITE);
    if(fr != FR_OK)
    {
        printf("[MAG_CAL] f_open failed, fr=%d file=%s\r\n", (int)fr, g_mag_cal_filename);
        return 20U;
    }

    g_mag_cal_file_open = 1U;
    fr = f_write(&g_mag_cal_file, MAG_CAL_LOGGER_HEADER, (UINT)strlen(MAG_CAL_LOGGER_HEADER), &written);
    if((fr != FR_OK) || (written != (UINT)strlen(MAG_CAL_LOGGER_HEADER)))
    {
        return 21U;
    }

    (void)f_sync(&g_mag_cal_file);
    printf("[MAG_CAL] logging to %s\r\n", g_mag_cal_filename);
    return 0U;
}

static void mag_cal_logger_capture_sample(uint32 now_ms, mag_cal_logger_sample_t *sample)
{
    if(sample == NULL)
    {
        return;
    }

    sample->time_ms = now_ms;
    sample->mag_x_raw = IMU_MagRawX;
    sample->mag_y_raw = IMU_MagRawY;
    sample->mag_z_raw = IMU_MagRawZ;
    sample->acc_x_g = imu_data.acc_x;
    sample->acc_y_g = imu_data.acc_y;
    sample->acc_z_g = imu_data.acc_z;
    sample->gyro_x_dps = imu_data.gyro_x * DEG_TO_RAD;
    sample->gyro_y_dps = imu_data.gyro_y * DEG_TO_RAD;
    sample->gyro_z_dps = imu_data.gyro_z * DEG_TO_RAD;
    sample->imu_pitch_deg = IMU_Pitch;
    sample->imu_roll_deg = IMU_Roll;
    sample->imu_yaw_deg = IMU_Yaw;
    sample->imu_rel_pitch_deg = IMU_RelPitch;
    sample->imu_rel_roll_deg = IMU_RelRoll;
    sample->imu_rel_yaw_deg = IMU_RelYaw;
    sample->imu_ref_ready = IMU_AHRS_IsReferenceReady();
}

static uint8 mag_cal_logger_format_sample(const mag_cal_logger_sample_t *sample)
{
    int len;

    if(sample == NULL)
    {
        return 1U;
    }

    len = sprintf(g_mag_cal_line,
                  "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u\r\n",
                  (unsigned long)sample->time_ms,
                  (double)sample->mag_x_raw,
                  (double)sample->mag_y_raw,
                  (double)sample->mag_z_raw,
                  (double)sample->acc_x_g,
                  (double)sample->acc_y_g,
                  (double)sample->acc_z_g,
                  (double)sample->gyro_x_dps,
                  (double)sample->gyro_y_dps,
                  (double)sample->gyro_z_dps,
                  (double)sample->imu_pitch_deg,
                  (double)sample->imu_roll_deg,
                  (double)sample->imu_yaw_deg,
                  (double)sample->imu_rel_pitch_deg,
                  (double)sample->imu_rel_roll_deg,
                  (double)sample->imu_rel_yaw_deg,
                  (unsigned)sample->imu_ref_ready);
    if(len <= 0)
    {
        return 1U;
    }

    return mag_cal_logger_append(g_mag_cal_line, (uint32)len);
}

uint8 mag_cal_logger_start(void)
{
    if((g_mag_cal_state != MAG_CAL_LOGGER_STATE_IDLE) &&
       (g_mag_cal_state != MAG_CAL_LOGGER_STATE_FAILED))
    {
        return 1U;
    }

    mag_cal_logger_queue_reset();
    g_mag_cal_last_error = 0U;
    g_mag_cal_filename[0] = '\0';
    g_mag_cal_state = MAG_CAL_LOGGER_STATE_STARTING;
    mag_cal_logger_memory_sync();
    return 0U;
}

void mag_cal_logger_set_auto_stop_ms(uint32 duration_ms)
{
    g_mag_cal_auto_stop_ms = duration_ms;
    mag_cal_logger_memory_sync();
}

void mag_cal_logger_stop(void)
{
    if((g_mag_cal_state == MAG_CAL_LOGGER_STATE_RUNNING) ||
       (g_mag_cal_state == MAG_CAL_LOGGER_STATE_STARTING))
    {
        g_mag_cal_state = MAG_CAL_LOGGER_STATE_STOPPING;
        mag_cal_logger_memory_sync();
    }
}

void mag_cal_logger_task(uint32 now_ms)
{
    mag_cal_logger_sample_t sample;

    if(g_mag_cal_state != MAG_CAL_LOGGER_STATE_RUNNING)
    {
        return;
    }

    if((g_mag_cal_last_sample_ms != 0U) &&
       ((now_ms - g_mag_cal_last_sample_ms) < MAG_CAL_LOGGER_SAMPLE_PERIOD_MS))
    {
        return;
    }

    g_mag_cal_last_sample_ms = now_ms;
    mag_cal_logger_capture_sample(now_ms, &sample);
    (void)mag_cal_logger_queue_push(&sample);
}

void mag_cal_logger_worker_task(uint32 now_ms)
{
    mag_cal_logger_sample_t sample;

    if(g_mag_cal_state == MAG_CAL_LOGGER_STATE_STARTING)
    {
        uint8 ret;

        if((g_mag_cal_next_retry_ms != 0U) &&
           (mag_cal_logger_time_before(now_ms, g_mag_cal_next_retry_ms) != 0U))
        {
            return;
        }

        ret = mag_cal_logger_backend_start();
        if(ret != 0U)
        {
            mag_cal_logger_backend_close();
            g_mag_cal_last_error = ret;
            g_mag_cal_next_retry_ms = now_ms + MAG_CAL_LOGGER_START_RETRY_MS;
            printf("[MAG_CAL] start failed ret=%u, retry in %lu ms\r\n",
                   (unsigned)ret,
                   (unsigned long)MAG_CAL_LOGGER_START_RETRY_MS);
            mag_cal_logger_memory_sync();
            return;
        }

        g_mag_cal_state = MAG_CAL_LOGGER_STATE_RUNNING;
        g_mag_cal_last_error = 0U;
        g_mag_cal_start_ms = now_ms;
        g_mag_cal_next_retry_ms = 0U;
        g_mag_cal_last_sync_ms = now_ms;
        mag_cal_logger_memory_sync();
    }

    if(g_mag_cal_state == MAG_CAL_LOGGER_STATE_RUNNING)
    {
        while(mag_cal_logger_queue_pop(&sample) == 0U)
        {
            if(mag_cal_logger_format_sample(&sample) != 0U)
            {
                mag_cal_logger_backend_fail(30U);
                return;
            }
        }

        if((g_mag_cal_buffer_used > 0U) &&
           ((now_ms - g_mag_cal_last_sync_ms) >= MAG_CAL_LOGGER_SYNC_PERIOD_MS))
        {
            if(mag_cal_logger_flush() != 0U)
            {
                mag_cal_logger_backend_fail(31U);
                return;
            }

            if(f_sync(&g_mag_cal_file) != FR_OK)
            {
                mag_cal_logger_backend_fail(32U);
                return;
            }

            g_mag_cal_last_sync_ms = now_ms;
        }

        if((g_mag_cal_auto_stop_ms > 0U) &&
           ((now_ms - g_mag_cal_start_ms) >= g_mag_cal_auto_stop_ms))
        {
            printf("[MAG_CAL] auto stop after %lu ms\r\n", (unsigned long)g_mag_cal_auto_stop_ms);
            g_mag_cal_state = MAG_CAL_LOGGER_STATE_STOPPING;
            mag_cal_logger_memory_sync();
        }
    }

    if(g_mag_cal_state == MAG_CAL_LOGGER_STATE_STOPPING)
    {
        while(mag_cal_logger_queue_pop(&sample) == 0U)
        {
            if(mag_cal_logger_format_sample(&sample) != 0U)
            {
                mag_cal_logger_backend_fail(33U);
                return;
            }
        }

        mag_cal_logger_backend_close();
        g_mag_cal_state = MAG_CAL_LOGGER_STATE_IDLE;
        mag_cal_logger_memory_sync();
    }
}

uint8 mag_cal_logger_is_running(void)
{
    return (g_mag_cal_state == MAG_CAL_LOGGER_STATE_RUNNING) ? 1U : 0U;
}

uint8 mag_cal_logger_get_state(void)
{
    return g_mag_cal_state;
}

uint8 mag_cal_logger_get_last_error(void)
{
    return g_mag_cal_last_error;
}

const char *mag_cal_logger_get_filename(void)
{
    return g_mag_cal_filename;
}
