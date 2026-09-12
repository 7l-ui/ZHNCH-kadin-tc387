#ifndef _BLUE_TARGET_H_
#define _BLUE_TARGET_H_

#include "zf_common_typedef.h"

/* Compile-time rear marker color selection. */
#define BLUE_TARGET_COLOR_BLUE                   (0U)
#define BLUE_TARGET_COLOR_YELLOW                 (1U)
#define BLUE_TARGET_COLOR_MODE                   BLUE_TARGET_COLOR_YELLOW

/* Approximate lens FOV. Pixel offset remains the primary steering signal. */
#define BLUE_TARGET_HORIZONTAL_FOV_DEG_X10       (1200)

/* Vehicle-forward optical center, calibrated from centered T3C022-024 samples. */
#define BLUE_TARGET_CENTER_X_PX                   (80)

/* T3C057-060 fit: distance_cm = 6911 / bbox_height_px - 26. */
#define BLUE_TARGET_DISTANCE_K_CM_PX              (6911U)
#define BLUE_TARGET_DISTANCE_OFFSET_CM            (26U)

typedef struct
{
    uint8 valid;
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
} blue_target_result_t;

void blue_target_reset(void);
void blue_target_process(const uint16 *rgb565_image);
void blue_target_get_result(blue_target_result_t *result);

uint8 blue_target_camera_start(void);
uint8 blue_target_camera_retry(void);
uint8 blue_target_camera_is_ready(void);
void blue_target_camera_worker_enable(uint8 enable);
void blue_target_camera_worker_task(void);
uint32 blue_target_camera_get_frame_count(void);
void blue_target_camera_get_health(uint32 *worker_heartbeat,
                                   uint32 *worker_last_task_ms,
                                   uint32 *last_frame_ms,
                                   uint8 *worker_enabled,
                                   uint8 *reset_pending);
void blue_target_camera_get_timing(uint32 *preview_copy_us,
                                   uint32 *worker_total_us);

/* A successful acquire keeps the stable preview locked until release. */
uint8 blue_target_preview_acquire(const uint16 **image,
                                  blue_target_result_t *result,
                                  uint32 *frame_count);
void blue_target_preview_release(void);

#endif
