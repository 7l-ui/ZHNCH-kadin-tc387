#include "blue_target.h"

#include "IfxCpu.h"
#include "guimai/guimai_board.h"
#include "zf_device_scc8660.h"
#include "zf_driver_timer.h"

#define BLUE_TARGET_SAMPLE_STEP             (2U)
#define BLUE_TARGET_GRID_WIDTH              (SCC8660_W / BLUE_TARGET_SAMPLE_STEP)
#define BLUE_TARGET_GRID_HEIGHT             (SCC8660_H / BLUE_TARGET_SAMPLE_STEP)
#define BLUE_TARGET_GRID_SIZE               (BLUE_TARGET_GRID_WIDTH * BLUE_TARGET_GRID_HEIGHT)
#define BLUE_TARGET_ROI_HEIGHT              (SCC8660_H)
#define BLUE_TARGET_ROI_GRID_HEIGHT         (BLUE_TARGET_ROI_HEIGHT / BLUE_TARGET_SAMPLE_STEP)

#define BLUE_TARGET_MIN_B5                  (13U)
#define BLUE_TARGET_MIN_B_OVER_R5           (4U)
#define BLUE_TARGET_MIN_B2_OVER_G6          (0U)
#define BLUE_TARGET_MAX_RGB565_SUM           (74U)
#define BLUE_TARGET_YELLOW_MIN_RG5           (8U)
#define BLUE_TARGET_YELLOW_STRONG_CHROMA5    (5U)
#define BLUE_TARGET_YELLOW_STRONG_RGB_RATIO  (3U)
#define BLUE_TARGET_YELLOW_SHADOW_CHROMA5    (7U)
#define BLUE_TARGET_YELLOW_SHADOW_RGB_RATIO  (2U)
#define BLUE_TARGET_YELLOW_HUE_SLACK5        (4U)
#define BLUE_TARGET_YELLOW_BLUE_CAST_STEP    (8U)
#define BLUE_TARGET_YELLOW_BLUE_CAST_MARGIN5 (3U)
#define BLUE_TARGET_YELLOW_BLUE_CAST_PCT     (70U)
#define BLUE_TARGET_YELLOW_DUSK_MIN_RG5      (9U)
#define BLUE_TARGET_YELLOW_DUSK_MAX_B5       (18U)
#define BLUE_TARGET_YELLOW_DUSK_MAX_RG_BALANCE5 (4U)
#define BLUE_TARGET_YELLOW_BRIGHT_MIN_RG5     (16U)
#define BLUE_TARGET_YELLOW_MIN_FILL_PCT      (40U)
#define BLUE_TARGET_YELLOW_MAX_WIDTH_TO_HEIGHT_PCT (150U)
#define BLUE_TARGET_YELLOW_CLIPPED_DISTANCE_CM (110U)
#define BLUE_TARGET_MIN_COMPONENT_SAMPLES   (20U)
#define BLUE_TARGET_MIN_WIDTH_SAMPLES       (3U)
#define BLUE_TARGET_MIN_HEIGHT_SAMPLES      (4U)
#define BLUE_TARGET_FULL_AREA_SAMPLES        (80U)
#define BLUE_TARGET_MIN_VALID_CONFIDENCE_PCT (55U)
#define BLUE_TARGET_TRACK_MIN_CONFIDENCE_PCT (35U)
#define BLUE_TARGET_TRACK_MAX_DX_PX          (16U)
#define BLUE_TARGET_TRACK_MAX_DY_PX          (16U)
#define BLUE_TARGET_TRACK_MIN_DH_PX          (8U)
#define BLUE_TARGET_TRACK_MAX_DH_PCT         (40U)
#define BLUE_TARGET_TRACK_EDGE_MARGIN_PX      (20U)
#define BLUE_TARGET_TRACK_EDGE_MAX_DX_PX      (24U)
#define BLUE_TARGET_TRACK_EDGE_MAX_DH_PCT     (60U)
#define BLUE_TARGET_TRACK_RELEASE_MS         (180U)
#define BLUE_TARGET_DISTANCE_FILTER_ALPHA_PCT (65U)
#define BLUE_TARGET_DISTANCE_JUMP_BASE_CM    (15U)
#define BLUE_TARGET_DISTANCE_JUMP_RATE_CMPS  (250U)
#define BLUE_TARGET_DISTANCE_JUMP_MAX_CM     (30U)
#define BLUE_TARGET_DISTANCE_JUMP_RECOVER_CMPS (180U)

#define BLUE_TARGET_MASK_COLOR               (1U)
#define BLUE_TARGET_MASK_VISITED             (2U)
#define BLUE_TARGET_MASK_BRIGHT              (4U)
#define BLUE_TARGET_ACQUIRE_BRIGHT_WEIGHT    (15U)
#define BLUE_TARGET_ACQUIRE_CENTER_WEIGHT    (3U)
#define BLUE_TARGET_ACQUIRE_AREA_WEIGHT      (8U)
#define BLUE_TARGET_ACQUIRE_AREA_CAP_SAMPLES (160U)
#define BLUE_TARGET_ACQUIRE_CLIPPED_PENALTY  (2500U)

#if ((SCC8660_W % BLUE_TARGET_SAMPLE_STEP) != 0) || \
    ((SCC8660_H % BLUE_TARGET_SAMPLE_STEP) != 0)
#error "blue_target requires camera dimensions divisible by the sample step"
#endif

#if ((BLUE_TARGET_ROI_HEIGHT % BLUE_TARGET_SAMPLE_STEP) != 0)
#error "blue_target ROI height must be divisible by the sample step"
#endif

#if (BLUE_TARGET_GRID_SIZE > 65535U)
#error "blue_target queue index must fit in uint16"
#endif

#if (BLUE_TARGET_COLOR_MODE != BLUE_TARGET_COLOR_BLUE) && \
    (BLUE_TARGET_COLOR_MODE != BLUE_TARGET_COLOR_YELLOW)
#error "invalid BLUE_TARGET_COLOR_MODE"
#endif

typedef struct
{
    uint16 count;
    uint32 sum_x;
    uint32 sum_y;
    uint16 min_x;
    uint16 min_y;
    uint16 max_x;
    uint16 max_y;
    uint16 bright_count;
    uint8 touches_top;
} blue_target_component_t;

typedef struct
{
    uint16 preview[SCC8660_H][SCC8660_W];
    volatile blue_target_result_t result;
    volatile blue_target_result_t preview_result;
    volatile uint32 result_sequence;
    volatile uint32 frame_count;
    volatile uint32 preview_frame_count;
    volatile uint32 preview_copy_us;
    volatile uint32 worker_total_us;
    volatile uint32 worker_heartbeat;
    volatile uint32 worker_last_task_ms;
    volatile uint32 last_frame_ms;
    IfxCpu_mutexLock preview_lock;
    volatile uint8 worker_enabled;
    volatile uint8 reset_request;
} blue_target_shared_t;

#pragma section all "cpu2_dsram"
static uint8 g_blue_target_mask[BLUE_TARGET_GRID_SIZE];
static uint16 g_blue_target_queue[BLUE_TARGET_GRID_SIZE];
static blue_target_result_t g_blue_target_work_result;
static blue_target_result_t g_blue_target_track_result;
static uint32 g_blue_target_track_last_ms;
static uint8 g_blue_target_track_locked;
static uint16 g_blue_target_distance_last_cm;
static uint32 g_blue_target_distance_last_ms;
static uint8 g_blue_target_distance_filter_valid;
static uint8 g_blue_target_distance_jump_active;
#pragma section all restore

#pragma section all "lmubss"
static blue_target_shared_t g_blue_target_shared_storage;
#pragma section all restore

static uint8 g_blue_target_camera_init_done = 0U;
static uint8 g_blue_target_camera_ready = 0U;

#define BLUE_TARGET_CAMERA_POWERUP_DELAY_MS    (1200U)

static uint8 blue_target_camera_init_once(void)
{
    uint32 now_ms = system_getval_ms();

    if (now_ms < BLUE_TARGET_CAMERA_POWERUP_DELAY_MS)
    {
        system_delay_ms(BLUE_TARGET_CAMERA_POWERUP_DELAY_MS - now_ms);
    }

    blue_target_camera_worker_enable(0U);
    g_blue_target_camera_init_done = 1U;
    g_blue_target_camera_ready = (scc8660_init_sccb() == 0U) ? 1U : 0U;
    blue_target_camera_worker_enable(g_blue_target_camera_ready);
    return (g_blue_target_camera_ready != 0U) ? 0U : 1U;
}

static blue_target_shared_t *blue_target_shared(void)
{
    uint32 address = (uint32)&g_blue_target_shared_storage;

    /* CPU0 and CPU2 must use the non-cached LMU alias for coherent sharing. */
    if ((address >= 0x90000000UL) && (address < 0xA0000000UL))
    {
        address += 0x20000000UL;
    }
    return (blue_target_shared_t *)address;
}

static void blue_target_release_camera_frame(void)
{
    scc8660_frame_release();
}

static void blue_target_publish_result(const blue_target_result_t *result)
{
    blue_target_shared_t *shared = blue_target_shared();

    shared->result_sequence++;
    __dsync();
    shared->result = *result;
    __dsync();
    shared->result_sequence++;
}

static uint8 blue_target_reset_on_worker(void)
{
    blue_target_shared_t *shared = blue_target_shared();
    blue_target_result_t empty_result;

    memset(&empty_result, 0, sizeof(empty_result));
    g_blue_target_work_result = empty_result;
    g_blue_target_track_result = empty_result;
    g_blue_target_track_last_ms = 0U;
    g_blue_target_track_locked = 0U;
    g_blue_target_distance_last_cm = 0U;
    g_blue_target_distance_last_ms = 0U;
    g_blue_target_distance_filter_valid = 0U;
    g_blue_target_distance_jump_active = 0U;
    blue_target_publish_result(&empty_result);

    if (!IfxCpu_acquireMutex(&shared->preview_lock))
    {
        return 0U;
    }

    shared->preview_result = empty_result;
    shared->preview_frame_count = 0U;
    shared->preview_copy_us = 0U;
    shared->worker_total_us = 0U;
    __dsync();
    IfxCpu_releaseMutex(&shared->preview_lock);
    return 1U;
}

static uint16 blue_target_swap_rgb565(uint16 pixel)
{
    return (uint16)((pixel << 8) | (pixel >> 8));
}

static uint8 blue_target_yellow_dusk_mode(const uint16 *rgb565_image)
{
#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    uint32 blue_cast_count = 0U;
    uint32 sample_count = 0U;
    uint16 y;
    uint16 x;

    if (rgb565_image == NULL)
    {
        return 0U;
    }
    for (y = 0U; y < BLUE_TARGET_ROI_HEIGHT;
         y += BLUE_TARGET_YELLOW_BLUE_CAST_STEP)
    {
        for (x = 0U; x < SCC8660_W; x += BLUE_TARGET_YELLOW_BLUE_CAST_STEP)
        {
            uint16 pixel = blue_target_swap_rgb565(
                rgb565_image[(uint32)y * SCC8660_W + x]);
            uint8 red5 = (uint8)((pixel >> 11) & 0x1FU);
            uint8 green5 = (uint8)((pixel >> 6) & 0x1FU);
            uint8 blue5 = (uint8)(pixel & 0x1FU);
            uint8 min_rg5 = (red5 < green5) ? red5 : green5;

            sample_count++;
            if (blue5 >= (uint8)(min_rg5 +
                                 BLUE_TARGET_YELLOW_BLUE_CAST_MARGIN5))
            {
                blue_cast_count++;
            }
        }
    }
    return ((blue_cast_count * 100U) >=
            (sample_count * BLUE_TARGET_YELLOW_BLUE_CAST_PCT)) ? 1U : 0U;
#else
    (void)rgb565_image;
    return 0U;
#endif
}

static uint8 blue_target_is_color(uint16 raw_pixel, uint8 yellow_dusk_mode)
{
    uint16 pixel = blue_target_swap_rgb565(raw_pixel);
    uint8 red5 = (uint8)((pixel >> 11) & 0x1FU);
    uint8 green6 = (uint8)((pixel >> 5) & 0x3FU);
    uint8 blue5 = (uint8)(pixel & 0x1FU);

#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    uint8 green5 = (uint8)(green6 >> 1);
    uint8 min_rg5 = (red5 < green5) ? red5 : green5;
    uint8 rg_balance5 = (red5 >= green5) ?
                       (uint8)(red5 - green5) : (uint8)(green5 - red5);
    uint8 yellow_chroma5;
    uint8 strong_yellow;
    uint8 shadow_yellow;

    if ((yellow_dusk_mode != 0U) &&
        (min_rg5 >= BLUE_TARGET_YELLOW_DUSK_MIN_RG5) &&
        (blue5 <= BLUE_TARGET_YELLOW_DUSK_MAX_B5) &&
        (min_rg5 >= blue5) &&
        (rg_balance5 <= BLUE_TARGET_YELLOW_DUSK_MAX_RG_BALANCE5))
    {
        return (uint8)(BLUE_TARGET_MASK_COLOR |
                       ((min_rg5 >= BLUE_TARGET_YELLOW_BRIGHT_MIN_RG5) ?
                        BLUE_TARGET_MASK_BRIGHT : 0U));
    }

    if ((min_rg5 < BLUE_TARGET_YELLOW_MIN_RG5) || (min_rg5 <= blue5))
    {
        return 0U;
    }
    yellow_chroma5 = (uint8)(min_rg5 - blue5);
    strong_yellow =
        ((yellow_chroma5 >= BLUE_TARGET_YELLOW_STRONG_CHROMA5) &&
         (((uint16)blue5 * BLUE_TARGET_YELLOW_STRONG_RGB_RATIO) <=
          (uint16)min_rg5)) ? 1U : 0U;
    shadow_yellow =
        ((yellow_chroma5 >= BLUE_TARGET_YELLOW_SHADOW_CHROMA5) &&
         (((uint16)blue5 * BLUE_TARGET_YELLOW_SHADOW_RGB_RATIO) <=
          (uint16)min_rg5)) ? 1U : 0U;

    /* Integer HSV-like test: saturation rejects gray and R/G balance limits hue. */
    if (((strong_yellow == 0U) && (shadow_yellow == 0U)) ||
        (((uint16)rg_balance5 * 2U) >
         ((uint16)yellow_chroma5 + BLUE_TARGET_YELLOW_HUE_SLACK5)))
    {
        return 0U;
    }
#else
    if (blue5 < BLUE_TARGET_MIN_B5)
    {
        return 0U;
    }
    if (blue5 < (uint8)(red5 + BLUE_TARGET_MIN_B_OVER_R5))
    {
        return 0U;
    }
    if (((uint16)blue5 * 2U) < ((uint16)green6 + BLUE_TARGET_MIN_B2_OVER_G6))
    {
        return 0U;
    }
    /* The board stays distinctly darker than the blue-white outdoor sky. */
    if (((uint16)red5 + (uint16)green6 + (uint16)blue5) >
        BLUE_TARGET_MAX_RGB565_SUM)
    {
        return 0U;
    }
#endif
#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    return (uint8)(BLUE_TARGET_MASK_COLOR |
                   ((min_rg5 >= BLUE_TARGET_YELLOW_BRIGHT_MIN_RG5) ?
                    BLUE_TARGET_MASK_BRIGHT : 0U));
#else
    return BLUE_TARGET_MASK_COLOR;
#endif
}

static uint8 blue_target_component_is_candidate(const blue_target_component_t *component)
{
    uint16 width;
    uint16 height;

    if ((component == NULL) ||
        (component->count < BLUE_TARGET_MIN_COMPONENT_SAMPLES) ||
        (component->touches_top != 0U))
    {
        return 0U;
    }

#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_BLUE)
    if (component->max_y >= (BLUE_TARGET_ROI_GRID_HEIGHT - 1U))
    {
        return 0U;
    }
#endif

    width = (uint16)(component->max_x - component->min_x + 1U);
    height = (uint16)(component->max_y - component->min_y + 1U);
    if ((width < BLUE_TARGET_MIN_WIDTH_SAMPLES) ||
        (height < BLUE_TARGET_MIN_HEIGHT_SAMPLES))
    {
        return 0U;
    }

#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    if (((uint32)component->count * 100U) <
        ((uint32)width * height * BLUE_TARGET_YELLOW_MIN_FILL_PCT))
    {
        return 0U;
    }
    /* The current rear marker is a 50 cm square. Glare can merge it with
     * nearby yellow pixels, so permit a modest horizontal extension only. */
    if (((uint32)width * 100U) >
        ((uint32)height * BLUE_TARGET_YELLOW_MAX_WIDTH_TO_HEIGHT_PCT))
    {
        return 0U;
    }
#else
    /* The blue rear marker remains portrait; reject horizon and sky fragments. */
    if (width > height)
    {
        return 0U;
    }
#endif
    return 1U;
}

static void blue_target_queue_neighbor(uint16 x, uint16 y, uint16 *tail)
{
    uint16 index = (uint16)(y * BLUE_TARGET_GRID_WIDTH + x);
    uint8 mask_value = g_blue_target_mask[index];

    if ((mask_value & BLUE_TARGET_MASK_COLOR) != 0U)
    {
        g_blue_target_mask[index] =
            (uint8)((mask_value & BLUE_TARGET_MASK_BRIGHT) |
                    BLUE_TARGET_MASK_VISITED);
        g_blue_target_queue[*tail] = index;
        (*tail)++;
    }
}

static void blue_target_read_component(uint16 start_index, blue_target_component_t *component)
{
    uint16 head = 0U;
    uint16 tail = 0U;

    memset(component, 0, sizeof(*component));
    component->min_x = BLUE_TARGET_GRID_WIDTH;
    component->min_y = BLUE_TARGET_GRID_HEIGHT;

    g_blue_target_mask[start_index] =
        (uint8)((g_blue_target_mask[start_index] & BLUE_TARGET_MASK_BRIGHT) |
                BLUE_TARGET_MASK_VISITED);
    g_blue_target_queue[tail++] = start_index;

    while (head < tail)
    {
        uint16 index = g_blue_target_queue[head++];
        uint16 x = (uint16)(index % BLUE_TARGET_GRID_WIDTH);
        uint16 y = (uint16)(index / BLUE_TARGET_GRID_WIDTH);

        component->count++;
        if ((g_blue_target_mask[index] & BLUE_TARGET_MASK_BRIGHT) != 0U)
        {
            component->bright_count++;
        }
        component->sum_x += x;
        component->sum_y += y;
        if (x < component->min_x) component->min_x = x;
        if (y < component->min_y) component->min_y = y;
        if (x > component->max_x) component->max_x = x;
        if (y > component->max_y) component->max_y = y;
        if (y == 0U) component->touches_top = 1U;

        if (x > 0U) blue_target_queue_neighbor((uint16)(x - 1U), y, &tail);
        if ((x + 1U) < BLUE_TARGET_GRID_WIDTH) blue_target_queue_neighbor((uint16)(x + 1U), y, &tail);
        if (y > 0U) blue_target_queue_neighbor(x, (uint16)(y - 1U), &tail);
        if ((y + 1U) < BLUE_TARGET_GRID_HEIGHT) blue_target_queue_neighbor(x, (uint16)(y + 1U), &tail);
    }
}

static void blue_target_make_result(const blue_target_component_t *component,
                                    blue_target_result_t *result)
{
    uint16 sample_width = (uint16)(component->max_x - component->min_x + 1U);
    uint16 sample_height = (uint16)(component->max_y - component->min_y + 1U);
    uint32 bbox_samples = (uint32)sample_width * sample_height;
    uint16 fill_pct = (uint16)(((uint32)component->count * 100U) / bbox_samples);
    uint16 area_score_count = component->count;
    uint16 confidence;
    int32 offset_x;
#if (BLUE_TARGET_DISTANCE_K_CM_PX > 0U)
    uint32 calibrated_distance_cm;
#endif

    if (area_score_count > BLUE_TARGET_FULL_AREA_SAMPLES)
    {
        area_score_count = BLUE_TARGET_FULL_AREA_SAMPLES;
    }
    confidence = (uint16)(((uint32)fill_pct * 60U) / 100U);
    confidence += (uint16)(((uint32)area_score_count * 40U) /
                           BLUE_TARGET_FULL_AREA_SAMPLES);
    if (confidence > 100U)
    {
        confidence = 100U;
    }

    result->valid = (confidence >= BLUE_TARGET_MIN_VALID_CONFIDENCE_PCT) ? 1U : 0U;
    result->confidence_pct = (uint8)confidence;
    result->fill_pct = (uint8)fill_pct;
    result->center_x = (uint16)((component->sum_x * BLUE_TARGET_SAMPLE_STEP) /
                                component->count);
    result->center_y = (uint16)((component->sum_y * BLUE_TARGET_SAMPLE_STEP) /
                                component->count);
    result->bbox_x = (uint16)(component->min_x * BLUE_TARGET_SAMPLE_STEP);
    result->bbox_y = (uint16)(component->min_y * BLUE_TARGET_SAMPLE_STEP);
    result->bbox_width = (uint16)(sample_width * BLUE_TARGET_SAMPLE_STEP);
    result->bbox_height = (uint16)(sample_height * BLUE_TARGET_SAMPLE_STEP);
    result->area_px = (uint32)component->count *
                      BLUE_TARGET_SAMPLE_STEP * BLUE_TARGET_SAMPLE_STEP;

    offset_x = (int32)result->center_x - (int32)BLUE_TARGET_CENTER_X_PX;
    result->offset_x = (int16)offset_x;
    result->bearing_deg_x10 = (int16)((offset_x *
                                      BLUE_TARGET_HORIZONTAL_FOV_DEG_X10) /
                                     (int32)SCC8660_W);

#if (BLUE_TARGET_DISTANCE_K_CM_PX > 0U)
    result->distance_valid = 1U;
    calibrated_distance_cm =
        (BLUE_TARGET_DISTANCE_K_CM_PX + (result->bbox_height / 2U)) /
        result->bbox_height;
    result->distance_cm =
        (uint16)((calibrated_distance_cm > BLUE_TARGET_DISTANCE_OFFSET_CM) ?
                 (calibrated_distance_cm - BLUE_TARGET_DISTANCE_OFFSET_CM) :
                 1U);
#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    /* A marker clipped by the frame bottom is nearer than its visible height
       suggests. Keep it trackable, but force a conservative braking range. */
    if ((component->max_y >= (BLUE_TARGET_ROI_GRID_HEIGHT - 1U)) &&
        (result->distance_cm > BLUE_TARGET_YELLOW_CLIPPED_DISTANCE_CM))
    {
        result->distance_cm = BLUE_TARGET_YELLOW_CLIPPED_DISTANCE_CM;
    }
#endif
#else
    result->distance_valid = 0U;
    result->distance_cm = 0U;
#endif
}

static uint16 blue_target_abs_diff_u16(uint16 a, uint16 b)
{
    return (a >= b) ? (uint16)(a - b) : (uint16)(b - a);
}

static uint32 blue_target_acquire_score(
    const blue_target_component_t *component,
    const blue_target_result_t *candidate)
{
#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    uint32 bright_pct;
    uint16 area_score_count;
    uint16 center_error;
    uint32 score;

    if ((component == NULL) || (candidate == NULL) ||
        (component->count == 0U))
    {
        return 0U;
    }

    bright_pct = ((uint32)component->bright_count * 100U) /
                 component->count;
    area_score_count = component->count;
    if (area_score_count > BLUE_TARGET_ACQUIRE_AREA_CAP_SAMPLES)
    {
        area_score_count = BLUE_TARGET_ACQUIRE_AREA_CAP_SAMPLES;
    }
    center_error = blue_target_abs_diff_u16(candidate->center_x,
                                             BLUE_TARGET_CENTER_X_PX);
    if (center_error > (SCC8660_W / 2U))
    {
        center_error = (SCC8660_W / 2U);
    }

    /* Bright-yellow purity separates the marker from dark yellow-brown
     * ground. Centering is only a small tie-breaker for initial acquire. */
    score = (uint32)candidate->confidence_pct * 100U;
    score += bright_pct * BLUE_TARGET_ACQUIRE_BRIGHT_WEIGHT;
    score += (uint32)area_score_count * BLUE_TARGET_ACQUIRE_AREA_WEIGHT;
    score += ((SCC8660_W / 2U) - center_error) *
             BLUE_TARGET_ACQUIRE_CENTER_WEIGHT;
    if (component->max_y >= (BLUE_TARGET_ROI_GRID_HEIGHT - 1U))
    {
        score = (score > BLUE_TARGET_ACQUIRE_CLIPPED_PENALTY) ?
                (score - BLUE_TARGET_ACQUIRE_CLIPPED_PENALTY) : 1U;
    }
    return score;
#else
    (void)candidate;
    return (component != NULL) ? component->count : 0U;
#endif
}

static void blue_target_distance_filter_reset(void)
{
    g_blue_target_distance_last_cm = 0U;
    g_blue_target_distance_last_ms = 0U;
    g_blue_target_distance_filter_valid = 0U;
    g_blue_target_distance_jump_active = 0U;
}

static void blue_target_filter_distance(blue_target_result_t *result,
                                        uint32 now_ms)
{
    uint32 dt_ms;
    uint32 jump_limit_cm;
    uint32 recover_step_cm;
    int32 distance_delta_cm;
    uint16 filtered_distance_cm;

    if ((result == NULL) || (result->valid == 0U) ||
        (result->distance_valid == 0U) || (result->distance_cm == 0U))
    {
        return;
    }

#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
    if (((uint32)result->bbox_y + result->bbox_height) >=
        BLUE_TARGET_ROI_HEIGHT)
    {
        if (result->distance_cm > BLUE_TARGET_YELLOW_CLIPPED_DISTANCE_CM)
        {
            result->distance_cm = BLUE_TARGET_YELLOW_CLIPPED_DISTANCE_CM;
        }
        g_blue_target_distance_last_cm = result->distance_cm;
        g_blue_target_distance_last_ms = now_ms;
        g_blue_target_distance_filter_valid = 1U;
        g_blue_target_distance_jump_active = 0U;
        return;
    }
#endif

    if (g_blue_target_distance_filter_valid == 0U)
    {
        g_blue_target_distance_last_cm = result->distance_cm;
        g_blue_target_distance_last_ms = now_ms;
        g_blue_target_distance_filter_valid = 1U;
        g_blue_target_distance_jump_active = 0U;
        return;
    }

    dt_ms = now_ms - g_blue_target_distance_last_ms;
    jump_limit_cm = BLUE_TARGET_DISTANCE_JUMP_BASE_CM +
                    ((uint32)BLUE_TARGET_DISTANCE_JUMP_RATE_CMPS * dt_ms) /
                    1000U;
    if (jump_limit_cm > BLUE_TARGET_DISTANCE_JUMP_MAX_CM)
    {
        jump_limit_cm = BLUE_TARGET_DISTANCE_JUMP_MAX_CM;
    }

    /* Reject one size glitch, then rate-limit a persistent range change. */
    if (blue_target_abs_diff_u16(result->distance_cm,
                                 g_blue_target_distance_last_cm) >
        jump_limit_cm)
    {
        if (g_blue_target_distance_jump_active == 0U)
        {
            g_blue_target_distance_jump_active = 1U;
            result->distance_cm = g_blue_target_distance_last_cm;
            g_blue_target_distance_last_ms = now_ms;
            return;
        }

        recover_step_cm =
            ((uint32)BLUE_TARGET_DISTANCE_JUMP_RECOVER_CMPS * dt_ms) / 1000U;
        if (recover_step_cm == 0U)
        {
            recover_step_cm = 1U;
        }
        distance_delta_cm = (int32)result->distance_cm -
                            (int32)g_blue_target_distance_last_cm;
        if (distance_delta_cm > (int32)recover_step_cm)
        {
            filtered_distance_cm =
                (uint16)(g_blue_target_distance_last_cm + recover_step_cm);
        }
        else if (distance_delta_cm < -(int32)recover_step_cm)
        {
            filtered_distance_cm =
                (g_blue_target_distance_last_cm > recover_step_cm) ?
                (uint16)(g_blue_target_distance_last_cm - recover_step_cm) : 0U;
        }
        else
        {
            filtered_distance_cm = result->distance_cm;
        }
        g_blue_target_distance_last_cm = filtered_distance_cm;
        g_blue_target_distance_last_ms = now_ms;
        result->distance_cm = filtered_distance_cm;
        return;
    }

    g_blue_target_distance_jump_active = 0U;
    filtered_distance_cm =
        (uint16)(((uint32)g_blue_target_distance_last_cm *
                  (100U - BLUE_TARGET_DISTANCE_FILTER_ALPHA_PCT) +
                  (uint32)result->distance_cm *
                  BLUE_TARGET_DISTANCE_FILTER_ALPHA_PCT + 50U) /
                 100U);
    g_blue_target_distance_last_cm = filtered_distance_cm;
    g_blue_target_distance_last_ms = now_ms;
    result->distance_cm = filtered_distance_cm;
}

static uint8 blue_target_track_match(const blue_target_result_t *candidate)
{
    uint16 center_dx_limit = BLUE_TARGET_TRACK_MAX_DX_PX;
    uint16 height_pct_limit = BLUE_TARGET_TRACK_MAX_DH_PCT;
    uint16 height_limit;

    if ((candidate == NULL) ||
        (g_blue_target_track_locked == 0U) ||
        (candidate->confidence_pct < BLUE_TARGET_TRACK_MIN_CONFIDENCE_PCT))
    {
        return 0U;
    }

    if ((candidate->center_x <= BLUE_TARGET_TRACK_EDGE_MARGIN_PX) ||
        (candidate->center_x >=
         (SCC8660_W - BLUE_TARGET_TRACK_EDGE_MARGIN_PX)) ||
        (g_blue_target_track_result.center_x <=
         BLUE_TARGET_TRACK_EDGE_MARGIN_PX) ||
        (g_blue_target_track_result.center_x >=
         (SCC8660_W - BLUE_TARGET_TRACK_EDGE_MARGIN_PX)))
    {
        center_dx_limit = BLUE_TARGET_TRACK_EDGE_MAX_DX_PX;
        height_pct_limit = BLUE_TARGET_TRACK_EDGE_MAX_DH_PCT;
    }

    if ((blue_target_abs_diff_u16(candidate->center_x,
                                  g_blue_target_track_result.center_x) >
         center_dx_limit) ||
        (blue_target_abs_diff_u16(candidate->center_y,
                                  g_blue_target_track_result.center_y) >
         BLUE_TARGET_TRACK_MAX_DY_PX))
    {
        return 0U;
    }

    height_limit = (uint16)(((uint32)g_blue_target_track_result.bbox_height *
                             height_pct_limit) / 100U);
    if (height_limit < BLUE_TARGET_TRACK_MIN_DH_PX)
    {
        height_limit = BLUE_TARGET_TRACK_MIN_DH_PX;
    }
    if (blue_target_abs_diff_u16(candidate->bbox_height,
                                 g_blue_target_track_result.bbox_height) >
        height_limit)
    {
        return 0U;
    }

    return 1U;
}

static uint32 blue_target_track_score(const blue_target_result_t *candidate)
{
    uint32 score;

    score = (uint32)blue_target_abs_diff_u16(candidate->center_x,
                                             g_blue_target_track_result.center_x) * 8U;
    score += (uint32)blue_target_abs_diff_u16(candidate->center_y,
                                              g_blue_target_track_result.center_y) * 3U;
    score += (uint32)blue_target_abs_diff_u16(candidate->bbox_height,
                                              g_blue_target_track_result.bbox_height) * 5U;
    score += (uint32)(100U - candidate->confidence_pct);
    return score;
}

void blue_target_reset(void)
{
    blue_target_shared()->reset_request = 1U;
    __dsync();
}

void blue_target_process(const uint16 *rgb565_image)
{
    blue_target_result_t next_result;
    blue_target_result_t best_acquire_result;
    blue_target_result_t best_track_result;
    uint16 best_count = 0U;
    uint32 best_acquire_score = 0U;
    uint32 best_track_score = 0xFFFFFFFFU;
    uint16 grid_x;
    uint16 grid_y;
    uint16 index;
    uint8 yellow_dusk_mode;
    uint32 now_ms;
    uint32 start_us = system_getval_us();

    memset(&next_result, 0, sizeof(next_result));
    memset(&best_acquire_result, 0, sizeof(best_acquire_result));
    memset(&best_track_result, 0, sizeof(best_track_result));
    if (rgb565_image == NULL)
    {
        g_blue_target_work_result = next_result;
        blue_target_publish_result(&next_result);
        return;
    }

    yellow_dusk_mode = blue_target_yellow_dusk_mode(rgb565_image);

    index = 0U;
    for (grid_y = 0U; grid_y < BLUE_TARGET_GRID_HEIGHT; grid_y++)
    {
        const uint16 *row = rgb565_image +
                            (uint32)(grid_y * BLUE_TARGET_SAMPLE_STEP) * SCC8660_W;
        for (grid_x = 0U; grid_x < BLUE_TARGET_GRID_WIDTH; grid_x++)
        {
            if (grid_y < BLUE_TARGET_ROI_GRID_HEIGHT)
            {
                g_blue_target_mask[index++] =
                    blue_target_is_color(row[grid_x * BLUE_TARGET_SAMPLE_STEP],
                                         yellow_dusk_mode);
            }
            else
            {
                g_blue_target_mask[index++] = 0U;
            }
        }
    }

    for (index = 0U; index < BLUE_TARGET_GRID_SIZE; index++)
    {
        blue_target_component_t component;
        blue_target_result_t candidate;
        uint32 acquire_score;
        uint32 track_score;

        if ((g_blue_target_mask[index] & BLUE_TARGET_MASK_COLOR) == 0U)
        {
            continue;
        }
        blue_target_read_component(index, &component);
        if (blue_target_component_is_candidate(&component) == 0U)
        {
            continue;
        }

        memset(&candidate, 0, sizeof(candidate));
        blue_target_make_result(&component, &candidate);
#if (BLUE_TARGET_COLOR_MODE == BLUE_TARGET_COLOR_YELLOW)
        acquire_score = (candidate.confidence_pct >=
                         BLUE_TARGET_MIN_VALID_CONFIDENCE_PCT) ?
                        blue_target_acquire_score(&component, &candidate) : 0U;
#else
        acquire_score = blue_target_acquire_score(&component, &candidate);
#endif
        if ((acquire_score > best_acquire_score) ||
            ((acquire_score == best_acquire_score) &&
             (component.count > best_count)))
        {
            best_acquire_score = acquire_score;
            best_count = component.count;
            best_acquire_result = candidate;
        }

        if (blue_target_track_match(&candidate) != 0U)
        {
            track_score = blue_target_track_score(&candidate);
            if (track_score < best_track_score)
            {
                best_track_score = track_score;
                best_track_result = candidate;
            }
        }
    }

    now_ms = system_getval_ms();
    if ((g_blue_target_track_locked != 0U) &&
        (best_track_score != 0xFFFFFFFFU))
    {
        best_track_result.valid = 1U;
        next_result = best_track_result;
        g_blue_target_track_result = best_track_result;
        g_blue_target_track_last_ms = now_ms;
    }
    else if (g_blue_target_track_locked != 0U)
    {
        next_result = g_blue_target_track_result;
        next_result.valid = 0U;
        next_result.distance_valid = 0U;
        next_result.confidence_pct = 0U;

        if ((g_blue_target_track_last_ms == 0U) ||
            ((now_ms - g_blue_target_track_last_ms) >
             BLUE_TARGET_TRACK_RELEASE_MS))
        {
            g_blue_target_track_locked = 0U;
        }
    }

    if ((g_blue_target_track_locked == 0U) &&
        (best_acquire_score != 0U) && (best_count != 0U))
    {
        next_result = best_acquire_result;
        if (best_acquire_result.confidence_pct >=
            BLUE_TARGET_MIN_VALID_CONFIDENCE_PCT)
        {
            next_result.valid = 1U;
            /* Keep range history across a re-acquire: the first new size
             * measurement must still be rejected when it is implausible. */
            g_blue_target_track_result = next_result;
            g_blue_target_track_last_ms = now_ms;
            g_blue_target_track_locked = 1U;
        }
    }
    blue_target_filter_distance(&next_result, now_ms);
    next_result.process_us = (uint16)(system_getval_us() - start_us);
    g_blue_target_work_result = next_result;
    blue_target_publish_result(&next_result);
}

void blue_target_get_result(blue_target_result_t *result)
{
    if (result != NULL)
    {
        blue_target_shared_t *shared = blue_target_shared();
        uint32 sequence_before;
        uint32 sequence_after;

        do
        {
            sequence_before = shared->result_sequence;
            __dsync();
            *result = shared->result;
            __dsync();
            sequence_after = shared->result_sequence;
        }
        while (((sequence_before & 1U) != 0U) ||
               (sequence_before != sequence_after));
    }
}

uint8 blue_target_camera_start(void)
{
    if ((g_blue_target_camera_init_done == 0U) ||
        (g_blue_target_camera_ready == 0U))
    {
        return blue_target_camera_init_once();
    }

    return (g_blue_target_camera_ready != 0U) ? 0U : 1U;
}

uint8 blue_target_camera_retry(void)
{
    if (g_blue_target_camera_ready != 0U)
    {
        return 0U;
    }
    return blue_target_camera_init_once();
}

uint8 blue_target_camera_is_ready(void)
{
    return g_blue_target_camera_ready;
}

void blue_target_camera_worker_enable(uint8 enable)
{
    blue_target_shared_t *shared = blue_target_shared();

    shared->worker_enabled = (enable != 0U) ? 1U : 0U;
    if (enable != 0U)
    {
        shared->reset_request = 1U;
    }
    __dsync();
}

void blue_target_camera_worker_task(void)
{
    blue_target_shared_t *shared = blue_target_shared();
    const uint16 *image;
    uint32 worker_start_us;
    uint32 copy_start_us;

    shared->worker_heartbeat++;
    shared->worker_last_task_ms = system_getval_ms();
    __dsync();

    if (shared->reset_request != 0U)
    {
        if (blue_target_reset_on_worker() != 0U)
        {
            shared->reset_request = 0U;
            __dsync();
        }
        else
        {
            return;
        }
    }

    if (shared->worker_enabled == 0U)
    {
        return;
    }

    if (guimai_voice_is_recording() != 0U)
    {
        if (scc8660_frame_is_ready() != 0U)
        {
            blue_target_release_camera_frame();
        }
        return;
    }

    if (scc8660_frame_is_ready() == 0U)
    {
        return;
    }

    worker_start_us = system_getval_us();
    image = scc8660_get_frame_buffer();
    blue_target_process(image);
    shared->frame_count++;
    shared->last_frame_ms = system_getval_ms();
    __dsync();

    if (IfxCpu_acquireMutex(&shared->preview_lock))
    {
        copy_start_us = system_getval_us();
        memcpy(shared->preview,
               image,
               sizeof(shared->preview));
        shared->preview_copy_us = system_getval_us() - copy_start_us;
        shared->preview_result = g_blue_target_work_result;
        shared->preview_frame_count = shared->frame_count;
        __dsync();
        IfxCpu_releaseMutex(&shared->preview_lock);
    }

    shared->worker_total_us = system_getval_us() - worker_start_us;
    __dsync();
    blue_target_release_camera_frame();
}

uint32 blue_target_camera_get_frame_count(void)
{
    __dsync();
    return blue_target_shared()->frame_count;
}

void blue_target_camera_get_health(uint32 *worker_heartbeat,
                                   uint32 *worker_last_task_ms,
                                   uint32 *last_frame_ms,
                                   uint8 *worker_enabled,
                                   uint8 *reset_pending)
{
    blue_target_shared_t *shared = blue_target_shared();

    __dsync();
    if (worker_heartbeat != NULL)
    {
        *worker_heartbeat = shared->worker_heartbeat;
    }
    if (worker_last_task_ms != NULL)
    {
        *worker_last_task_ms = shared->worker_last_task_ms;
    }
    if (last_frame_ms != NULL)
    {
        *last_frame_ms = shared->last_frame_ms;
    }
    if (worker_enabled != NULL)
    {
        *worker_enabled = shared->worker_enabled;
    }
    if (reset_pending != NULL)
    {
        *reset_pending = shared->reset_request;
    }
}

void blue_target_camera_get_timing(uint32 *preview_copy_us,
                                   uint32 *worker_total_us)
{
    blue_target_shared_t *shared = blue_target_shared();

    __dsync();
    if (preview_copy_us != NULL)
    {
        *preview_copy_us = shared->preview_copy_us;
    }
    if (worker_total_us != NULL)
    {
        *worker_total_us = shared->worker_total_us;
    }
}

uint8 blue_target_preview_acquire(const uint16 **image,
                                  blue_target_result_t *result,
                                  uint32 *frame_count)
{
    blue_target_shared_t *shared = blue_target_shared();

    if ((image == NULL) || (result == NULL) || (frame_count == NULL))
    {
        return 0U;
    }

    if (!IfxCpu_acquireMutex(&shared->preview_lock))
    {
        return 0U;
    }

    __dsync();
    if (shared->preview_frame_count == 0U)
    {
        IfxCpu_releaseMutex(&shared->preview_lock);
        return 0U;
    }

    *image = shared->preview[0];
    *result = shared->preview_result;
    *frame_count = shared->preview_frame_count;
    return 1U;
}

void blue_target_preview_release(void)
{
    __dsync();
    IfxCpu_releaseMutex(&blue_target_shared()->preview_lock);
}
