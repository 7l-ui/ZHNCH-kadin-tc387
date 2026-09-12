#include "UI_image.h"
#include "ui_task.h"
#include "nav_gps.h"
#include "zf_driver_flash.h"

#define EPSILON              (1e-9)
#define POINT_STORAGE_MAX    (30)
#define PICK_SAMPLE_TARGET   (10U)
#define UI_IMAGE_FLASH_SECTION     (0U)
#define UI_IMAGE_FLASH_PAGE_BASE   (100U)
#define UI_IMAGE_FLASH_MAGIC       (0x47505350UL)
#define UI_IMAGE_FLASH_VERSION     (1U)
#define GPS_DISPLAY_MARGIN_PX      (4.0)

int16_t image_multiple = 10;
int16_t image_offset_x = 0;
int16_t image_offset_y = 0;
int8_t image_choise = 1;

static gps_point g_points[UI_TASK_COUNT][POINT_STORAGE_MAX];
static uint8 g_point_count[UI_TASK_COUNT] = {0};
static int8_t image_point[UI_TASK_COUNT] = {0};
static uint8 g_pick_active[UI_TASK_COUNT] = {0};
static uint8 g_pick_count[UI_TASK_COUNT] = {0};
static uint32 g_pick_last_ms[UI_TASK_COUNT] = {0};
static double g_pick_lat[UI_TASK_COUNT][PICK_SAMPLE_TARGET];
static double g_pick_lon[UI_TASK_COUNT][PICK_SAMPLE_TARGET];

typedef struct
{
    double x;
    double y;
} plane_point;

typedef struct
{
    double x;
    double y;
} screen_point;

static int16 display_start_x = 0;
static int16 display_start_y = 0;
static int16 display_width = 0;
static int16 display_high = 0;
static int16 transition_point_num = 0;

static plane_point plane_point_data[DISPLAY_POINT_MAX];
static screen_point screen_point_data[DISPLAY_POINT_MAX];
static screen_type_enum display_screen_type = SCREEN_IPS200_SPI;

static double UI_Image_ClampDouble(double value, double min_value, double max_value)
{
    if (value != value)
    {
        return min_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16 UI_Image_ToDisplayX(double x)
{
    int32 value = (int32)(x + 0.5) + display_start_x;
    int32 min_value = display_start_x;
    int32 max_value = display_start_x + display_width;

    if (x != x)
    {
        value = min_value;
    }
    if (value < min_value)
    {
        value = min_value;
    }
    if (value > max_value)
    {
        value = max_value;
    }
    return (uint16)value;
}

static uint16 UI_Image_ToDisplayY(double y)
{
    int32 value = (int32)(y + 0.5) + display_start_y;
    int32 min_value = display_start_y;
    int32 max_value = display_start_y + display_high;

    if (y != y)
    {
        value = min_value;
    }
    if (value < min_value)
    {
        value = min_value;
    }
    if (value > max_value)
    {
        value = max_value;
    }
    return (uint16)value;
}

static void UI_Image_DrawPoint(uint16 x, uint16 y, uint16 color)
{
    uint16 min_x = (uint16)display_start_x;
    uint16 min_y = (uint16)display_start_y;
    uint16 max_x = (uint16)(display_start_x + display_width);
    uint16 max_y = (uint16)(display_start_y + display_high);

    ips200_draw_point(x, y, color);
    if (x > min_x) ips200_draw_point((uint16)(x - 1U), y, color);
    if (x < max_x) ips200_draw_point((uint16)(x + 1U), y, color);
    if (y > min_y) ips200_draw_point(x, (uint16)(y - 1U), color);
    if (y < max_y) ips200_draw_point(x, (uint16)(y + 1U), color);
}

static void UI_Image_ClearDisplayArea(void)
{
    uint16 min_x = (uint16)display_start_x;
    uint16 min_y = (uint16)display_start_y;
    uint16 max_x = (uint16)(display_start_x + display_width);
    uint16 max_y = (uint16)(display_start_y + display_high);

    for (uint16 y = min_y; y <= max_y; y++)
    {
        ips200_draw_line(min_x, y, max_x, y, RGB565_WHITE);
    }
}

typedef union
{
    double double_type;
    uint32 uint32_type[2];
} ui_image_double_u32_t;

static uint8 ui_img_task_idx(void)
{
    return ui_task_to_index(ui_task_get());
}

static uint8 UI_Image_IsFinite(double value)
{
    return ((value == value) && (value < 1.0e12) && (value > -1.0e12)) ? 1U : 0U;
}

static uint8 UI_Image_IsGpsPointValid(double lat, double lon)
{
    if ((0U == UI_Image_IsFinite(lat)) || (0U == UI_Image_IsFinite(lon)))
    {
        return 0U;
    }
    if ((lat < -90.0) || (lat > 90.0) || (lon < -180.0) || (lon > 180.0))
    {
        return 0U;
    }
    if ((0.0 == lat) && (0.0 == lon))
    {
        return 0U;
    }
    return 1U;
}

static uint32 UI_Image_FlashPage(uint8 task_id)
{
    return UI_IMAGE_FLASH_PAGE_BASE + (uint32)ui_task_to_index(task_id);
}

static void UI_Image_ClearTaskRam(uint8 idx)
{
    uint8 i;

    if (idx >= UI_TASK_COUNT)
    {
        return;
    }

    g_point_count[idx] = 0U;
    image_point[idx] = 0;
    g_pick_active[idx] = 0U;
    g_pick_count[idx] = 0U;
    g_pick_last_ms[idx] = 0U;

    for (i = 0U; i < POINT_STORAGE_MAX; i++)
    {
        g_points[idx][i].lat = 0.0;
        g_points[idx][i].lon = 0.0;
    }
}

static void UI_Image_ClearFlashWriteBuffer(void)
{
    uint16 i;

    for (i = 0U; i < EEPROM_PAGE_LENGTH; i++)
    {
        flash_union_buffer[i].uint32_type = 0U;
    }
}

uint8 UI_Image_SavePoints(uint8 task_id)
{
    uint8 idx = ui_task_to_index(task_id);
    uint16 write_index = 0U;
    uint8 i;
    ui_image_double_u32_t value;

    if ((3U + ((uint16)POINT_STORAGE_MAX * 4U)) > EEPROM_PAGE_LENGTH)
    {
        return 1U;
    }

    UI_Image_ClearFlashWriteBuffer();
    flash_union_buffer[write_index++].uint32_type = UI_IMAGE_FLASH_MAGIC;
    flash_union_buffer[write_index++].uint32_type = UI_IMAGE_FLASH_VERSION;
    flash_union_buffer[write_index++].uint32_type = g_point_count[idx];

    for (i = 0U; i < g_point_count[idx]; i++)
    {
        value.double_type = g_points[idx][i].lat;
        flash_union_buffer[write_index++].uint32_type = value.uint32_type[0];
        flash_union_buffer[write_index++].uint32_type = value.uint32_type[1];

        value.double_type = g_points[idx][i].lon;
        flash_union_buffer[write_index++].uint32_type = value.uint32_type[0];
        flash_union_buffer[write_index++].uint32_type = value.uint32_type[1];
    }

    if (flash_check(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx)))
    {
        flash_erase_page(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx));
    }

    return flash_write_page_from_buffer(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx));
}

uint8 UI_Image_LoadPoints(uint8 task_id)
{
    uint8 idx = ui_task_to_index(task_id);
    uint16 read_index = 0U;
    uint32 count;
    uint8 i;
    ui_image_double_u32_t value;

    UI_Image_ClearTaskRam(idx);

    if (0U == flash_check(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx)))
    {
        return 1U;
    }

    flash_read_page_to_buffer(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx));
    if ((flash_union_buffer[read_index++].uint32_type != UI_IMAGE_FLASH_MAGIC) ||
        (flash_union_buffer[read_index++].uint32_type != UI_IMAGE_FLASH_VERSION))
    {
        return 1U;
    }

    count = flash_union_buffer[read_index++].uint32_type;
    if (count > POINT_STORAGE_MAX)
    {
        return 1U;
    }

    for (i = 0U; i < (uint8)count; i++)
    {
        value.uint32_type[0] = flash_union_buffer[read_index++].uint32_type;
        value.uint32_type[1] = flash_union_buffer[read_index++].uint32_type;
        {
            double lat = value.double_type;

            value.uint32_type[0] = flash_union_buffer[read_index++].uint32_type;
            value.uint32_type[1] = flash_union_buffer[read_index++].uint32_type;
            if (0U != UI_Image_IsGpsPointValid(lat, value.double_type))
            {
                g_points[idx][g_point_count[idx]].lat = lat;
                g_points[idx][g_point_count[idx]].lon = value.double_type;
                g_point_count[idx]++;
            }
        }
    }

    image_point[idx] = 0;
    return 0U;
}

void UI_Image_LoadAllPoints(void)
{
    uint8 i;

    for (i = 0U; i < UI_TASK_COUNT; i++)
    {
        (void)UI_Image_LoadPoints(i);
    }
}

void UI_Image_ClearPoints(uint8 task_id)
{
    uint8 idx = ui_task_to_index(task_id);

    UI_Image_ClearTaskRam(idx);
    if (flash_check(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx)))
    {
        flash_erase_page(UI_IMAGE_FLASH_SECTION, UI_Image_FlashPage(idx));
    }
}

static double UI_Image_RemoveOutlierAverage(const double *buffer, uint8 count)
{
    double max_value = buffer[0];
    double min_value = buffer[0];
    double sum = 0.0;
    uint8 max_removed = 0U;
    uint8 min_removed = 0U;
    uint8 i;

    if (count <= 2U)
    {
        for (i = 0U; i < count; i++)
        {
            sum += buffer[i];
        }
        return sum / (double)count;
    }

    for (i = 1U; i < count; i++)
    {
        if (buffer[i] > max_value) max_value = buffer[i];
        if (buffer[i] < min_value) min_value = buffer[i];
    }

    for (i = 0U; i < count; i++)
    {
        if ((0U == max_removed) && (buffer[i] == max_value))
        {
            max_removed = 1U;
        }
        else if ((0U == min_removed) && (buffer[i] == min_value))
        {
            min_removed = 1U;
        }
        else
        {
            sum += buffer[i];
        }
    }

    return sum / (double)(count - 2U);
}

static void UI_Image_CommitPickedPoint(uint8 idx)
{
    double lat;
    double lon;

    if ((idx >= UI_TASK_COUNT) || (g_point_count[idx] >= POINT_STORAGE_MAX))
    {
        return;
    }

    lat = UI_Image_RemoveOutlierAverage(g_pick_lat[idx], g_pick_count[idx]);
    lon = UI_Image_RemoveOutlierAverage(g_pick_lon[idx], g_pick_count[idx]);

    g_points[idx][g_point_count[idx]].lat = lat;
    g_points[idx][g_point_count[idx]].lon = lon;
    g_point_count[idx]++;
    if (g_point_count[idx] == 1U)
    {
        image_point[idx] = 0;
    }
    (void)UI_Image_SavePoints(idx);
}

void UI_Image_Task(void)
{
    uint8 idx = ui_img_task_idx();
    nav_gps_sample_t sample;

    if ((idx >= UI_TASK_COUNT) || (0U == g_pick_active[idx]))
    {
        return;
    }

    nav_gps_get_last_sample(&sample);
    if ((0U == sample.valid) ||
        (0.0 == sample.latitude_deg) ||
        (0.0 == sample.longitude_deg) ||
        (sample.timestamp_ms == g_pick_last_ms[idx]))
    {
        return;
    }

    g_pick_last_ms[idx] = sample.timestamp_ms;
    if (g_pick_count[idx] < PICK_SAMPLE_TARGET)
    {
        g_pick_lat[idx][g_pick_count[idx]] = sample.latitude_deg;
        g_pick_lon[idx][g_pick_count[idx]] = sample.longitude_deg;
        g_pick_count[idx]++;
    }

    if (g_pick_count[idx] >= PICK_SAMPLE_TARGET)
    {
        UI_Image_CommitPickedPoint(idx);
        g_pick_active[idx] = 0U;
        g_pick_count[idx] = 0U;
    }
}

void UI_Image_AddPoint(double lat, double lon)
{
    uint8 idx = ui_img_task_idx();
    (void)lat;
    (void)lon;

    if (g_point_count[idx] >= POINT_STORAGE_MAX)
    {
        return;
    }
    g_pick_active[idx] = 1U;
    g_pick_count[idx] = 0U;
    g_pick_last_ms[idx] = 0U;
}

void UI_Image_RemoveLastPoint(void)
{
    uint8 idx = ui_img_task_idx();
    if (0U != g_pick_active[idx])
    {
        g_pick_active[idx] = 0U;
        g_pick_count[idx] = 0U;
        return;
    }
    if (g_point_count[idx] == 0)
    {
        return;
    }
    g_point_count[idx]--;
    g_points[idx][g_point_count[idx]].lat = 0;
    g_points[idx][g_point_count[idx]].lon = 0;
    if (image_point[idx] >= (int8_t)g_point_count[idx])
    {
        image_point[idx] = (g_point_count[idx] == 0) ? 0 : (int8_t)(g_point_count[idx] - 1);
    }
    (void)UI_Image_SavePoints(idx);
}

uint8 UI_Image_GetPointCount(void)
{
    return g_point_count[ui_img_task_idx()];
}

const gps_point *UI_Image_GetPoints(void)
{
    return g_points[ui_img_task_idx()];
}

uint8 UI_Image_IsPicking(void)
{
    return g_pick_active[ui_img_task_idx()];
}

uint8 UI_Image_GetPickCount(void)
{
    return g_pick_count[ui_img_task_idx()];
}

uint8 UI_Image_GetPickTarget(void)
{
    return PICK_SAMPLE_TARGET;
}

void UI_Image_SelectNext(void)
{
    image_choise++;
    if (image_choise > 3)
    {
        image_choise = 1;
    }
}

void UI_image_show(void)
{
    uint8 idx = ui_img_task_idx();
    if (image_choise != 1) ips200_show_string(60, 16 * 1, "  ");
    if (image_choise != 2) ips200_show_string(60, 16 * 2, "  ");
    if (image_choise != 3) ips200_show_string(60, 16 * 3, "  ");

    ips200_show_string(60, 16 * image_choise, "<");

    ips200_show_string(0, 16 * 1, "pit");
    ips200_show_int(30, 16 * 1, image_point[idx], 2);
    ips200_show_string(90, 16 * 1, "num");
    ips200_show_uint(120, 16 * 1, g_point_count[idx], 2);
    if (0U != g_pick_active[idx])
    {
        ips200_show_string(160, 16 * 1, "pick");
        ips200_show_uint(204, 16 * 1, g_pick_count[idx], 2);
        ips200_show_string(220, 16 * 1, "/");
        ips200_show_uint(228, 16 * 1, PICK_SAMPLE_TARGET, 2);
    }

    if (g_point_count[idx] > 0 && image_point[idx] >= 0 && image_point[idx] < (int8_t)g_point_count[idx])
    {
        ips200_show_float(0, 16 * 2, g_points[idx][image_point[idx]].lat, 2, 6);
        ips200_show_float(0, 16 * 3, g_points[idx][image_point[idx]].lon, 3, 6);
    }
    else
    {
        ips200_show_string(0, 16 * 2, "lat: --");
        ips200_show_string(0, 16 * 3, "lon: --");
    }
}

void UI_Image_Callback(uint8 dir)
{
    uint8 idx = ui_img_task_idx();
    if (image_choise == 1)
    {
        if (g_point_count[idx] == 0)
        {
            image_point[idx] = 0;
            return;
        }
        if (dir == 0)
        {
            image_point[idx]--;
            if (image_point[idx] < 0)
            {
                image_point[idx] = (int8_t)(g_point_count[idx] - 1);
            }
        }
        else
        {
            image_point[idx]++;
            if (image_point[idx] >= (int8_t)g_point_count[idx])
            {
                image_point[idx] = 0;
            }
        }
        return;
    }

    if (g_point_count[idx] == 0 || image_point[idx] < 0 || image_point[idx] >= (int8_t)g_point_count[idx])
    {
        return;
    }

    if (image_choise == 2)
    {
        if (dir == 0) g_points[idx][image_point[idx]].lat -= 0.000001;
        else g_points[idx][image_point[idx]].lat += 0.000001;
    }
    else if (image_choise == 3)
    {
        if (dir == 0) g_points[idx][image_point[idx]].lon -= 0.000001;
        else g_points[idx][image_point[idx]].lon += 0.000001;
    }
}

static void spherical_to_plane(gps_point *gps_point_input)
{
    nav_local_frame_t frame;
    nav_point_t point;

    if ((gps_point_input == NULL) || (transition_point_num <= 0))
    {
        return;
    }

    nav_gps_local_frame_init(&frame, gps_point_input[0].lat, gps_point_input[0].lon);
    for (int i = 0; i < transition_point_num; i++)
    {
        if (0U != nav_gps_local_frame_ll_to_xy(&frame,
                                                gps_point_input[i].lat,
                                                gps_point_input[i].lon,
                                                &point))
        {
            plane_point_data[i].x = point.x_m;
            plane_point_data[i].y = point.y_m;
        }
    }
}

static void plane_to_screen(void)
{
    double min_x = plane_point_data[0].x;
    double max_x = plane_point_data[0].x;
    double min_y = plane_point_data[0].y;
    double max_y = plane_point_data[0].y;
    double width;
    double height;
    double draw_width;
    double draw_high;
    double margin_x;
    double margin_y;
    double scale = 1.0;
    double x_offset;
    double y_offset;

    if ((transition_point_num <= 0) || (display_width <= 0) || (display_high <= 0))
    {
        return;
    }

    for (int i = 1; i < transition_point_num; i++)
    {
        min_x = fmin(min_x, plane_point_data[i].x);
        max_x = fmax(max_x, plane_point_data[i].x);
        min_y = fmin(min_y, plane_point_data[i].y);
        max_y = fmax(max_y, plane_point_data[i].y);
    }

    width = max_x - min_x;
    height = max_y - min_y;
    margin_x = GPS_DISPLAY_MARGIN_PX;
    margin_y = GPS_DISPLAY_MARGIN_PX;

    if (((double)display_width <= (2.0 * margin_x)) ||
        ((double)display_high <= (2.0 * margin_y)))
    {
        margin_x = 0.0;
        margin_y = 0.0;
    }

    draw_width = (double)display_width - (2.0 * margin_x);
    draw_high = (double)display_high - (2.0 * margin_y);
    x_offset = margin_x;
    y_offset = margin_y;

    if ((width <= EPSILON) && (height <= EPSILON))
    {
        for (int i = 0; i < transition_point_num; i++)
        {
            screen_point_data[i].x = (double)display_width / 2.0;
            screen_point_data[i].y = (double)display_high / 2.0;
        }
        return;
    }
    else if (width <= EPSILON)
    {
        scale = draw_high / height;
        x_offset = (double)display_width / 2.0;
    }
    else if (height <= EPSILON)
    {
        scale = draw_width / width;
        y_offset = (double)display_high / 2.0;
    }
    else
    {
        const double scale_x = draw_width / width;
        const double scale_y = draw_high / height;
        scale = fmin(scale_x, scale_y);
        x_offset += (draw_width - width * scale) / 2.0;
        y_offset += (draw_high - height * scale) / 2.0;
    }

    for (int i = 0; i < transition_point_num; i++)
    {
        if (width <= EPSILON)
        {
            screen_point_data[i].x = x_offset;
        }
        else
        {
            screen_point_data[i].x = (plane_point_data[i].x - min_x) * scale + x_offset;
        }

        if (height <= EPSILON)
        {
            screen_point_data[i].y = y_offset;
        }
        else
        {
            screen_point_data[i].y = (double)display_high -
                                     ((plane_point_data[i].y - min_y) * scale + y_offset);
        }

        screen_point_data[i].x = UI_Image_ClampDouble(screen_point_data[i].x, 0.0, (double)display_width);
        screen_point_data[i].y = UI_Image_ClampDouble(screen_point_data[i].y, 0.0, (double)display_high);
    }
}

void user_gps_transition(gps_point *gps_point_input, int16 point_num)
{
    if (point_num < 0)
    {
        point_num = 0;
    }
    if (point_num >= DISPLAY_POINT_MAX)
    {
        point_num = DISPLAY_POINT_MAX - 1;
    }

    transition_point_num = point_num;

    memset(plane_point_data, 0, sizeof(plane_point_data));
    memset(screen_point_data, 0, sizeof(screen_point_data));

    if (transition_point_num > 0)
    {
        spherical_to_plane(gps_point_input);
        plane_to_screen();
    }
}

void user_gps_display(uint16 display_color)
{
    uint16 x0;
    uint16 y0;
    uint16 x1;
    uint16 y1;

    UI_Image_ClearDisplayArea();

    for (int i = 0; i < transition_point_num; i++)
    {
        switch (display_screen_type)
        {
            case SCREEN_IPS200_SPI:
                x0 = UI_Image_ToDisplayX(screen_point_data[i].x);
                y0 = UI_Image_ToDisplayY(screen_point_data[i].y);
                UI_Image_DrawPoint(x0, y0, display_color);
                if (i < (transition_point_num - 1))
                {
                    x1 = UI_Image_ToDisplayX(screen_point_data[i + 1].x);
                    y1 = UI_Image_ToDisplayY(screen_point_data[i + 1].y);
                    ips200_draw_line(x0, y0, x1, y1, display_color);
                }
                break;
            case SCREEN_IPS114:
            case SCREEN_IPS200_PARALLEL8:
            case SCREEN_TFT180:
            default:
                break;
        }
    }
}

void user_gps_display_init(screen_type_enum screen_type, int16 start_x, int16 start_y, int16 width, int16 high)
{
    display_start_x = start_x;
    display_start_y = start_y;
    display_width = width - 1;
    display_high = high - 1;
    display_screen_type = screen_type;
}
