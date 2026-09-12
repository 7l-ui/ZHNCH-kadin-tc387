#ifndef _UI_IMAGE_H_
#define _UI_IMAGE_H_

#include "zf_common_headfile.h"

#define DISPLAY_POINT_MAX   (100)

typedef struct
{
    double lat;
    double lon;
} gps_point;

typedef enum
{
    SCREEN_IPS114,
    SCREEN_IPS200_SPI,
    SCREEN_IPS200_PARALLEL8,
    SCREEN_TFT180,
} screen_type_enum;

extern int16_t image_offset_x;
extern int16_t image_offset_y;
extern int16_t image_multiple;
extern int8_t image_choise;

void UI_image_show(void);
void UI_Image_Task(void);
void UI_Image_Callback(uint8 dir);
void UI_Image_SelectNext(void);
void UI_Image_AddPoint(double lat, double lon);
void UI_Image_RemoveLastPoint(void);
uint8 UI_Image_GetPointCount(void);
const gps_point *UI_Image_GetPoints(void);
uint8 UI_Image_IsPicking(void);
uint8 UI_Image_GetPickCount(void);
uint8 UI_Image_GetPickTarget(void);
uint8 UI_Image_SavePoints(uint8 task_id);
uint8 UI_Image_LoadPoints(uint8 task_id);
void UI_Image_LoadAllPoints(void);
void UI_Image_ClearPoints(uint8 task_id);

void user_gps_transition(gps_point *gps_point_input, int16 point_num);
void user_gps_display(uint16 color);
void user_gps_display_init(screen_type_enum screen_type, int16 start_x, int16 start_y, int16 width, int16 high);

#endif
