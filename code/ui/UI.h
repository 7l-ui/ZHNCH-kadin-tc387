#ifndef _UI_H_
#define _UI_H_

#include "zf_common_headfile.h"
#include "UI_image.h"
#include "ui_task.h"

typedef enum
{
    InfoPage = 0,
    SetPage,
    ParamPage,
    GPSPage,
    GPS_SetPage
} UI_Page;

extern UI_Page Page_Num;
extern uint8 UI_Mode;
extern bool Gui_Refersh_Bool;
extern uint8 Camera_compare;
extern uint8 program_select;

void UI_Init(void);
void UI(void);
void UI_KeyChange_Callback(uint8 dir);
void UI_GpsTest_Start(void);
void UI_GpsTest_Stop(void);
void UI_GpsTest_Reset(void);
void UI_GpsTest_MarkPoint(void);
void UI_Task3CameraSnapshotCycleSize(void);
void UI_Task3CameraSnapshotRequest(void);
void UI_Task3CameraCaptureModeToggle(void);
void UI_Task3CameraTestStart(uint8 mode);
uint8 UI_Task3FollowTestCaptureStart(void);
void UI_Task3CameraTestStop(void);
void UI_Task3CameraTestManualSave(void);

#endif
