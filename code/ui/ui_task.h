#ifndef _UI_TASK_H_
#define _UI_TASK_H_

#include "zf_common_headfile.h"

#define UI_TASK_MIN_ID          (0U)
#define UI_TASK_MAX_ID          (4U)
#define UI_TASK_DEFAULT_ID      (3U)
#define UI_TASK_COUNT           (UI_TASK_MAX_ID - UI_TASK_MIN_ID + 1U)

typedef struct
{
    uint8  program_select;      // SetPage select item (1~2)
    uint16 value1;              // Task param 1
    uint8  value2;              // Task param 2
} ui_task_state_t;

extern uint8 g_ui_task_id;
extern ui_task_state_t g_ui_task_state[UI_TASK_COUNT];

static inline void ui_task_set(uint8 task_id)
{
    if (task_id > UI_TASK_MAX_ID)
    {
        task_id = UI_TASK_MAX_ID;
    }
    g_ui_task_id = task_id;
}

static inline void ui_task_next(void)
{
    if (g_ui_task_id >= UI_TASK_MAX_ID)
    {
        g_ui_task_id = UI_TASK_MIN_ID;
    }
    else
    {
        g_ui_task_id++;
    }
}

static inline void ui_task_prev(void)
{
    if (g_ui_task_id <= UI_TASK_MIN_ID)
    {
        g_ui_task_id = UI_TASK_MAX_ID;
    }
    else
    {
        g_ui_task_id--;
    }
}

static inline uint8 ui_task_get(void)
{
    return g_ui_task_id;
}

static inline uint8 ui_task_to_index(uint8 task_id)
{
    if (task_id > UI_TASK_MAX_ID)
    {
        return UI_TASK_MAX_ID;
    }
    return task_id;
}

static inline ui_task_state_t *ui_task_get_state(uint8 task_id)
{
    return &g_ui_task_state[ui_task_to_index(task_id)];
}

static inline ui_task_state_t *ui_task_curr_state(void)
{
    return ui_task_get_state(g_ui_task_id);
}

#endif
