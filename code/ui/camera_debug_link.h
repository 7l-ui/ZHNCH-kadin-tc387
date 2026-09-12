#ifndef _CAMERA_DEBUG_LINK_H_
#define _CAMERA_DEBUG_LINK_H_

/* Polls DEBUG UART commands. Call from CPU0's normal main loop, never an ISR. */
void camera_debug_link_task(void);

#endif
