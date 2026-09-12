#ifndef _DIANJI_H_
#define _DIANJI_H_

#include "zf_common_headfile.h"

typedef enum
{
    DIANJI_MOTOR_M = 0,   // middle
    DIANJI_MOTOR_R = 1,   // right
    DIANJI_MOTOR_L = 2,   // left
    DIANJI_MOTOR_MAX

} dianji_motor_id_t;

typedef struct
{
    uint8 initialized;
    uint8 enable_requested;
    uint8 enable_active;
    uint8 fault_latched;
    int16 target_duty[DIANJI_MOTOR_MAX];
} dianji_status_t;

//-------------------------------------------------------------------------------------------------------------------
// Brief          Motor module init (power-on safe sequence)
// Notes          1) Set indicator nets to inactive first
//                2) Init configured motor outputs with 0 duty
//                3) Hold safe delay after power-on
//-------------------------------------------------------------------------------------------------------------------
void dianji_init(void);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Periodic task (recommended every 1~10ms)
// Notes          Handles request guard delay and output state machine
//-------------------------------------------------------------------------------------------------------------------
void dianji_task(void);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Request/cancel driver enable
// Parameter      enable_req: 1=request enable, 0=disable
// Notes          Output is applied after guard delay, not immediately
//-------------------------------------------------------------------------------------------------------------------
void dianji_request_enable(uint8 enable_req);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Emergency stop (disable immediately and latch fault)
//-------------------------------------------------------------------------------------------------------------------
void dianji_emergency_stop(void);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Clear fault latch (does not auto-enable)
//-------------------------------------------------------------------------------------------------------------------
void dianji_clear_fault(void);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Set one motor target duty
// Parameter      duty_signed range: [-PWM_DUTY_MAX, PWM_DUTY_MAX]
// Notes          PWM/PWM mode: direction is selected by DIANJI_PWM_OUTPUT_SIGN_*
//                DRV8701 PH/EN mode: APWM net -> PH GPIO, BPWM net -> EN PWM
//                Opposite direction changes are blanked before output resumes.
//-------------------------------------------------------------------------------------------------------------------
void dianji_set_motor_duty(dianji_motor_id_t motor, int16 duty_signed);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Set all 3 motors target duty together
// Parameter      duty_m/duty_r/duty_l range: [-PWM_DUTY_MAX, PWM_DUTY_MAX]
//-------------------------------------------------------------------------------------------------------------------
void dianji_set_all_duty(int16 duty_m, int16 duty_r, int16 duty_l);

//-------------------------------------------------------------------------------------------------------------------
// Brief          Get current module status
//-------------------------------------------------------------------------------------------------------------------
void dianji_get_status(dianji_status_t *status);

#endif
