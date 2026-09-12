#ifndef _DIANJI_CONFIG_H_
#define _DIANJI_CONFIG_H_

#include "zf_common_headfile.h"
#include "xl9555_app.h"

// ---------------- Driver mode ----------------
#define DIANJI_DRIVER_PWM_PWM             (0U)     // legacy APWM/BPWM H-bridge mode
#define DIANJI_DRIVER_PH_EN_8701          (1U)     // DRV8701E: APWM=PH, BPWM=EN/PWM
#define DIANJI_DRIVER_MODE                (DIANJI_DRIVER_PWM_PWM)

// ---------------- PWM/GPIO pin mapping (from schematic) ----------------
// M motor: APWM_M/BPWM_M -> P21.2/P21.3
// R motor: APWM_R/BPWM_R -> P21.4/P21.5
// L motor: APWM_L/BPWM_L -> P33.12/P33.13
#define DIANJI_PWM_APWM_M_CH              (ATOM0_CH0_P21_2)
#define DIANJI_PWM_BPWM_M_CH              (ATOM0_CH1_P21_3)
#define DIANJI_PWM_APWM_R_CH              (ATOM0_CH2_P21_4)
#define DIANJI_PWM_BPWM_R_CH              (ATOM0_CH3_P21_5)
#define DIANJI_PWM_APWM_L_CH              (ATOM2_CH4_P33_12)
#define DIANJI_PWM_BPWM_L_CH              (ATOM2_CH5_P33_13)

// DRV8701E PH/EN mode: APWM nets are used as direction GPIO.
#define DIANJI_PH_M_PIN                   (P21_2)
#define DIANJI_PH_R_PIN                   (P21_4)
#define DIANJI_PH_L_PIN                   (P33_12)

// DRV8701E truth table: EN=1 and PH=1/0 selects direction.
// Flip one of these if the corresponding motor direction is reversed.
#define DIANJI_PH_FORWARD_LEVEL_M         (1U)
#define DIANJI_PH_FORWARD_LEVEL_R         (0U)
#define DIANJI_PH_FORWARD_LEVEL_L         (0U)
#define DIANJI_PH_REVERSE_LEVEL_M         (0U)
#define DIANJI_PH_REVERSE_LEVEL_R         (1U)
#define DIANJI_PH_REVERSE_LEVEL_L         (1U)

// PWM/PWM mode direction map.
//  1: positive duty -> APWM, negative duty -> BPWM
// -1: positive duty -> BPWM, negative duty -> APWM
#define DIANJI_PWM_OUTPUT_SIGN_M          (1)
#define DIANJI_PWM_OUTPUT_SIGN_R          (1)
#define DIANJI_PWM_OUTPUT_SIGN_L          (1)

// ---------------- PWM parameters ----------------
#define DIANJI_PWM_FREQ_HZ                (10000U)
#define DIANJI_PWM_INIT_DUTY              (0U)
#define DIANJI_DUTY_LIMIT                 (9500U)   // rear motor limit, 0 ~ PWM_DUTY_MAX(10000)
#define DIANJI_DUTY_LIMIT_M               (9500U)   // front steer motor
#define DIANJI_DUTY_LIMIT_R               (DIANJI_DUTY_LIMIT)
#define DIANJI_DUTY_LIMIT_L               (DIANJI_DUTY_LIMIT)

// Software bridge protection for HIP4081 + large MOSFET bridge.
// Keeps at least this much off-time within a PWM period by clamping max duty.
// At 20kHz, 1us equals about 200 duty counts out of PWM_DUTY_MAX(10000).
#define DIANJI_PWM_SOFT_DEADTIME_US       (1U)

// When command direction changes (+ <-> -), force APWM/BPWM both off first.
// This is intentionally millisecond-level because the MOSFETs are paralleled.
#define DIANJI_DIR_CHANGE_BLANK_MS        (3U)

// In PH/EN mode, set PH first, then wait briefly before enabling EN PWM.
#define DIANJI_PH_EN_DIR_SETUP_US         (1U)

// ---------------- Power-on / enable timing ----------------
#define DIANJI_BOOT_SAFE_DELAY_MS         (20U)     // power-on safe hold
#define DIANJI_ENABLE_GUARD_DELAY_MS      (10U)     // wait before enabling driver

// ---------------- XL9555 pin mapping ----------------
// According to provided schematic:
// RUNLED/ENABLE are indicator LED nets on the DRV8701 board.
#define DIANJI_XL9555_RUNLED_PIN          (XL9555_PIN_IO1_0)
#define DIANJI_XL9555_ENABLE_PIN          (XL9555_PIN_IO1_1)

// ---------------- XL9555 logic level ----------------
// Current board uses these as LED control nets. Drive high when enabled/running.
#define DIANJI_RUNLED_ON_LEVEL            (1U)
#define DIANJI_RUNLED_OFF_LEVEL           (0U)

#define DIANJI_ENABLE_ACTIVE_LEVEL        (1U)
#define DIANJI_ENABLE_INACTIVE_LEVEL      (0U)

#endif
