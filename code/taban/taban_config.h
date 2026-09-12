#ifndef _TABAN_CONFIG_H_
#define _TABAN_CONFIG_H_

#include "zf_common_headfile.h"

// Two analog pedal inputs.
// Note: In this SDK enum, A16/A17 belong to ADC2 group.
#define TABAN0_ADC_CHANNEL                 (ADC2_CH0_A16)
#define TABAN1_ADC_CHANNEL                 (ADC2_CH1_A17)
#define TABAN_ADC_RESOLUTION               (ADC_12BIT)

// Sampling and filtering.
#define TABAN_SAMPLE_COUNT                 (8U)
#define TABAN_FILTER_DIV                   (4U)

// Calibration window.
// Adjust these two values after measuring the real pedal.
#define TABAN0_ADC_MIN                     (1022U)
#define TABAN0_ADC_MAX                     (3200U)
#define TABAN1_ADC_MIN                     (1022U)
#define TABAN1_ADC_MAX                     (3200U)

// Deadzones near the two ends of travel.
#define TABAN0_ADC_DEADZONE_LOW            (40U)
#define TABAN0_ADC_DEADZONE_HIGH           (20U)
#define TABAN1_ADC_DEADZONE_LOW            (40U)
#define TABAN1_ADC_DEADZONE_HIGH           (20U)

// Startup safety:
// pedal must be released once after power-on before output becomes valid.
#define TABAN0_STARTUP_RELEASE_ADC         (1150U)
#define TABAN1_STARTUP_RELEASE_ADC         (1150U)

// Sensor fault window.
#define TABAN0_ADC_FAULT_LOW               (50U)
#define TABAN0_ADC_FAULT_HIGH              (4050U)
#define TABAN1_ADC_FAULT_LOW               (50U)
#define TABAN1_ADC_FAULT_HIGH              (4050U)

// Target speed generation:
// pedal0 -> throttle, pedal1 -> brake.
// target_speed is always [0, TABAN_TARGET_SPEED_MAX_MPS]:
// speed_permille = accel_curve(accel) * (1000 - brake_curve(brake)) / 1000
// target_speed_mps = speed_permille / 1000 * TABAN_TARGET_SPEED_MAX_MPS
#define TABAN_TARGET_SPEED_MAX_MPS         (2.00f)

// Pedal feel shaping (0~1000):
// 0 -> linear, 1000 -> fully use shaped curve.
// accel curve uses x^2 (softer low-end), brake curve uses x*(2-x) (stronger early bite).
#define TABAN_ACCEL_CURVE_SOFTEN_PERMILLE  (350U)
#define TABAN_BRAKE_CURVE_ENHANCE_PERMILLE (250U)

// Brake freeplay in pedal permille. Below this, brake has no effect.
#define TABAN_BRAKE_FREEPLAY_PERMILLE      (90U)
// Brake saturation point in pedal permille. From this point to 1000, treat as full brake.
#define TABAN_BRAKE_FULL_INPUT_PERMILLE    (800U)

// Brake priority threshold. If brake curve exceeds this value, throttle is fully cut.
#define TABAN_BRAKE_OVERRIDE_PERMILLE      (900U)

#endif
