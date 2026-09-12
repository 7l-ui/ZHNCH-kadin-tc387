#ifndef CODE_DICI_H_
#define CODE_DICI_H_

#include "zf_common_headfile.h"

#define DEG_TO_RAD      (57.295779513082320876798154814105f)
#define dt              (0.001f)
#define IMU_AHRS_DEFAULT_DT    (dt)
#define K               (0.9f)

#define IMU_SENSOR_USE_963RA          (1U)

#define IMU_AHRS_MODE_6AXIS    (0U)
#define IMU_AHRS_MODE_9AXIS    (1U)

#ifndef IMU_AHRS_MODE
#define IMU_AHRS_MODE          IMU_AHRS_MODE_6AXIS
#endif

#if ((IMU_AHRS_MODE != IMU_AHRS_MODE_6AXIS) && (IMU_AHRS_MODE != IMU_AHRS_MODE_9AXIS))
#error "IMU_AHRS_MODE must be IMU_AHRS_MODE_6AXIS or IMU_AHRS_MODE_9AXIS"
#endif

#define IMU_AHRS_USE_MAG       (IMU_AHRS_MODE == IMU_AHRS_MODE_9AXIS)

#define IMU_FUSION_ALGO_MAHONY       (0U)
#define IMU_FUSION_ALGO_LEGACY_EKF   (1U)
#ifndef IMU_FUSION_ALGO
#define IMU_FUSION_ALGO              IMU_FUSION_ALGO_MAHONY
#endif

#if ((IMU_FUSION_ALGO != IMU_FUSION_ALGO_MAHONY) && (IMU_FUSION_ALGO != IMU_FUSION_ALGO_LEGACY_EKF))
#error "IMU_FUSION_ALGO must be IMU_FUSION_ALGO_MAHONY or IMU_FUSION_ALGO_LEGACY_EKF"
#endif

#if (IMU_FUSION_ALGO == IMU_FUSION_ALGO_MAHONY)
#define IMU_FUSION_ALGO_SHADOW       IMU_FUSION_ALGO_LEGACY_EKF
#else
#define IMU_FUSION_ALGO_SHADOW       IMU_FUSION_ALGO_MAHONY
#endif

#ifndef IMU_TASK1_COMPARE_ENABLE_IN_RUN
#define IMU_TASK1_COMPARE_ENABLE_IN_RUN      (0U)
#endif

#ifndef IMU_TASK1_COMPARE_ENABLE_IN_IMU_PAGE
#define IMU_TASK1_COMPARE_ENABLE_IN_IMU_PAGE (1U)
#endif

typedef struct
{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
        float mag_x;
        float mag_y;
        float mag_z;
}imu_t;

typedef struct
{
        uint8_t receive_flag;
        uint8_t filter_initialized;
        uint8_t dt_initialized;
        uint8_t attitude_initialized;
        uint8_t ref_locked;
        uint8_t ref_capture_request;
        uint8_t mag_enabled;
        uint8_t fusion_algo;
        uint8_t compare_enabled;
        uint8_t compare_valid;
        uint8_t compare_shadow_algo;
        uint8_t gyro_bias_ready;
        uint16_t gyro_bias_sample_count;
        uint16_t gyro_bias_sample_target;
        float sample_dt_s;
        float sample_dt_cfg_s;
        float two_kp;
        float two_ki;
        float integral_fb_x;
        float integral_fb_y;
        float integral_fb_z;
        float accel_confidence;
        float mag_confidence;
        float acc_norm_g;
        float mag_norm;
        float mag_norm_ref;
        float gyro_x_dps;
        float gyro_y_dps;
        float gyro_z_dps;
        float gyro_norm_dps;
        float gyro_bias_x_dps;
        float gyro_bias_y_dps;
        float gyro_bias_z_dps;
        float q0;
        float q1;
        float q2;
        float q3;
        float q_ref0;
        float q_ref1;
        float q_ref2;
        float q_ref3;
        float ref_elapsed_s;
        float ref_stable_elapsed_s;
        float cmp_main_pitch;
        float cmp_main_roll;
        float cmp_main_yaw;
        float cmp_shadow_pitch;
        float cmp_shadow_roll;
        float cmp_shadow_yaw;
        float shadow_abs_pitch;
        float shadow_abs_roll;
        float shadow_abs_yaw;
        float cmp_delta_pitch;
        float cmp_delta_roll;
        float cmp_delta_yaw;
} imu_ahrs_debug_t;

extern volatile imu_t imu_data;
extern volatile uint8_t IMU_Receive_flag;
extern volatile float IMU_Pitch;
extern volatile float IMU_Roll;
extern volatile float IMU_Yaw;
extern volatile float IMU_RelPitch;
extern volatile float IMU_RelRoll;
extern volatile float IMU_RelYaw;
extern volatile float IMU_GYRO_Z;
extern volatile float IMU_MagRawX;
extern volatile float IMU_MagRawY;
extern volatile float IMU_MagRawZ;

uint8_t imu_get_values(void);
uint8_t IMU_Sensor_Init(void);


void EKF_Init(void);
void EKF_UpData(void);

void IMU_AHRS_SetSamplePeriod(float dt_s);
void IMU_AHRS_SetGains(float kp, float ki);
void IMU_AHRS_SetMagEnable(uint8_t enable);
void IMU_AHRS_CaptureReference(void);
uint8_t IMU_AHRS_IsReferenceReady(void);
void IMU_AHRS_GetDebugInfo(imu_ahrs_debug_t *out);
void IMU_AHRS_SetCompareEnable(uint8_t enable);
void IMU_AHRS_SetMagCalibration(const float offset[3], const float scale[3]);
void IMU_AHRS_SetMagCalibrationMatrix(const float offset[3], const float softiron[3][3]);
void IMU_AHRS_Reset(void);


#endif /* CODE_DICI_H_ */
