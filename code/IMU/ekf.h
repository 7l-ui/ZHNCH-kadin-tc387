#ifndef CODE_EKF_H_
#define CODE_EKF_H_

#include "zf_common_headfile.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD      (57.295779513082320876798154814105f)
#endif
#ifndef dt
#define dt              (0.001f)
#endif
#ifndef K
#define K               (0.9f)
#endif


typedef struct
{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
} ekf_imu_t;

extern ekf_imu_t ekf_legacy_imu_data;
extern uint8_t ekf_legacy_imu_receive_flag;
extern float ekf_legacy_imu_pitch;
extern float ekf_legacy_imu_roll;
extern float ekf_legacy_imu_yaw;
extern float ekf_legacy_imu_gyro_z;

uint8_t EKF_LEGACY_GetValues(void);

void EKF_LEGACY_Init(void);
void EKF_LEGACY_UpData(void);
void EKF_LEGACY_GetQuaternion(float q_out[4]);


#endif /* CODE_EKF_H_ */
