#include "ekf.h"
#include "matrix.h"
#include "zf_device_imu963ra.h"

float ekf_legacy_imu_pitch = 0;
float ekf_legacy_imu_roll  = 0;
float ekf_legacy_imu_yaw   = 0;
float ekf_legacy_imu_gyro_z = 0;

static matrix_t ekf_legacy_exf_x;
static matrix_t ekf_legacy_error;
static EulerAngles ekf_legacy_euler_angle;
ekf_imu_t ekf_legacy_imu_data = {0, 0, 0, 0, 0, 0};
static matrix_type ekf_legacy_r_yz = 0.001f;

uint8_t ekf_legacy_imu_receive_flag = 0;
static int16 imu963ra_acc_x_l = 0;
static int16 imu963ra_acc_y_l = 0;
static int16 imu963ra_acc_z_l = 0;

const matrix_type q[4][4] = {{0.005, 0, 0, 0}, {0, 0.005, 0, 0}, {0, 0, 0.005, 0}, {0, 0, 0, 0.005}};
const matrix_type r[3][3] = {{10000, 0, 0}, {0, 10000, 0}, {0, 0, 10000}};
const matrix_type p[4][4] = {{1000000, 0, 0, 0}, {0, 1000000, 0, 0}, {0, 0, 1000000, 0}, {0, 0, 0, 1000000}};
const matrix_type ekf[4] = {1, 0, 0, 0};

static matrix_t Q;
static matrix_t R;
static matrix_t P;

void EKF_LEGACY_Init(void)
{
    Matrix_From_Array(&ekf_legacy_exf_x, (const matrix_type*)ekf, 4, 1);
    Matrix_From_Array(&Q, (const matrix_type*)q, 4, 4);
    Matrix_From_Array(&R, (const matrix_type*)r, 3, 3);
    Matrix_From_Array(&P, (const matrix_type*)p, 4, 4);

    ekf_legacy_imu_pitch = 0.0f;
    ekf_legacy_imu_roll = 0.0f;
    ekf_legacy_imu_yaw = 0.0f;
    ekf_legacy_imu_gyro_z = 0.0f;
    ekf_legacy_imu_data.gyro_x = 0.0f;
    ekf_legacy_imu_data.gyro_y = 0.0f;
    ekf_legacy_imu_data.gyro_z = 0.0f;
    ekf_legacy_imu_data.acc_x = 0.0f;
    ekf_legacy_imu_data.acc_y = 0.0f;
    ekf_legacy_imu_data.acc_z = 0.0f;
    ekf_legacy_imu_receive_flag = 0U;
    imu963ra_acc_x_l = 0;
    imu963ra_acc_y_l = 0;
    imu963ra_acc_z_l = 0;
}

void EKF_LEGACY_GetQuaternion(float q_out[4])
{
    if(q_out != 0)
    {
        q_out[0] = ekf_legacy_exf_x.data[0][0];
        q_out[1] = ekf_legacy_exf_x.data[1][0];
        q_out[2] = ekf_legacy_exf_x.data[2][0];
        q_out[3] = ekf_legacy_exf_x.data[3][0];
    }
}
static inline void quaternion_to_euler(void)
{
    float q0 = (ekf_legacy_exf_x.data[0][0]);
    float q1 = (ekf_legacy_exf_x.data[1][0]);
    float q2 = (ekf_legacy_exf_x.data[2][0]);
    float q3 = (ekf_legacy_exf_x.data[3][0]);

    ekf_legacy_imu_pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * DEG_TO_RAD;
    ekf_legacy_imu_roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * DEG_TO_RAD;
    ekf_legacy_imu_yaw   = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * DEG_TO_RAD;

    ekf_legacy_euler_angle.pitch = ekf_legacy_imu_pitch;
    ekf_legacy_euler_angle.roll = ekf_legacy_imu_roll;
    ekf_legacy_euler_angle.yaw = ekf_legacy_imu_yaw;
}


uint8_t EKF_LEGACY_GetValues(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();

    ekf_legacy_imu_data.acc_x = K * imu963ra_acc_x + (1 - K) * imu963ra_acc_x_l;
    ekf_legacy_imu_data.acc_y = K * imu963ra_acc_y + (1 - K) * imu963ra_acc_y_l;
    ekf_legacy_imu_data.acc_z = K * imu963ra_acc_z + (1 - K) * imu963ra_acc_z_l;
    imu963ra_acc_x_l = ekf_legacy_imu_data.acc_x;
    imu963ra_acc_y_l = ekf_legacy_imu_data.acc_y;
    imu963ra_acc_z_l = ekf_legacy_imu_data.acc_z;

    ekf_legacy_imu_data.gyro_x = ((float)imu963ra_gyro_x / 14.3f) * PI / 180.0f;
    ekf_legacy_imu_data.gyro_y = ((float)imu963ra_gyro_y / 14.3f) * PI / 180.0f;
    ekf_legacy_imu_data.gyro_z = ((float)imu963ra_gyro_z / 14.3f) * PI / 180.0f;
    if(ekf_legacy_imu_data.gyro_x < 0.01f && ekf_legacy_imu_data.gyro_x > -0.01f) ekf_legacy_imu_data.gyro_x = 0;
    if(ekf_legacy_imu_data.gyro_y < 0.01f && ekf_legacy_imu_data.gyro_y > -0.01f) ekf_legacy_imu_data.gyro_y = 0;
    if(ekf_legacy_imu_data.gyro_z < 0.01f && ekf_legacy_imu_data.gyro_z > -0.01f) ekf_legacy_imu_data.gyro_z = 0;

    ekf_legacy_imu_gyro_z = (float)imu963ra_gyro_z / 14.3f;
    if(ekf_legacy_imu_gyro_z < 1.0f && ekf_legacy_imu_gyro_z > -1.0f) ekf_legacy_imu_gyro_z = 0;

    ekf_legacy_imu_receive_flag = 1;
    return 1;
}

void EKF_LEGACY_UpData(void)
{
    float gx, gy, gz;
    gx = ekf_legacy_imu_data.gyro_x;
    gy = ekf_legacy_imu_data.gyro_y;
    gz = ekf_legacy_imu_data.gyro_z;

    matrix_t Z;
    Matrix_Init(&Z, 3, 1);

    Z.data[0][0] = (matrix_type)ekf_legacy_imu_data.acc_x;
    Z.data[1][0] = (matrix_type)ekf_legacy_imu_data.acc_y;
    Z.data[2][0] = (matrix_type)ekf_legacy_imu_data.acc_z;

    normalize_vector(&Z);

    matrix_type f[4][4]= {{1, -0.5f * gx * dt, -0.5f * gy * dt, -0.5f * gz * dt},
                          {0.5f * gx * dt, 1, 0.5f * gz * dt, -0.5f * gy * dt},
                          {0.5f * gy * dt, -0.5f * gz * dt, 1, 0.5f * gx * dt},
                          {0.5f * gz * dt, 0.5f * gy * dt, -0.5f * gx * dt, 1}};

    matrix_t F, FT;
    Matrix_From_Array(&F, (const matrix_type*)f, 4, 4);
    FT = Matrix_Transpose(&F);

    ekf_legacy_exf_x = multiply_matrices(&F, &ekf_legacy_exf_x);
    normalize_vector(&ekf_legacy_exf_x);

    {
        float q0 = (ekf_legacy_exf_x.data[0][0]);
        float q1 = (ekf_legacy_exf_x.data[1][0]);
        float q2 = (ekf_legacy_exf_x.data[2][0]);
        float q3 = (ekf_legacy_exf_x.data[3][0]);

        matrix_type h[3][4]={{-2 * q2, 2 * q3, -2 * q0, 2 * q1},
                             {2 * q1, 2 * q0, 2 * q3, 2 * q2},
                             {2 * q0, -2 * q1, -2 * q2, 2 * q3}};

        matrix_t H, HT;
        Matrix_From_Array(&H, (const matrix_type*)h, 3, 4);
        HT = Matrix_Transpose(&H);

        {
            matrix_t PK_;
            PK_ = multiply_matrices(&F, &P);
            PK_ = multiply_matrices(&PK_, &FT);
            P = add_matrices(&PK_, &Q);
        }

        {
            matrix_t DK, invDK;
            DK = multiply_matrices(&H, &P);
            DK = multiply_matrices(&DK, &HT);
            DK = add_matrices(&DK, &R);

            if(inverse_matrix(&DK, &invDK))
            {
                quaternion_to_euler();
                return;
            }

            {
                matrix_t EK, EKT;
                EK = multiply_matrices(&H, &ekf_legacy_exf_x);
                EK = subtract_matrices(&Z, &EK);
                EKT = Matrix_Transpose(&EK);

                ekf_legacy_error = multiply_matrices(&EKT, &invDK);
                ekf_legacy_error = multiply_matrices(&ekf_legacy_error, &EK);

                if(ekf_legacy_error.data[0][0] > ekf_legacy_r_yz)
                {
                    quaternion_to_euler();
                    return;
                }

                {
                    matrix_t Kk;
                    matrix_t temp;
                    matrix_t I;

                    Kk = multiply_matrices(&P, &HT);
                    Kk = multiply_matrices(&Kk, &invDK);

                    temp = multiply_matrices(&Kk, &EK);
                    ekf_legacy_exf_x = add_matrices(&ekf_legacy_exf_x, &temp);
                    normalize_vector(&ekf_legacy_exf_x);

                    Matrix_Identity(&I, 4);
                    temp = multiply_matrices(&Kk, &H);
                    temp = subtract_matrices(&I, &temp);
                    P = multiply_matrices(&temp, &P);
                }
            }
        }
    }

    quaternion_to_euler();
}
