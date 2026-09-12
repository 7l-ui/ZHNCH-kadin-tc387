#include "dici.h"
#include "matrix.h"
#include "zf_device_imu963ra.h"
#include "ekf.h"
#include <math.h>

volatile float IMU_Pitch = 0.0f;
volatile float IMU_Roll  = 0.0f;
volatile float IMU_Yaw   = 0.0f;
volatile float IMU_RelPitch = 0.0f;
volatile float IMU_RelRoll  = 0.0f;
volatile float IMU_RelYaw   = 0.0f;
volatile float IMU_GYRO_Z = 0.0f;
volatile float IMU_MagRawX = 0.0f;
volatile float IMU_MagRawY = 0.0f;
volatile float IMU_MagRawZ = 0.0f;

matrix_t exf_x;
matrix_t error;
EulerAngles euler_angle;
volatile imu_t imu_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
matrix_type r_yz = 0.001f;

volatile uint8_t IMU_Receive_flag = 0;

static float ahrs_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float ahrs_q_ref[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float twoKp = 1.2f;
static float twoKi = 0.02f;
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;
static float sample_dt_cfg = IMU_AHRS_DEFAULT_DT;
static float sample_dt = IMU_AHRS_DEFAULT_DT;

static float mag_offset[3] = {0.0f, 0.0f, 0.0f};
static float mag_softiron[3][3] =
{
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static uint8_t imu_filter_init = 0;
static float acc_lp[3] = {0.0f, 0.0f, 0.0f};
static float mag_lp[3] = {0.0f, 0.0f, 0.0f};
static float accel_confidence = 1.0f;
static float mag_confidence = 1.0f;
static float mag_norm_ref = 0.0f;
static uint8_t mag_ref_init = 0;
static uint8_t dt_tick_init = 0;
static uint32 dt_last_tick_10ns = 0;
static uint8_t ahrs_attitude_initialized = 0U;
static uint8_t ahrs_ref_locked = 0;
static float ahrs_ref_elapsed_s = 0.0f;
static float ahrs_ref_stable_elapsed_s = 0.0f;
static volatile uint8_t ahrs_ref_capture_request = 0;
static uint8_t ahrs_mag_enable = 0;
static uint8_t gyro_bias_ready = 0;
static uint16_t gyro_bias_sample_count = 0;
static float gyro_bias_sum_dps[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};
static float ahrs_last_abs_pitch_deg = 0.0f;
static float ahrs_last_abs_roll_deg = 0.0f;
static float ahrs_last_abs_yaw_deg = 0.0f;
static uint8_t ahrs_last_abs_att_valid = 0U;

typedef struct
{
    uint8_t enabled;
    uint8_t valid;
    uint8_t shadow_attitude_initialized;
    float shadow_q[4];
    float shadow_integral[3];
    float main_ref_q[4];
    float shadow_ref_q[4];
    float main_rel_pitch;
    float main_rel_roll;
    float main_rel_yaw;
    float shadow_rel_pitch;
    float shadow_rel_roll;
    float shadow_rel_yaw;
    float shadow_abs_pitch;
    float shadow_abs_roll;
    float shadow_abs_yaw;
    float delta_pitch;
    float delta_roll;
    float delta_yaw;
} imu_compare_state_t;

static imu_compare_state_t imu_compare =
{
    0U, 0U, 0U,
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 0.0f},
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f
};

#define IMU_DT_MIN_S            (0.0001f)
#define IMU_DT_MAX_S            (0.02f)
#define ACC_CONF_FULL_ERR_G     (0.08f)
#define ACC_CONF_ZERO_ERR_G     (0.30f)
#define ACC_CONF_RISE_ALPHA     (0.08f)
#define ACC_CONF_FALL_ALPHA     (0.25f)
#define MAG_CONF_FULL_REL_ERR   (0.12f)
#define MAG_CONF_ZERO_REL_ERR   (0.45f)
#define AHRS_INIT_ACC_MIN_G         (0.50f)
#define AHRS_INIT_ACC_MAX_G         (1.50f)
#define AHRS_INIT_MAG_MIN_NORM      (1e-6f)
#define AHRS_REF_STABLE_GYRO_DPS    (2.5f)
#define AHRS_REF_STABLE_ATT_RATE_DPS (3.0f)
#define AHRS_REF_STABLE_MAG_CONF    (0.35f)
#define AHRS_REF_STABLE_ACC_CONF    (0.50f)
#define AHRS_REF_STABLE_HOLD_S      (0.6f)
#define AHRS_MAG_CORRECT_AFTER_REF_LOCK (0U)
#define GYRO_BIAS_SAMPLE_COUNT      (1000U)
#define GYRO_BIAS_MAX_STATIC_DPS    (20.0f)
#define GYRO_INPUT_DEADBAND_DPS     (0.20f)
#define IMU_BAD_FRAME_ACC_NORM_HIGH_G   (1.70f)
#define IMU_BAD_FRAME_ACC_NORM_LOW_G    (0.20f)
#define IMU_BAD_FRAME_GYRO_NORM_DPS     (100.0f)

static inline float imu_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline float imu_clip(float x, float low, float high)
{
    if(x < low)
    {
        return low;
    }
    if(x > high)
    {
        return high;
    }
    return x;
}

static inline float imu_wrap_deg_pm180(float deg)
{
    while(deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while(deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static inline float imu_safe_inv_norm3(float x, float y, float z)
{
    float n2 = x * x + y * y + z * z;
    if(n2 <= 1e-12f)
    {
        return 0.0f;
    }
    return invSqrt(n2);
}

static inline float imu_apply_confidence(float err, float confidence)
{
    return err * imu_clip(confidence, 0.0f, 1.0f);
}

static uint8_t imu_raw_frame_is_bad(int16 ax_raw, int16 ay_raw, int16 az_raw,
                                    int16 gx_raw, int16 gy_raw, int16 gz_raw)
{
    uint8_t acc_same = ((ax_raw == ay_raw) && (ay_raw == az_raw)) ? 1U : 0U;
    uint8_t gyro_same = ((gx_raw == gy_raw) && (gy_raw == gz_raw)) ? 1U : 0U;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float acc_norm;
    float gyro_norm;

    if((ax_raw == 0) && (ay_raw == 0) && (az_raw == 0))
    {
        return 1U;
    }

    if((0U == acc_same) && (0U == gyro_same))
    {
        return 0U;
    }

    ax_g = imu963ra_acc_transition(ax_raw);
    ay_g = imu963ra_acc_transition(ay_raw);
    az_g = imu963ra_acc_transition(az_raw);
    acc_norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    if((0U != acc_same) &&
       ((acc_norm > IMU_BAD_FRAME_ACC_NORM_HIGH_G) ||
        (acc_norm < IMU_BAD_FRAME_ACC_NORM_LOW_G)))
    {
        return 1U;
    }

    if(0U != gyro_same)
    {
        gx_dps = imu963ra_gyro_transition(gx_raw);
        gy_dps = imu963ra_gyro_transition(gy_raw);
        gz_dps = imu963ra_gyro_transition(gz_raw);
        gyro_norm = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);

        if((0U != acc_same) && (gyro_norm > IMU_BAD_FRAME_GYRO_NORM_DPS))
        {
            return 1U;
        }
    }

    return 0U;
}

static inline void quaternion_to_euler(void);

static void imu_zero_relative_euler(void)
{
    IMU_RelPitch = 0.0f;
    IMU_RelRoll = 0.0f;
    IMU_RelYaw = 0.0f;
}

static void imu_reset_gyro_bias(void)
{
    gyro_bias_ready = 0;
    gyro_bias_sample_count = 0;
    gyro_bias_sum_dps[0] = 0.0f;
    gyro_bias_sum_dps[1] = 0.0f;
    gyro_bias_sum_dps[2] = 0.0f;
    gyro_bias_dps[0] = 0.0f;
    gyro_bias_dps[1] = 0.0f;
    gyro_bias_dps[2] = 0.0f;
}

static void imu_capture_reference_now(void)
{
    ahrs_q_ref[0] = ahrs_q[0];
    ahrs_q_ref[1] = ahrs_q[1];
    ahrs_q_ref[2] = ahrs_q[2];
    ahrs_q_ref[3] = ahrs_q[3];
    ahrs_ref_locked = 1;
    ahrs_ref_elapsed_s = 0.0f;
    ahrs_ref_stable_elapsed_s = 0.0f;
    imu_zero_relative_euler();
}

static uint8_t imu_make_initial_quaternion(float q_out[4])
{
    float ax = imu_data.acc_x;
    float ay = imu_data.acc_y;
    float az = imu_data.acc_z;
    float acc_norm;
    float pitch_rad;
    float roll_rad;
    float yaw_rad;
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;
    float q0;
    float q1;
    float q2;
    float q3;
    float recipNorm;
#if IMU_AHRS_USE_MAG
    float mx = imu_data.mag_x;
    float my = imu_data.mag_y;
    float mz = imu_data.mag_z;
    float mag_norm;
    float xh;
    float yh;
#endif

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if((acc_norm < AHRS_INIT_ACC_MIN_G) || (acc_norm > AHRS_INIT_ACC_MAX_G))
    {
        return 0U;
    }

#if IMU_AHRS_USE_MAG
    mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    if((ahrs_mag_enable != 0U) && (mag_norm <= AHRS_INIT_MAG_MIN_NORM))
    {
        return 0U;
    }
#else
    yaw_rad = 0.0f;
#endif

    roll_rad = atan2f(ay, az);
    pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));

#if IMU_AHRS_USE_MAG
    if(ahrs_mag_enable != 0U)
    {
        xh = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
        yh = mx * sinf(roll_rad) * sinf(pitch_rad) +
             my * cosf(roll_rad) -
             mz * sinf(roll_rad) * cosf(pitch_rad);
        yaw_rad = atan2f(yh, xh);
    }
    else
    {
        yaw_rad = 0.0f;
    }
#else
    yaw_rad = 0.0f;
#endif

    cr = cosf(roll_rad * 0.5f);
    sr = sinf(roll_rad * 0.5f);
    cp = cosf(pitch_rad * 0.5f);
    sp = sinf(pitch_rad * 0.5f);
    cy = cosf((-yaw_rad) * 0.5f);
    sy = sinf((-yaw_rad) * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    {
        float qn = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        if(qn <= 1e-12f)
        {
            return 0U;
        }
        recipNorm = invSqrt(qn);
    }

    if(q_out == NULL)
    {
        return 0U;
    }

    q_out[0] = q0 * recipNorm;
    q_out[1] = q1 * recipNorm;
    q_out[2] = q2 * recipNorm;
    q_out[3] = q3 * recipNorm;

    return 1U;
}

static uint8_t imu_init_attitude_from_sensors(void)
{
    if(imu_make_initial_quaternion(ahrs_q) == 0U)
    {
        return 0U;
    }

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    quaternion_to_euler();
    ahrs_last_abs_pitch_deg = IMU_Pitch;
    ahrs_last_abs_roll_deg = IMU_Roll;
    ahrs_last_abs_yaw_deg = IMU_Yaw;
    ahrs_last_abs_att_valid = 1U;
    ahrs_attitude_initialized = 1U;
    return 1U;
}

static uint8_t imu_is_reference_stable(void)
{
    float gyro_norm_dps;
    float pitch_delta_deg;
    float roll_delta_deg;
    float yaw_delta_deg;
    float att_rate_dps;

    gyro_norm_dps = sqrtf((gyro_bias_ready ? (imu_data.gyro_x * DEG_TO_RAD) : 0.0f) * (gyro_bias_ready ? (imu_data.gyro_x * DEG_TO_RAD) : 0.0f) +
                          (gyro_bias_ready ? (imu_data.gyro_y * DEG_TO_RAD) : 0.0f) * (gyro_bias_ready ? (imu_data.gyro_y * DEG_TO_RAD) : 0.0f) +
                          (gyro_bias_ready ? (imu_data.gyro_z * DEG_TO_RAD) : 0.0f) * (gyro_bias_ready ? (imu_data.gyro_z * DEG_TO_RAD) : 0.0f));

    if(!ahrs_last_abs_att_valid || (sample_dt <= 1e-6f))
    {
        ahrs_last_abs_pitch_deg = IMU_Pitch;
        ahrs_last_abs_roll_deg = IMU_Roll;
        ahrs_last_abs_yaw_deg = IMU_Yaw;
        ahrs_last_abs_att_valid = 1U;
        return 0U;
    }

    pitch_delta_deg = imu_wrap_deg_pm180(IMU_Pitch - ahrs_last_abs_pitch_deg);
    roll_delta_deg = imu_wrap_deg_pm180(IMU_Roll - ahrs_last_abs_roll_deg);
    yaw_delta_deg = imu_wrap_deg_pm180(IMU_Yaw - ahrs_last_abs_yaw_deg);
    att_rate_dps = imu_absf(pitch_delta_deg) / sample_dt;
    if((imu_absf(roll_delta_deg) / sample_dt) > att_rate_dps)
    {
        att_rate_dps = imu_absf(roll_delta_deg) / sample_dt;
    }
    if((imu_absf(yaw_delta_deg) / sample_dt) > att_rate_dps)
    {
        att_rate_dps = imu_absf(yaw_delta_deg) / sample_dt;
    }

    ahrs_last_abs_pitch_deg = IMU_Pitch;
    ahrs_last_abs_roll_deg = IMU_Roll;
    ahrs_last_abs_yaw_deg = IMU_Yaw;

    if(gyro_norm_dps > AHRS_REF_STABLE_GYRO_DPS)
    {
        return 0U;
    }

    if(att_rate_dps > AHRS_REF_STABLE_ATT_RATE_DPS)
    {
        return 0U;
    }

    if(accel_confidence < AHRS_REF_STABLE_ACC_CONF)
    {
        return 0U;
    }

#if IMU_AHRS_USE_MAG
    if((ahrs_mag_enable != 0U) && (mag_confidence < AHRS_REF_STABLE_MAG_CONF))
    {
        return 0U;
    }
#endif

    return 1U;
}

static void imu_update_gyro_bias(float gx_dps, float gy_dps, float gz_dps)
{
    if(gyro_bias_ready)
    {
        return;
    }

    if((imu_absf(gx_dps) > GYRO_BIAS_MAX_STATIC_DPS) ||
       (imu_absf(gy_dps) > GYRO_BIAS_MAX_STATIC_DPS) ||
       (imu_absf(gz_dps) > GYRO_BIAS_MAX_STATIC_DPS))
    {
        gyro_bias_sample_count = 0;
        gyro_bias_sum_dps[0] = 0.0f;
        gyro_bias_sum_dps[1] = 0.0f;
        gyro_bias_sum_dps[2] = 0.0f;
        return;
    }

    gyro_bias_sum_dps[0] += gx_dps;
    gyro_bias_sum_dps[1] += gy_dps;
    gyro_bias_sum_dps[2] += gz_dps;
    gyro_bias_sample_count++;

    if(gyro_bias_sample_count >= GYRO_BIAS_SAMPLE_COUNT)
    {
        float inv_count = 1.0f / (float)gyro_bias_sample_count;

        gyro_bias_dps[0] = gyro_bias_sum_dps[0] * inv_count;
        gyro_bias_dps[1] = gyro_bias_sum_dps[1] * inv_count;
        gyro_bias_dps[2] = gyro_bias_sum_dps[2] * inv_count;
        gyro_bias_ready = 1;
    }
}

static float imu_compute_acc_confidence(float ax, float ay, float az)
{
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    float err = imu_absf(acc_norm - 1.0f);

    if(err <= ACC_CONF_FULL_ERR_G)
    {
        return 1.0f;
    }
    if(err >= ACC_CONF_ZERO_ERR_G)
    {
        return 0.0f;
    }

    return 1.0f - ((err - ACC_CONF_FULL_ERR_G) / (ACC_CONF_ZERO_ERR_G - ACC_CONF_FULL_ERR_G));
}

static float imu_compute_mag_confidence(float mx, float my, float mz)
{
    float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    float rel_err;

    if(mag_norm <= 1e-6f)
    {
        return 0.0f;
    }

    if(!mag_ref_init)
    {
        mag_norm_ref = mag_norm;
        mag_ref_init = 1;
    }
    else
    {
        mag_norm_ref = 0.995f * mag_norm_ref + 0.005f * mag_norm;
    }

    if(mag_norm_ref <= 1e-6f)
    {
        return 0.0f;
    }

    rel_err = imu_absf(mag_norm - mag_norm_ref) / mag_norm_ref;

    if(rel_err <= MAG_CONF_FULL_REL_ERR)
    {
        return 1.0f;
    }
    if(rel_err >= MAG_CONF_ZERO_REL_ERR)
    {
        return 0.0f;
    }

    return 1.0f - ((rel_err - MAG_CONF_FULL_REL_ERR) / (MAG_CONF_ZERO_REL_ERR - MAG_CONF_FULL_REL_ERR));
}

static void imu_update_dynamic_dt(void)
{
    uint32 now_tick_10ns;
    uint32 delta_tick_10ns;
    float dt_now;

    if(!dt_tick_init)
    {
        now_tick_10ns = system_getval();
        dt_last_tick_10ns = now_tick_10ns;
        dt_tick_init = 1;
        sample_dt = sample_dt_cfg;
        return;
    }

    now_tick_10ns = system_getval();
    delta_tick_10ns = now_tick_10ns - dt_last_tick_10ns;
    dt_last_tick_10ns = now_tick_10ns;

    if(delta_tick_10ns == 0U)
    {
        sample_dt = sample_dt_cfg;
        return;
    }

    dt_now = (float)delta_tick_10ns * 1e-8f;
    if((dt_now >= IMU_DT_MIN_S) && (dt_now <= IMU_DT_MAX_S))
    {
        sample_dt = dt_now;
    }
    else
    {
        sample_dt = sample_dt_cfg;
    }
}

static void imu_quaternion_to_euler_values(float q0, float q1, float q2, float q3,
                                           float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    float sinp;

    sinp = -2.0f * q1 * q3 + 2.0f * q0 * q2;
    sinp = imu_clip(sinp, -1.0f, 1.0f);

    if(pitch_deg != NULL)
    {
        *pitch_deg = asinf(sinp) * DEG_TO_RAD;
    }
    if(roll_deg != NULL)
    {
        *roll_deg = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                           -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * DEG_TO_RAD;
    }
    if(yaw_deg != NULL)
    {
        *yaw_deg = -atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                           -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * DEG_TO_RAD;
    }
}

static void quaternion_to_euler_from(float q0, float q1, float q2, float q3)
{
    float pitch;
    float roll;
    float yaw;

    imu_quaternion_to_euler_values(q0, q1, q2, q3, &pitch, &roll, &yaw);

    IMU_Pitch = pitch;
    IMU_Roll = roll;
    IMU_Yaw = yaw;

    euler_angle.pitch = IMU_Pitch;
    euler_angle.roll = IMU_Roll;
    euler_angle.yaw = IMU_Yaw;
}

static inline void quaternion_to_euler(void)
{
    quaternion_to_euler_from(ahrs_q[0], ahrs_q[1], ahrs_q[2], ahrs_q[3]);
}

static void imu_quaternion_relative_euler_values(const float ref_q[4], const float cur_q[4],
                                                 float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    float ref_q0 = ref_q[0];
    float ref_q1 = ref_q[1];
    float ref_q2 = ref_q[2];
    float ref_q3 = ref_q[3];
    float cur_q0 = cur_q[0];
    float cur_q1 = cur_q[1];
    float cur_q2 = cur_q[2];
    float cur_q3 = cur_q[3];
    float rel_q0;
    float rel_q1;
    float rel_q2;
    float rel_q3;
    float n;
    float recipNorm;

    /* Relative attitude: q_rel = conj(q_ref) * q_now */
    rel_q0 = ref_q0 * cur_q0 + ref_q1 * cur_q1 + ref_q2 * cur_q2 + ref_q3 * cur_q3;
    rel_q1 = ref_q0 * cur_q1 - ref_q1 * cur_q0 - ref_q2 * cur_q3 + ref_q3 * cur_q2;
    rel_q2 = ref_q0 * cur_q2 + ref_q1 * cur_q3 - ref_q2 * cur_q0 - ref_q3 * cur_q1;
    rel_q3 = ref_q0 * cur_q3 - ref_q1 * cur_q2 + ref_q2 * cur_q1 - ref_q3 * cur_q0;

    n = rel_q0 * rel_q0 + rel_q1 * rel_q1 + rel_q2 * rel_q2 + rel_q3 * rel_q3;
    if(n > 1e-12f)
    {
        recipNorm = invSqrt(n);
        rel_q0 *= recipNorm;
        rel_q1 *= recipNorm;
        rel_q2 *= recipNorm;
        rel_q3 *= recipNorm;
    }
    else
    {
        rel_q0 = 1.0f;
        rel_q1 = 0.0f;
        rel_q2 = 0.0f;
        rel_q3 = 0.0f;
    }

    imu_quaternion_to_euler_values(rel_q0, rel_q1, rel_q2, rel_q3,
                                   pitch_deg, roll_deg, yaw_deg);
}

static void quaternion_to_relative_euler(void)
{
    float pitch;
    float roll;
    float yaw;

    imu_quaternion_relative_euler_values(ahrs_q_ref, ahrs_q,
                                         &pitch, &roll, &yaw);
    IMU_RelPitch = pitch;
    IMU_RelRoll = roll;
    IMU_RelYaw = yaw;
}

static void MahonyAHRSupdateState(float q_state[4],
                                  float *integral_x, float *integral_y, float *integral_z,
                                  float gx, float gy, float gz,
                                  float ax, float ay, float az,
                                  float mx, float my, float mz)
{
    float recipNorm;
    float n;
    float hx = 0.0f;
    float hy = 0.0f;
    float bx = 0.0f;
    float bz = 0.0f;
    float q0q0;
    float q0q1;
    float q0q2;
    float q0q3;
    float q1q1;
    float q1q2;
    float q1q3;
    float q2q2;
    float q2q3;
    float q3q3;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfwx;
    float halfwy;
    float halfwz;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;
    float halfex_acc;
    float halfey_acc;
    float halfez_acc;
    float halfex_mag;
    float halfey_mag;
    float halfez_mag;
    float acc_conf;
    float mag_conf = 0.0f;
    float qa;
    float qb;
    float qc;

    float q0;
    float q1;
    float q2;
    float q3;

    uint8_t mag_valid = (uint8_t)(((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)) ? 1 : 0);

    if((q_state == NULL) || (integral_x == NULL) || (integral_y == NULL) || (integral_z == NULL))
    {
        return;
    }

    q0 = q_state[0];
    q1 = q_state[1];
    q2 = q_state[2];
    q3 = q_state[3];

    if((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
    {
        recipNorm = imu_safe_inv_norm3(ax, ay, az);
        if(recipNorm > 0.0f)
        {
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            if(mag_valid)
            {
                recipNorm = imu_safe_inv_norm3(mx, my, mz);
                if(recipNorm > 0.0f)
                {
                    q0q0 = q0 * q0;
                    q0q1 = q0 * q1;
                    q0q2 = q0 * q2;
                    q0q3 = q0 * q3;
                    q1q1 = q1 * q1;
                    q1q2 = q1 * q2;
                    q1q3 = q1 * q3;
                    q2q2 = q2 * q2;
                    q2q3 = q2 * q3;
                    q3q3 = q3 * q3;

                    mx *= recipNorm;
                    my *= recipNorm;
                    mz *= recipNorm;

                    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
                    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
                    bx = sqrtf(hx * hx + hy * hy);
                    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
                }
                else
                {
                    mag_valid = 0;
                }
            }

            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            halfex_acc = (ay * halfvz - az * halfvy);
            halfey_acc = (az * halfvx - ax * halfvz);
            halfez_acc = (ax * halfvy - ay * halfvx);

            acc_conf = imu_clip(accel_confidence, 0.0f, 1.0f);
            halfex += imu_apply_confidence(halfex_acc, acc_conf);
            halfey += imu_apply_confidence(halfey_acc, acc_conf);
            halfez += imu_apply_confidence(halfez_acc, acc_conf);

            if(mag_valid)
            {
                halfwx = bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2);
                halfwy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3);
                halfwz = bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2);

                halfex_mag = (my * halfwz - mz * halfwy);
                halfey_mag = (mz * halfwx - mx * halfwz);
                halfez_mag = (mx * halfwy - my * halfwx);

                mag_conf = imu_clip(mag_confidence, 0.0f, 1.0f);
                halfex += imu_apply_confidence(halfex_mag, mag_conf);
                halfey += imu_apply_confidence(halfey_mag, mag_conf);
                halfez += imu_apply_confidence(halfez_mag, mag_conf);
            }

            if((halfex != 0.0f) || (halfey != 0.0f) || (halfez != 0.0f))
            {
                if(twoKi > 0.0f)
                {
                    *integral_x += twoKi * halfex * sample_dt;
                    *integral_y += twoKi * halfey * sample_dt;
                    *integral_z += twoKi * halfez * sample_dt;

                    *integral_x = imu_clip(*integral_x, -0.35f, 0.35f);
                    *integral_y = imu_clip(*integral_y, -0.35f, 0.35f);
                    *integral_z = imu_clip(*integral_z, -0.35f, 0.35f);

                    gx += *integral_x;
                    gy += *integral_y;
                    gz += *integral_z;
                }
                else
                {
                    *integral_x = 0.0f;
                    *integral_y = 0.0f;
                    *integral_z = 0.0f;
                }

                gx += twoKp * halfex;
                gy += twoKp * halfey;
                gz += twoKp * halfez;
            }
        }
    }

    gx *= 0.5f * sample_dt;
    gy *= 0.5f * sample_dt;
    gz *= 0.5f * sample_dt;

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = imu_safe_inv_norm3(q0, q1, q2);
    if(recipNorm > 0.0f)
    {
        n = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        if(n > 1e-12f)
        {
            recipNorm = invSqrt(n);
            q0 *= recipNorm;
            q1 *= recipNorm;
            q2 *= recipNorm;
            q3 *= recipNorm;
        }
        else
        {
            q0 = 1.0f;
            q1 = 0.0f;
            q2 = 0.0f;
            q3 = 0.0f;
        }
    }

    q_state[0] = q0;
    q_state[1] = q1;
    q_state[2] = q2;
    q_state[3] = q3;
}

static void MahonyAHRSupdate(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz)
{
    MahonyAHRSupdateState(ahrs_q, &integralFBx, &integralFBy, &integralFBz,
                          gx, gy, gz, ax, ay, az, mx, my, mz);
}

static void imu_compare_reset_runtime(void)
{
    imu_compare.valid = 0U;
    imu_compare.shadow_attitude_initialized = 0U;
    imu_compare.shadow_q[0] = 1.0f;
    imu_compare.shadow_q[1] = 0.0f;
    imu_compare.shadow_q[2] = 0.0f;
    imu_compare.shadow_q[3] = 0.0f;
    imu_compare.shadow_integral[0] = 0.0f;
    imu_compare.shadow_integral[1] = 0.0f;
    imu_compare.shadow_integral[2] = 0.0f;
    imu_compare.main_ref_q[0] = 1.0f;
    imu_compare.main_ref_q[1] = 0.0f;
    imu_compare.main_ref_q[2] = 0.0f;
    imu_compare.main_ref_q[3] = 0.0f;
    imu_compare.shadow_ref_q[0] = 1.0f;
    imu_compare.shadow_ref_q[1] = 0.0f;
    imu_compare.shadow_ref_q[2] = 0.0f;
    imu_compare.shadow_ref_q[3] = 0.0f;
    imu_compare.main_rel_pitch = 0.0f;
    imu_compare.main_rel_roll = 0.0f;
    imu_compare.main_rel_yaw = 0.0f;
    imu_compare.shadow_rel_pitch = 0.0f;
    imu_compare.shadow_rel_roll = 0.0f;
    imu_compare.shadow_rel_yaw = 0.0f;
    imu_compare.shadow_abs_pitch = 0.0f;
    imu_compare.shadow_abs_roll = 0.0f;
    imu_compare.shadow_abs_yaw = 0.0f;
    imu_compare.delta_pitch = 0.0f;
    imu_compare.delta_roll = 0.0f;
    imu_compare.delta_yaw = 0.0f;
}

static void imu_compare_init_shadow_backend(void)
{
#if (IMU_FUSION_ALGO_SHADOW == IMU_FUSION_ALGO_LEGACY_EKF)
    EKF_LEGACY_Init();
#endif
}

void IMU_AHRS_SetCompareEnable(uint8_t enable)
{
    if(enable != 0U)
    {
        if(imu_compare.enabled == 0U)
        {
            imu_compare_reset_runtime();
            imu_compare_init_shadow_backend();
            imu_compare.enabled = 1U;
        }
    }
    else
    {
        imu_compare.enabled = 0U;
        imu_compare_reset_runtime();
    }
}

void IMU_AHRS_Reset(void)
{
    ahrs_q[0] = 1.0f;
    ahrs_q[1] = 0.0f;
    ahrs_q[2] = 0.0f;
    ahrs_q[3] = 0.0f;

    ahrs_q_ref[0] = 1.0f;
    ahrs_q_ref[1] = 0.0f;
    ahrs_q_ref[2] = 0.0f;
    ahrs_q_ref[3] = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    sample_dt_cfg = IMU_AHRS_DEFAULT_DT;
    sample_dt = IMU_AHRS_DEFAULT_DT;
    dt_tick_init = 0;
    dt_last_tick_10ns = 0;

    IMU_Pitch = 0.0f;
    IMU_Roll = 0.0f;
    IMU_Yaw = 0.0f;
    IMU_RelPitch = 0.0f;
    IMU_RelRoll = 0.0f;
    IMU_RelYaw = 0.0f;

    accel_confidence = 1.0f;
    mag_confidence = 1.0f;
    mag_norm_ref = 0.0f;
    mag_ref_init = 0;
    imu_filter_init = 0;
    ahrs_attitude_initialized = 0U;
    ahrs_ref_locked = 0;
    ahrs_ref_elapsed_s = 0.0f;
    ahrs_ref_stable_elapsed_s = 0.0f;
    ahrs_ref_capture_request = 0;
    ahrs_mag_enable = 0;
    ahrs_last_abs_pitch_deg = 0.0f;
    ahrs_last_abs_roll_deg = 0.0f;
    ahrs_last_abs_yaw_deg = 0.0f;
    ahrs_last_abs_att_valid = 0U;
    imu_reset_gyro_bias();

    euler_angle.pitch = 0.0f;
    euler_angle.roll = 0.0f;
    euler_angle.yaw = 0.0f;
    imu_zero_relative_euler();

#if (IMU_FUSION_ALGO == IMU_FUSION_ALGO_LEGACY_EKF)
    EKF_LEGACY_Init();
#endif

    imu_compare_reset_runtime();
    if(imu_compare.enabled != 0U)
    {
        imu_compare_init_shadow_backend();
    }
}

void IMU_AHRS_SetSamplePeriod(float dt_s)
{
    if((dt_s >= IMU_DT_MIN_S) && (dt_s <= IMU_DT_MAX_S))
    {
        sample_dt_cfg = dt_s;
        sample_dt = dt_s;
    }
}

void IMU_AHRS_SetGains(float kp, float ki)
{
    if((kp >= 0.0f) && (kp <= 20.0f))
    {
        twoKp = kp;
    }

    if((ki >= 0.0f) && (ki <= 5.0f))
    {
        twoKi = ki;
    }
}

void IMU_AHRS_SetMagEnable(uint8_t enable)
{
    ahrs_mag_enable = (enable != 0U) ? 1U : 0U;

    if(ahrs_mag_enable == 0U)
    {
        integralFBz = 0.0f;
    }
}

void IMU_AHRS_CaptureReference(void)
{
    ahrs_ref_capture_request = 1U;
}

uint8_t IMU_AHRS_IsReferenceReady(void)
{
    return ahrs_ref_locked;
}

void IMU_AHRS_GetDebugInfo(imu_ahrs_debug_t *out)
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    if(out == NULL)
    {
        return;
    }

    gyro_x_dps = imu_data.gyro_x * DEG_TO_RAD;
    gyro_y_dps = imu_data.gyro_y * DEG_TO_RAD;
    gyro_z_dps = imu_data.gyro_z * DEG_TO_RAD;

    out->receive_flag = IMU_Receive_flag;
    out->filter_initialized = imu_filter_init;
    out->dt_initialized = dt_tick_init;
    out->attitude_initialized = ahrs_attitude_initialized;
    out->ref_locked = ahrs_ref_locked;
    out->ref_capture_request = ahrs_ref_capture_request;
    out->mag_enabled = ahrs_mag_enable;
    out->compare_enabled = imu_compare.enabled;
    out->compare_valid = imu_compare.valid;
    out->fusion_algo = IMU_FUSION_ALGO;
    out->compare_shadow_algo = IMU_FUSION_ALGO_SHADOW;
    out->gyro_bias_ready = gyro_bias_ready;
    out->gyro_bias_sample_count = gyro_bias_sample_count;
    out->gyro_bias_sample_target = GYRO_BIAS_SAMPLE_COUNT;
    out->sample_dt_s = sample_dt;
    out->sample_dt_cfg_s = sample_dt_cfg;
    out->two_kp = twoKp;
    out->two_ki = twoKi;
    out->integral_fb_x = integralFBx;
    out->integral_fb_y = integralFBy;
    out->integral_fb_z = integralFBz;
    out->accel_confidence = accel_confidence;
    out->mag_confidence = mag_confidence;
    out->acc_norm_g = sqrtf(imu_data.acc_x * imu_data.acc_x +
                            imu_data.acc_y * imu_data.acc_y +
                            imu_data.acc_z * imu_data.acc_z);
    out->mag_norm = sqrtf(imu_data.mag_x * imu_data.mag_x +
                          imu_data.mag_y * imu_data.mag_y +
                          imu_data.mag_z * imu_data.mag_z);
    out->mag_norm_ref = mag_norm_ref;
    out->gyro_x_dps = gyro_x_dps;
    out->gyro_y_dps = gyro_y_dps;
    out->gyro_z_dps = gyro_z_dps;
    out->gyro_norm_dps = sqrtf(gyro_x_dps * gyro_x_dps +
                               gyro_y_dps * gyro_y_dps +
                               gyro_z_dps * gyro_z_dps);
    out->gyro_bias_x_dps = gyro_bias_dps[0];
    out->gyro_bias_y_dps = gyro_bias_dps[1];
    out->gyro_bias_z_dps = gyro_bias_dps[2];
    out->q0 = ahrs_q[0];
    out->q1 = ahrs_q[1];
    out->q2 = ahrs_q[2];
    out->q3 = ahrs_q[3];
    out->q_ref0 = ahrs_q_ref[0];
    out->q_ref1 = ahrs_q_ref[1];
    out->q_ref2 = ahrs_q_ref[2];
    out->q_ref3 = ahrs_q_ref[3];
    out->ref_elapsed_s = ahrs_ref_elapsed_s;
    out->ref_stable_elapsed_s = ahrs_ref_stable_elapsed_s;
    out->cmp_main_pitch = imu_compare.main_rel_pitch;
    out->cmp_main_roll = imu_compare.main_rel_roll;
    out->cmp_main_yaw = imu_compare.main_rel_yaw;
    out->cmp_shadow_pitch = imu_compare.shadow_rel_pitch;
    out->cmp_shadow_roll = imu_compare.shadow_rel_roll;
    out->cmp_shadow_yaw = imu_compare.shadow_rel_yaw;
    out->shadow_abs_pitch = imu_compare.shadow_abs_pitch;
    out->shadow_abs_roll = imu_compare.shadow_abs_roll;
    out->shadow_abs_yaw = imu_compare.shadow_abs_yaw;
    out->cmp_delta_pitch = imu_compare.delta_pitch;
    out->cmp_delta_roll = imu_compare.delta_roll;
    out->cmp_delta_yaw = imu_compare.delta_yaw;
}

void IMU_AHRS_SetMagCalibration(const float offset[3], const float scale[3])
{
    float sx = 1.0f;
    float sy = 1.0f;
    float sz = 1.0f;

    if(offset != NULL)
    {
        mag_offset[0] = offset[0];
        mag_offset[1] = offset[1];
        mag_offset[2] = offset[2];
    }

    if(scale != NULL)
    {
        sx = (imu_absf(scale[0]) > 1e-6f) ? scale[0] : 1.0f;
        sy = (imu_absf(scale[1]) > 1e-6f) ? scale[1] : 1.0f;
        sz = (imu_absf(scale[2]) > 1e-6f) ? scale[2] : 1.0f;

        mag_softiron[0][0] = sx;  mag_softiron[0][1] = 0.0f; mag_softiron[0][2] = 0.0f;
        mag_softiron[1][0] = 0.0f; mag_softiron[1][1] = sy;  mag_softiron[1][2] = 0.0f;
        mag_softiron[2][0] = 0.0f; mag_softiron[2][1] = 0.0f; mag_softiron[2][2] = sz;
    }
}

void IMU_AHRS_SetMagCalibrationMatrix(const float offset[3], const float softiron[3][3])
{
    if(offset != NULL)
    {
        mag_offset[0] = offset[0];
        mag_offset[1] = offset[1];
        mag_offset[2] = offset[2];
    }

    if(softiron != NULL)
    {
        mag_softiron[0][0] = softiron[0][0];
        mag_softiron[0][1] = softiron[0][1];
        mag_softiron[0][2] = softiron[0][2];
        mag_softiron[1][0] = softiron[1][0];
        mag_softiron[1][1] = softiron[1][1];
        mag_softiron[1][2] = softiron[1][2];
        mag_softiron[2][0] = softiron[2][0];
        mag_softiron[2][1] = softiron[2][1];
        mag_softiron[2][2] = softiron[2][2];
    }
}

static void imu_legacy_ekf_load_current_input(void)
{
    ekf_legacy_imu_data.acc_x = imu_data.acc_x;
    ekf_legacy_imu_data.acc_y = imu_data.acc_y;
    ekf_legacy_imu_data.acc_z = imu_data.acc_z;
    ekf_legacy_imu_data.gyro_x = imu_data.gyro_x;
    ekf_legacy_imu_data.gyro_y = imu_data.gyro_y;
    ekf_legacy_imu_data.gyro_z = imu_data.gyro_z;
    ekf_legacy_imu_receive_flag = IMU_Receive_flag;
}

static void imu_legacy_ekf_update_from_current_input(float q_out[4])
{
    imu_legacy_ekf_load_current_input();
    EKF_LEGACY_UpData();
    EKF_LEGACY_GetQuaternion(q_out);
}

#if (IMU_FUSION_ALGO == IMU_FUSION_ALGO_LEGACY_EKF)
static void imu_legacy_ekf_update_main_from_current_input(void)
{
    float q_now[4];

    imu_legacy_ekf_update_from_current_input(q_now);

    ahrs_q[0] = q_now[0];
    ahrs_q[1] = q_now[1];
    ahrs_q[2] = q_now[2];
    ahrs_q[3] = q_now[3];
    ahrs_attitude_initialized = 1U;
}
#endif

static void imu_compare_capture_refs(void)
{
    imu_compare.main_ref_q[0] = ahrs_q[0];
    imu_compare.main_ref_q[1] = ahrs_q[1];
    imu_compare.main_ref_q[2] = ahrs_q[2];
    imu_compare.main_ref_q[3] = ahrs_q[3];
    imu_compare.shadow_ref_q[0] = imu_compare.shadow_q[0];
    imu_compare.shadow_ref_q[1] = imu_compare.shadow_q[1];
    imu_compare.shadow_ref_q[2] = imu_compare.shadow_q[2];
    imu_compare.shadow_ref_q[3] = imu_compare.shadow_q[3];
}

static void imu_compare_update(void)
{
    uint8_t just_initialized = 0U;

    if(imu_compare.enabled == 0U)
    {
        return;
    }

    if((IMU_Receive_flag == 0U) ||
       (gyro_bias_ready == 0U) ||
       (ahrs_attitude_initialized == 0U))
    {
        imu_compare.valid = 0U;
        return;
    }

#if (IMU_FUSION_ALGO_SHADOW == IMU_FUSION_ALGO_MAHONY)
    if(imu_compare.shadow_attitude_initialized == 0U)
    {
        if(imu_make_initial_quaternion(imu_compare.shadow_q) == 0U)
        {
            imu_compare.valid = 0U;
            return;
        }
        imu_compare.shadow_integral[0] = 0.0f;
        imu_compare.shadow_integral[1] = 0.0f;
        imu_compare.shadow_integral[2] = 0.0f;
        imu_compare.shadow_attitude_initialized = 1U;
        just_initialized = 1U;
    }

    {
#if IMU_AHRS_USE_MAG
        float mag_x = 0.0f;
        float mag_y = 0.0f;
        float mag_z = 0.0f;

        if((ahrs_mag_enable != 0U) &&
           ((ahrs_ref_locked == 0U) || (AHRS_MAG_CORRECT_AFTER_REF_LOCK != 0U)))
        {
            mag_x = imu_data.mag_x;
            mag_y = imu_data.mag_y;
            mag_z = imu_data.mag_z;
        }

        MahonyAHRSupdateState(imu_compare.shadow_q,
                              &imu_compare.shadow_integral[0],
                              &imu_compare.shadow_integral[1],
                              &imu_compare.shadow_integral[2],
                              imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                              imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                              mag_x, mag_y, mag_z);
#else
        MahonyAHRSupdateState(imu_compare.shadow_q,
                              &imu_compare.shadow_integral[0],
                              &imu_compare.shadow_integral[1],
                              &imu_compare.shadow_integral[2],
                              imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                              imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                              0.0f, 0.0f, 0.0f);
#endif
    }
#else
    imu_legacy_ekf_update_from_current_input(imu_compare.shadow_q);
    if(imu_compare.shadow_attitude_initialized == 0U)
    {
        imu_compare.shadow_attitude_initialized = 1U;
        just_initialized = 1U;
    }
#endif

    if(just_initialized != 0U)
    {
        imu_compare_capture_refs();
    }

    imu_quaternion_to_euler_values(imu_compare.shadow_q[0],
                                   imu_compare.shadow_q[1],
                                   imu_compare.shadow_q[2],
                                   imu_compare.shadow_q[3],
                                   &imu_compare.shadow_abs_pitch,
                                   &imu_compare.shadow_abs_roll,
                                   &imu_compare.shadow_abs_yaw);

    imu_quaternion_relative_euler_values(imu_compare.main_ref_q, ahrs_q,
                                         &imu_compare.main_rel_pitch,
                                         &imu_compare.main_rel_roll,
                                         &imu_compare.main_rel_yaw);
    imu_quaternion_relative_euler_values(imu_compare.shadow_ref_q, imu_compare.shadow_q,
                                         &imu_compare.shadow_rel_pitch,
                                         &imu_compare.shadow_rel_roll,
                                         &imu_compare.shadow_rel_yaw);

    imu_compare.delta_pitch = imu_wrap_deg_pm180(imu_compare.shadow_rel_pitch - imu_compare.main_rel_pitch);
    imu_compare.delta_roll = imu_wrap_deg_pm180(imu_compare.shadow_rel_roll - imu_compare.main_rel_roll);
    imu_compare.delta_yaw = imu_wrap_deg_pm180(imu_compare.shadow_rel_yaw - imu_compare.main_rel_yaw);
    imu_compare.valid = 1U;
}

uint8_t IMU_Sensor_Init(void)
{
    return imu963ra_init();
}

void EKF_Init(void)
{
    Matrix_Init(&exf_x, 4, 1);
    Matrix_Init(&error, 1, 1);

    IMU_AHRS_Reset();

    exf_x.data[0][0] = 1.0f;
    exf_x.data[1][0] = 0.0f;
    exf_x.data[2][0] = 0.0f;
    exf_x.data[3][0] = 0.0f;
}

uint8_t imu_get_values(void)
{
    float gyro_dps_x;
    float gyro_dps_y;
    float gyro_dps_z;
    float acc_now[3];
    float mag_now[3];
    float mag_center_x;
    float mag_center_y;
    float mag_center_z;
    float acc_conf;
#if IMU_AHRS_USE_MAG
    float mag_conf;
#endif

    imu963ra_get_acc();
    imu963ra_get_gyro();

    if(imu_raw_frame_is_bad(imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z,
                            imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z))
    {
        IMU_Receive_flag = 0;
        imu_compare.valid = 0U;
        return 0;
    }
#if IMU_AHRS_USE_MAG
    imu963ra_get_mag();
#endif

    acc_now[0] = imu963ra_acc_transition(imu963ra_acc_x);
    acc_now[1] = imu963ra_acc_transition(imu963ra_acc_y);
    acc_now[2] = imu963ra_acc_transition(imu963ra_acc_z);

#if IMU_AHRS_USE_MAG
    mag_now[0] = imu963ra_mag_transition(imu963ra_mag_x);
    mag_now[1] = imu963ra_mag_transition(imu963ra_mag_y);
    mag_now[2] = imu963ra_mag_transition(imu963ra_mag_z);
#else
    mag_now[0] = 0.0f;
    mag_now[1] = 0.0f;
    mag_now[2] = 0.0f;
#endif
    IMU_MagRawX = mag_now[0];
    IMU_MagRawY = mag_now[1];
    IMU_MagRawZ = mag_now[2];

    if(!imu_filter_init)
    {
        acc_lp[0] = acc_now[0];
        acc_lp[1] = acc_now[1];
        acc_lp[2] = acc_now[2];

        mag_lp[0] = mag_now[0];
        mag_lp[1] = mag_now[1];
        mag_lp[2] = mag_now[2];

        imu_filter_init = 1;
    }
    else
    {
        acc_lp[0] = K * acc_now[0] + (1.0f - K) * acc_lp[0];
        acc_lp[1] = K * acc_now[1] + (1.0f - K) * acc_lp[1];
        acc_lp[2] = K * acc_now[2] + (1.0f - K) * acc_lp[2];

        mag_lp[0] = K * mag_now[0] + (1.0f - K) * mag_lp[0];
        mag_lp[1] = K * mag_now[1] + (1.0f - K) * mag_lp[1];
        mag_lp[2] = K * mag_now[2] + (1.0f - K) * mag_lp[2];
    }

    imu_data.acc_x = acc_lp[0];
    imu_data.acc_y = acc_lp[1];
    imu_data.acc_z = acc_lp[2];

    mag_center_x = mag_lp[0] - mag_offset[0];
    mag_center_y = mag_lp[1] - mag_offset[1];
    mag_center_z = mag_lp[2] - mag_offset[2];

    imu_data.mag_x = mag_softiron[0][0] * mag_center_x + mag_softiron[0][1] * mag_center_y + mag_softiron[0][2] * mag_center_z;
    imu_data.mag_y = mag_softiron[1][0] * mag_center_x + mag_softiron[1][1] * mag_center_y + mag_softiron[1][2] * mag_center_z;
    imu_data.mag_z = mag_softiron[2][0] * mag_center_x + mag_softiron[2][1] * mag_center_y + mag_softiron[2][2] * mag_center_z;

    acc_conf = imu_compute_acc_confidence(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
#if IMU_AHRS_USE_MAG
    mag_conf = imu_compute_mag_confidence(imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
    mag_confidence = imu_clip(0.8f * mag_confidence + 0.2f * mag_conf, 0.0f, 1.0f);
#else
    mag_confidence = 1.0f;
#endif
    if(acc_conf < accel_confidence)
    {
        accel_confidence = imu_clip((1.0f - ACC_CONF_FALL_ALPHA) * accel_confidence +
                                    ACC_CONF_FALL_ALPHA * acc_conf,
                                    0.0f,
                                    1.0f);
    }
    else
    {
        accel_confidence = imu_clip((1.0f - ACC_CONF_RISE_ALPHA) * accel_confidence +
                                    ACC_CONF_RISE_ALPHA * acc_conf,
                                    0.0f,
                                    1.0f);
    }

    gyro_dps_x = imu963ra_gyro_transition(imu963ra_gyro_x);
    gyro_dps_y = imu963ra_gyro_transition(imu963ra_gyro_y);
    gyro_dps_z = imu963ra_gyro_transition(imu963ra_gyro_z);

    imu_update_gyro_bias(gyro_dps_x, gyro_dps_y, gyro_dps_z);
    if(gyro_bias_ready)
    {
        gyro_dps_x -= gyro_bias_dps[0];
        gyro_dps_y -= gyro_bias_dps[1];
        gyro_dps_z -= gyro_bias_dps[2];
    }
    else
    {
        gyro_dps_x = 0.0f;
        gyro_dps_y = 0.0f;
        gyro_dps_z = 0.0f;
    }

    if(imu_absf(gyro_dps_x) < GYRO_INPUT_DEADBAND_DPS) gyro_dps_x = 0.0f;
    if(imu_absf(gyro_dps_y) < GYRO_INPUT_DEADBAND_DPS) gyro_dps_y = 0.0f;
    if(imu_absf(gyro_dps_z) < GYRO_INPUT_DEADBAND_DPS) gyro_dps_z = 0.0f;

    imu_data.gyro_x = gyro_dps_x * PI / 180.0f;
    imu_data.gyro_y = gyro_dps_y * PI / 180.0f;
    imu_data.gyro_z = gyro_dps_z * PI / 180.0f;

    IMU_GYRO_Z = gyro_dps_z;

    IMU_Receive_flag = 1;

    return 1;
}

void EKF_UpData(void)
{
#if (IMU_FUSION_ALGO == IMU_FUSION_ALGO_MAHONY)
#if IMU_AHRS_USE_MAG
    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;
#endif
#endif

    imu_update_dynamic_dt();

#if (IMU_FUSION_ALGO == IMU_FUSION_ALGO_MAHONY)
#if IMU_AHRS_USE_MAG
    if((ahrs_mag_enable != 0U) &&
       ((ahrs_ref_locked == 0U) || (AHRS_MAG_CORRECT_AFTER_REF_LOCK != 0U)))
    {
        mag_x = imu_data.mag_x;
        mag_y = imu_data.mag_y;
        mag_z = imu_data.mag_z;
    }
#endif

    if(ahrs_attitude_initialized == 0U)
    {
        (void)imu_init_attitude_from_sensors();
    }

    MahonyAHRSupdate(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                     imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
#if IMU_AHRS_USE_MAG
                     mag_x, mag_y, mag_z);
#else
                     0.0f, 0.0f, 0.0f);
#endif
#elif (IMU_FUSION_ALGO == IMU_FUSION_ALGO_LEGACY_EKF)
    imu_legacy_ekf_update_main_from_current_input();
#endif

    exf_x.data[0][0] = ahrs_q[0];
    exf_x.data[1][0] = ahrs_q[1];
    exf_x.data[2][0] = ahrs_q[2];
    exf_x.data[3][0] = ahrs_q[3];

    quaternion_to_euler();
    imu_compare_update();
    if(!gyro_bias_ready)
    {
        ahrs_ref_stable_elapsed_s = 0.0f;
        ahrs_last_abs_att_valid = 0U;
        ahrs_ref_elapsed_s = 0.0f;
        imu_zero_relative_euler();
        return;
    }

    if(ahrs_attitude_initialized == 0U)
    {
        ahrs_ref_stable_elapsed_s = 0.0f;
        ahrs_last_abs_att_valid = 0U;
        imu_zero_relative_euler();
        return;
    }

    if(ahrs_ref_capture_request != 0U)
    {
        ahrs_ref_capture_request = 0U;
        imu_capture_reference_now();
    }

    if(!ahrs_ref_locked)
    {
        ahrs_ref_elapsed_s += sample_dt;

        if(imu_is_reference_stable())
        {
            ahrs_ref_stable_elapsed_s += sample_dt;
            if(ahrs_ref_stable_elapsed_s >= AHRS_REF_STABLE_HOLD_S)
            {
                imu_capture_reference_now();
            }
        }
        else
        {
            ahrs_ref_stable_elapsed_s = 0.0f;
        }
    }

    if(!ahrs_ref_locked)
    {
        imu_zero_relative_euler();
        return;
    }

    quaternion_to_relative_euler();
}
//dy ahstu zhugeliang ltl
