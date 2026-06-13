/*
 * MPU6050.c
 *
 *  Created on: Aug 25, 2024
 *      Author: 王滋行
 */

#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include <stddef.h>
#include <stdint.h>

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* First-order low-pass filter for angular velocity feedback. */
#define MPU6050_ANGULAR_VELOCITY_LPF_ALPHA 0.20f
static float filtered_pitch_rate_dps = 0.0f;
static float filtered_yaw_rate_dps = 0.0f;
static float filtered_gx_rate_dps = 0.0f;
static uint8_t angular_velocity_filter_initialized = 0U;

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    } else {
        return -1;
    }

    return 0;
}

int MPU6050_DMP_init(void)
{
    int ret;
    //mpu_init
    ret = mpu_init(NULL);
    if(ret != 0)
    {
        return ERROR_MPU_INIT;
    }
    //设置传感器
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_SET_SENSOR;
    }
    //设置fifo
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_CONFIG_FIFO;
    }
    //设置采样率
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_RATE;
    }
    //加载DMP固件
    ret = dmp_load_motion_driver_firmware();
    if(ret != 0)
    {
        return ERROR_LOAD_MOTION_DRIVER;
    }
    //设置陀螺仪方向
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(ret != 0)
    {
        return ERROR_SET_ORIENTATION;
    }
    //设置DMP功能
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if(ret != 0)
    {
        return ERROR_ENABLE_FEATURE;
    }
    //设置输出速率
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_FIFO_RATE;
    }
    //自检
    ret = run_self_test();
    if(ret != 0)
    {
        return ERROR_SELF_TEST;
    }
    //使能DMP
    ret = mpu_set_dmp_state(1);
    if(ret != 0)
    {
        return ERROR_DMP_STATE;
    }

    filtered_pitch_rate_dps = 0.0f;
    filtered_yaw_rate_dps = 0.0f;
    filtered_gx_rate_dps = 0.0f;
    angular_velocity_filter_initialized = 0U;

    return 0;
}

int MPU6050_DMP_Get_Date(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if(sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }

    return 0;
}

int MPU6050_DMP_Get_AngularVelocity(float *pitch_rate_dps, float *yaw_rate_dps)
{
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float sens;
    float q0, q1, q2, q3;

    if ((pitch_rate_dps == NULL) || (yaw_rate_dps == NULL))
    {
        return -1;
    }

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if (mpu_get_gyro_sens(&sens))
    {
        return -1;
    }

    /* Convert raw gyro to deg/s in body frame */
    float raw_body_gx = (float)gyro[0] / sens;
    float raw_body_gy = (float)gyro[1] / sens;
    float raw_body_gz = (float)gyro[2] / sens;

    /* First-order low-pass filter in body frame (before projection) */
    if (angular_velocity_filter_initialized == 0U)
    {
        filtered_pitch_rate_dps = raw_body_gy;
        filtered_yaw_rate_dps   = raw_body_gz;
        filtered_gx_rate_dps    = raw_body_gx;
        angular_velocity_filter_initialized = 1U;
    }
    else
    {
        filtered_pitch_rate_dps += MPU6050_ANGULAR_VELOCITY_LPF_ALPHA * (raw_body_gy - filtered_pitch_rate_dps);
        filtered_yaw_rate_dps   += MPU6050_ANGULAR_VELOCITY_LPF_ALPHA * (raw_body_gz - filtered_yaw_rate_dps);
        filtered_gx_rate_dps    += MPU6050_ANGULAR_VELOCITY_LPF_ALPHA * (raw_body_gx - filtered_gx_rate_dps);
    }

    /* Build filtered body-frame angular velocity vector */
    float body_filtered_gx = filtered_gx_rate_dps;
    float body_filtered_gy = filtered_pitch_rate_dps;
    float body_filtered_gz = filtered_yaw_rate_dps;

    /* Project body-frame angular velocity to world frame using DMP quaternion */
    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        /* ω_world = q ⊗ ω_body ⊗ q⁻¹
         * where ω_body = [0, gx, gy, gz] as a pure quaternion.
         * Compute ω_world = q * ω_body * conj(q), keep only the vector part.
         *
         * Let q   = [q0, q1, q2, q3]  (w, x, y, z)
         * Let ω_b = [0,  gx, gy, gz]
         *
         * Step 1: t = q * ω_b
         *    t_w = -q1*gx - q2*gy - q3*gz
         *    t_x =  q0*gx + q2*gz - q3*gy
         *    t_y =  q0*gy - q1*gz + q3*gx
         *    t_z =  q0*gz + q1*gy - q2*gx
         *
         * Step 2: ω_w = t * conj(q)  → vector part
         *    ω_x = t_w*(-q1) + t_x*q0 + t_y*q3 - t_z*q2
         *    ω_y = t_w*(-q2) - t_x*q3 + t_y*q0 + t_z*q1
         *    ω_z = t_w*(-q3) + t_x*q2 - t_y*q1 + t_z*q0
         */

        float gx = body_filtered_gx;
        float gy = body_filtered_gy;
        float gz = body_filtered_gz;

        float t_w = -q1 * gx - q2 * gy - q3 * gz;
        float t_x =  q0 * gx + q2 * gz - q3 * gy;
        float t_y =  q0 * gy - q1 * gz + q3 * gx;
        float t_z =  q0 * gz + q1 * gy - q2 * gx;

        float world_gx = t_w * (-q1) + t_x * q0 + t_y * q3 - t_z * q2;
        float world_gy = t_w * (-q2) - t_x * q3 + t_y * q0 + t_z * q1;
        float world_gz = t_w * (-q3) + t_x * q2 - t_y * q1 + t_z * q0;

        /* World-frame pitch rate = rotation around world Y axis = world_gy
         * World-frame yaw rate   = rotation around world Z axis = world_gz */
        *pitch_rate_dps = world_gy;
        *yaw_rate_dps   = world_gz;
    }
    else
    {
        /* Fallback: no quaternion available, use body-frame values directly */
        *pitch_rate_dps = filtered_pitch_rate_dps;
        *yaw_rate_dps   = filtered_yaw_rate_dps;
    }

    return 0;
}

