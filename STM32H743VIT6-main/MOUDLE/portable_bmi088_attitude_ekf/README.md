# Portable BMI088 Attitude EKF

This folder is a standalone extraction of the attitude part of the original project:

- State: `q0, q1, q2, q3, gyro_bias_x, gyro_bias_y`
- Measurement: normalized accelerometer vector
- Input units: gyro in `rad/s`, accel in `m/s^2`, `dt` in seconds
- Output: quaternion, yaw/pitch/roll in degrees, continuous yaw angle

The accelerometer can correct roll and pitch through the gravity direction. It cannot observe yaw drift, so `gyro_bias_z` is kept at zero. If yaw must be drift-free, add an external yaw observation from magnetometer, vision, wheel odometry, or another heading source.

## Files to copy

Copy these two files into the target project:

```text
attitude_ekf.c
attitude_ekf.h
```

No CMSIS-DSP dependency is required in this portable version.

## Basic use

Keep the robot still during initialization, average several BMI088 accel samples, then initialize:

```c
#include "attitude_ekf.h"

static AttitudeEKF imu_ekf;

void imu_attitude_init(float avg_ax, float avg_ay, float avg_az)
{
    AttitudeEKF_InitFromAccel(&imu_ekf,
                              avg_ax, avg_ay, avg_az,
                              10.0f,       /* quaternion process noise */
                              0.001f,      /* gyro x/y bias process noise */
                              1000000.0f,  /* accel measurement noise */
                              1.0f,        /* fading lambda */
                              0.0f);       /* accel LPF time constant */
}
```

Call the update function at a steady rate, ideally 500 Hz to 1000 Hz:

```c
void imu_attitude_update(void)
{
    float gx = bmi088.gyro[0];   /* rad/s */
    float gy = bmi088.gyro[1];
    float gz = bmi088.gyro[2];
    float ax = bmi088.accel[0];  /* m/s^2 */
    float ay = bmi088.accel[1];
    float az = bmi088.accel[2];
    float dt = get_delta_time_s();

    AttitudeEKF_Update(&imu_ekf, gx, gy, gz, ax, ay, az, dt);

    float yaw = imu_ekf.yaw;
    float pitch = imu_ekf.pitch;
    float roll = imu_ekf.roll;
}
```

## BMI088 data requirements

Before calling `AttitudeEKF_Update`, convert BMI088 data like the original project:

- gyro raw data -> `rad/s`
- accel raw data -> `m/s^2`
- subtract basic gyro zero offsets, especially `gz`, because this EKF does not estimate yaw-axis bias
- scale accel so the stationary norm is close to `9.81`

If the angle direction is wrong in the target project, fix the BMI088 axis mapping before feeding the EKF. The original convention expects a level stationary sensor to report acceleration close to `[0, 0, +9.81]`.

## Original-compatible defaults

The default parameters matching the source project are:

```c
q_noise = 10.0f;
bias_noise = 0.001f;
accel_noise = 1000000.0f;
lambda = 1.0f;
accel_lpf = 0.0f;
```

For noisier chassis motion, increase `accel_noise` or set `accel_lpf` to a small value such as `0.005f` to `0.02f`. For faster roll/pitch correction, decrease `accel_noise`, but acceleration during driving will pull the attitude estimate more easily.
