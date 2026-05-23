#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t converge_flag;
    uint8_t stable_flag;
    uint32_t error_count;
    uint32_t update_count;

    float q[4];          /* q0, q1, q2, q3 */
    float gyro_bias[3];  /* rad/s. z bias is not observable here and is kept 0. */
    float gyro[3];       /* bias-corrected gyro, rad/s */
    float accel[3];      /* low-pass accel, m/s^2 */

    float yaw;
    float pitch;
    float roll;
    float yaw_total_angle;

    float q_noise;
    float bias_noise;
    float accel_noise;
    float lambda;
    float accel_lpf;
    float dt;

    float gyro_norm;
    float accel_norm;
    float adaptive_gain_scale;
    float chi_square;
    float chi_square_threshold;

    int16_t yaw_round_count;
    float yaw_last;

    /* Internal EKF storage. */
    float x[6];
    float x_minus[6];
    float p[36];
    float p_minus[36];
    float f[36];
    float q_mat[36];
    float h[18];
    float r[9];
    float s[9];
    float k[18];
} AttitudeEKF;

void AttitudeEKF_MakeInitQuaternionFromAccel(float ax, float ay, float az, float q[4]);

void AttitudeEKF_Init(AttitudeEKF *ekf,
                      const float init_q[4],
                      float q_noise,
                      float bias_noise,
                      float accel_noise,
                      float lambda,
                      float accel_lpf);

void AttitudeEKF_InitFromAccel(AttitudeEKF *ekf,
                               float ax,
                               float ay,
                               float az,
                               float q_noise,
                               float bias_noise,
                               float accel_noise,
                               float lambda,
                               float accel_lpf);

void AttitudeEKF_Update(AttitudeEKF *ekf,
                        float gx,
                        float gy,
                        float gz,
                        float ax,
                        float ay,
                        float az,
                        float dt);

#ifdef __cplusplus
}
#endif

#endif
