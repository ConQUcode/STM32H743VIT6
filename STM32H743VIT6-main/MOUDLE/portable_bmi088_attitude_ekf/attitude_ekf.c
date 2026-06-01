#include "attitude_ekf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef ATTITUDE_EKF_PI
#define ATTITUDE_EKF_PI 3.14159265358979323846f
#endif

#define DEG_PER_RAD 57.29577951308232f
#define EKF_EPS 1.0e-9f

static const float kIdentity6[36] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};

static const float kInitialP[36] = {
    100000.0f, 0.1f,      0.1f,      0.1f,      0.1f,   0.1f,
    0.1f,      100000.0f, 0.1f,      0.1f,      0.1f,   0.1f,
    0.1f,      0.1f,      100000.0f, 0.1f,      0.1f,   0.1f,
    0.1f,      0.1f,      0.1f,      100000.0f, 0.1f,   0.1f,
    0.1f,      0.1f,      0.1f,      0.1f,      100.0f, 0.1f,
    0.1f,      0.1f,      0.1f,      0.1f,      0.1f,   100.0f,
};

static float clampf_local(float x, float min_v, float max_v)
{
    if (x < min_v) {
        return min_v;
    }
    if (x > max_v) {
        return max_v;
    }
    return x;
}

static float inv_sqrt(float x)
{
    if (x <= EKF_EPS) {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

static void normalize_quaternion(float q[4])
{
    const float inv_norm = inv_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (inv_norm <= 0.0f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    for (uint8_t i = 0; i < 4; ++i) {
        q[i] *= inv_norm;
    }
}

static void mat_mul(const float *a, uint8_t a_rows, uint8_t a_cols,
                    const float *b, uint8_t b_cols,
                    float *out)
{
    for (uint8_t r = 0; r < a_rows; ++r) {
        for (uint8_t c = 0; c < b_cols; ++c) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < a_cols; ++k) {
                sum += a[r * a_cols + k] * b[k * b_cols + c];
            }
            out[r * b_cols + c] = sum;
        }
    }
}

static void mat_transpose(const float *a, uint8_t rows, uint8_t cols, float *out)
{
    for (uint8_t r = 0; r < rows; ++r) {
        for (uint8_t c = 0; c < cols; ++c) {
            out[c * rows + r] = a[r * cols + c];
        }
    }
}

static int mat_inverse_3x3(const float a[9], float inv[9])
{
    const float a00 = a[0], a01 = a[1], a02 = a[2];
    const float a10 = a[3], a11 = a[4], a12 = a[5];
    const float a20 = a[6], a21 = a[7], a22 = a[8];

    const float c00 = a11 * a22 - a12 * a21;
    const float c01 = -(a10 * a22 - a12 * a20);
    const float c02 = a10 * a21 - a11 * a20;
    const float c10 = -(a01 * a22 - a02 * a21);
    const float c11 = a00 * a22 - a02 * a20;
    const float c12 = -(a00 * a21 - a01 * a20);
    const float c20 = a01 * a12 - a02 * a11;
    const float c21 = -(a00 * a12 - a02 * a10);
    const float c22 = a00 * a11 - a01 * a10;

    const float det = a00 * c00 + a01 * c01 + a02 * c02;
    if (fabsf(det) <= EKF_EPS) {
        return 0;
    }

    const float inv_det = 1.0f / det;
    inv[0] = c00 * inv_det;
    inv[1] = c10 * inv_det;
    inv[2] = c20 * inv_det;
    inv[3] = c01 * inv_det;
    inv[4] = c11 * inv_det;
    inv[5] = c21 * inv_det;
    inv[6] = c02 * inv_det;
    inv[7] = c12 * inv_det;
    inv[8] = c22 * inv_det;
    return 1;
}

static void set_process_and_measure_noise(AttitudeEKF *ekf)
{
    memset(ekf->q_mat, 0, sizeof(ekf->q_mat));
    ekf->q_mat[0] = ekf->q_noise * ekf->dt;
    ekf->q_mat[7] = ekf->q_noise * ekf->dt;
    ekf->q_mat[14] = ekf->q_noise * ekf->dt;
    ekf->q_mat[21] = ekf->q_noise * ekf->dt;
    ekf->q_mat[28] = ekf->bias_noise * ekf->dt;
    ekf->q_mat[35] = ekf->bias_noise * ekf->dt;

    memset(ekf->r, 0, sizeof(ekf->r));
    ekf->r[0] = ekf->accel_noise;
    ekf->r[4] = ekf->accel_noise;
    ekf->r[8] = ekf->accel_noise;
}

static void build_f_from_gyro(AttitudeEKF *ekf)
{
    const float halfgxdt = 0.5f * ekf->gyro[0] * ekf->dt;
    const float halfgydt = 0.5f * ekf->gyro[1] * ekf->dt;
    const float halfgzdt = 0.5f * ekf->gyro[2] * ekf->dt;

    memcpy(ekf->f, kIdentity6, sizeof(ekf->f));

    ekf->f[1] = -halfgxdt;
    ekf->f[2] = -halfgydt;
    ekf->f[3] = -halfgzdt;

    ekf->f[6] = halfgxdt;
    ekf->f[8] = halfgzdt;
    ekf->f[9] = -halfgydt;

    ekf->f[12] = halfgydt;
    ekf->f[13] = -halfgzdt;
    ekf->f[15] = halfgxdt;

    ekf->f[18] = halfgzdt;
    ekf->f[19] = halfgydt;
    ekf->f[20] = -halfgxdt;
}

static void add_bias_jacobian_and_fading(AttitudeEKF *ekf)
{
    const float q0 = ekf->x_minus[0];
    const float q1 = ekf->x_minus[1];
    const float q2 = ekf->x_minus[2];
    const float q3 = ekf->x_minus[3];

    ekf->f[4] = q1 * ekf->dt * 0.5f;
    ekf->f[5] = q2 * ekf->dt * 0.5f;

    ekf->f[10] = -q0 * ekf->dt * 0.5f;
    ekf->f[11] = q3 * ekf->dt * 0.5f;

    ekf->f[16] = -q3 * ekf->dt * 0.5f;
    ekf->f[17] = -q0 * ekf->dt * 0.5f;

    ekf->f[22] = q2 * ekf->dt * 0.5f;
    ekf->f[23] = -q1 * ekf->dt * 0.5f;

    ekf->p[28] /= ekf->lambda;
    ekf->p[35] /= ekf->lambda;

    if (ekf->p[28] > 10000.0f) {
        ekf->p[28] = 10000.0f;
    }
    if (ekf->p[35] > 10000.0f) {
        ekf->p[35] = 10000.0f;
    }
}

static void predict_covariance(AttitudeEKF *ekf)
{
    float ft[36];
    float temp[36];
    mat_transpose(ekf->f, 6, 6, ft);
    mat_mul(ekf->f, 6, 6, ekf->p, 6, temp);
    mat_mul(temp, 6, 6, ft, 6, ekf->p_minus);

    for (uint8_t i = 0; i < 36; ++i) {
        ekf->p_minus[i] += ekf->q_mat[i];
    }
}

static void set_h(AttitudeEKF *ekf)
{
    const float doubleq0 = 2.0f * ekf->x_minus[0];
    const float doubleq1 = 2.0f * ekf->x_minus[1];
    const float doubleq2 = 2.0f * ekf->x_minus[2];
    const float doubleq3 = 2.0f * ekf->x_minus[3];

    memset(ekf->h, 0, sizeof(ekf->h));

    ekf->h[0] = -doubleq2;
    ekf->h[1] = doubleq3;
    ekf->h[2] = -doubleq0;
    ekf->h[3] = doubleq1;

    ekf->h[6] = doubleq1;
    ekf->h[7] = doubleq0;
    ekf->h[8] = doubleq3;
    ekf->h[9] = doubleq2;

    ekf->h[12] = doubleq0;
    ekf->h[13] = -doubleq1;
    ekf->h[14] = -doubleq2;
    ekf->h[15] = doubleq3;
}

static void predict_gravity_from_q(const float q[4], float hq[3])
{
    const float q0 = q[0];
    const float q1 = q[1];
    const float q2 = q[2];
    const float q3 = q[3];

    hq[0] = 2.0f * (q1 * q3 - q0 * q2);
    hq[1] = 2.0f * (q0 * q1 + q2 * q3);
    hq[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

static void compute_euler(AttitudeEKF *ekf)
{
    const float q0 = ekf->q[0];
    const float q1 = ekf->q[1];
    const float q2 = ekf->q[2];
    const float q3 = ekf->q[3];

    ekf->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      2.0f * (q0 * q0 + q1 * q1) - 1.0f) * DEG_PER_RAD;
    ekf->pitch = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        2.0f * (q0 * q0 + q3 * q3) - 1.0f) * DEG_PER_RAD;
    ekf->roll = asinf(clampf_local(-2.0f * (q1 * q3 - q0 * q2), -1.0f, 1.0f)) * DEG_PER_RAD;

    if (ekf->yaw - ekf->yaw_last > 180.0f) {
        ekf->yaw_round_count--;
    } else if (ekf->yaw - ekf->yaw_last < -180.0f) {
        ekf->yaw_round_count++;
    }

    ekf->yaw_total_angle = 360.0f * (float)ekf->yaw_round_count + ekf->yaw;
    ekf->yaw_last = ekf->yaw;
}

static void finish_prediction_only(AttitudeEKF *ekf)
{
    memcpy(ekf->x, ekf->x_minus, sizeof(ekf->x));
    memcpy(ekf->p, ekf->p_minus, sizeof(ekf->p));
}

static void update_outputs(AttitudeEKF *ekf)
{
    memcpy(ekf->q, ekf->x, sizeof(ekf->q));
    normalize_quaternion(ekf->q);
    memcpy(ekf->x, ekf->q, sizeof(ekf->q));

    ekf->gyro_bias[0] = ekf->x[4];
    ekf->gyro_bias[1] = ekf->x[5];
    ekf->gyro_bias[2] = 0.0f;

    compute_euler(ekf);
}

void AttitudeEKF_MakeInitQuaternionFromAccel(float ax, float ay, float az, float q[4])
{
    float a[3] = {ax, ay, az};
    const float inv_norm = inv_sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (inv_norm <= 0.0f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    a[0] *= inv_norm;
    a[1] *= inv_norm;
    a[2] *= inv_norm;

    const float gravity[3] = {0.0f, 0.0f, 1.0f};
    float axis[3] = {
        a[1] * gravity[2] - a[2] * gravity[1],
        a[2] * gravity[0] - a[0] * gravity[2],
        a[0] * gravity[1] - a[1] * gravity[0],
    };

    float dot = a[0] * gravity[0] + a[1] * gravity[1] + a[2] * gravity[2];
    dot = clampf_local(dot, -1.0f, 1.0f);

    const float axis_inv_norm = inv_sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (axis_inv_norm <= 0.0f) {
        if (dot > 0.0f) {
            q[0] = 1.0f;
            q[1] = 0.0f;
            q[2] = 0.0f;
            q[3] = 0.0f;
        } else {
            q[0] = 0.0f;
            q[1] = 1.0f;
            q[2] = 0.0f;
            q[3] = 0.0f;
        }
        return;
    }

    axis[0] *= axis_inv_norm;
    axis[1] *= axis_inv_norm;
    axis[2] *= axis_inv_norm;

    const float angle = acosf(dot);
    const float half_sin = sinf(angle * 0.5f);
    q[0] = cosf(angle * 0.5f);
    q[1] = axis[0] * half_sin;
    q[2] = axis[1] * half_sin;
    q[3] = axis[2] * half_sin;
    normalize_quaternion(q);
}

void AttitudeEKF_Init(AttitudeEKF *ekf,
                      const float init_q[4],
                      float q_noise,
                      float bias_noise,
                      float accel_noise,
                      float lambda,
                      float accel_lpf)
{
    if (ekf == NULL || init_q == NULL) {
        return;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->initialized = 1;
    ekf->q_noise = q_noise;
    ekf->bias_noise = bias_noise;
    ekf->accel_noise = accel_noise;
    ekf->lambda = lambda;
    ekf->accel_lpf = accel_lpf;
    ekf->chi_square_threshold = 1.0e-8f;

    if (ekf->lambda <= 0.0f || ekf->lambda > 1.0f) {
        ekf->lambda = 1.0f;
    }
    if (ekf->accel_lpf < 0.0f) {
        ekf->accel_lpf = 0.0f;
    }

    memcpy(ekf->x, init_q, sizeof(float) * 4);
    normalize_quaternion(ekf->x);
    memcpy(ekf->q, ekf->x, sizeof(ekf->q));
    memcpy(ekf->p, kInitialP, sizeof(ekf->p));
    memcpy(ekf->f, kIdentity6, sizeof(ekf->f));
    compute_euler(ekf);
}

void AttitudeEKF_InitFromAccel(AttitudeEKF *ekf,
                               float ax,
                               float ay,
                               float az,
                               float q_noise,
                               float bias_noise,
                               float accel_noise,
                               float lambda,
                               float accel_lpf)
{
    float init_q[4];
    AttitudeEKF_MakeInitQuaternionFromAccel(ax, ay, az, init_q);
    AttitudeEKF_Init(ekf, init_q, q_noise, bias_noise, accel_noise, lambda, accel_lpf);
}

void AttitudeEKF_Update(AttitudeEKF *ekf,
                        float gx,
                        float gy,
                        float gz,
                        float ax,
                        float ay,
                        float az,
                        float dt)
{
    if (ekf == NULL || ekf->initialized == 0 || dt <= EKF_EPS) {
        return;
    }

    ekf->dt = dt;
    ekf->gyro[0] = gx - ekf->gyro_bias[0];
    ekf->gyro[1] = gy - ekf->gyro_bias[1];
    ekf->gyro[2] = gz;

    build_f_from_gyro(ekf);
    mat_mul(ekf->f, 6, 6, ekf->x, 1, ekf->x_minus);
    normalize_quaternion(ekf->x_minus);

    add_bias_jacobian_and_fading(ekf);
    set_process_and_measure_noise(ekf);
    predict_covariance(ekf);
    set_h(ekf);

    if (ekf->update_count == 0 || ekf->accel_lpf <= 0.0f) {
        ekf->accel[0] = ax;
        ekf->accel[1] = ay;
        ekf->accel[2] = az;
    } else {
        const float old_w = ekf->accel_lpf / (dt + ekf->accel_lpf);
        const float new_w = dt / (dt + ekf->accel_lpf);
        ekf->accel[0] = ekf->accel[0] * old_w + ax * new_w;
        ekf->accel[1] = ekf->accel[1] * old_w + ay * new_w;
        ekf->accel[2] = ekf->accel[2] * old_w + az * new_w;
    }

    const float accel_sq = ekf->accel[0] * ekf->accel[0] +
                           ekf->accel[1] * ekf->accel[1] +
                           ekf->accel[2] * ekf->accel[2];
    const float accel_inv_norm = inv_sqrt(accel_sq);
    if (accel_inv_norm <= 0.0f) {
        finish_prediction_only(ekf);
        update_outputs(ekf);
        ekf->update_count++;
        return;
    }

    const float z[3] = {
        ekf->accel[0] * accel_inv_norm,
        ekf->accel[1] * accel_inv_norm,
        ekf->accel[2] * accel_inv_norm,
    };

    ekf->gyro_norm = sqrtf(ekf->gyro[0] * ekf->gyro[0] +
                           ekf->gyro[1] * ekf->gyro[1] +
                           ekf->gyro[2] * ekf->gyro[2]);
    ekf->accel_norm = 1.0f / accel_inv_norm;
    ekf->stable_flag = (ekf->gyro_norm < 0.3f &&
                        ekf->accel_norm > 9.3f &&
                        ekf->accel_norm < 10.3f);

    float ht[18];
    float hp[18];
    float hpht[9];
    float inv_s[9];
    mat_transpose(ekf->h, 3, 6, ht);
    mat_mul(ekf->h, 3, 6, ekf->p_minus, 6, hp);
    mat_mul(hp, 3, 6, ht, 3, hpht);
    for (uint8_t i = 0; i < 9; ++i) {
        ekf->s[i] = hpht[i] + ekf->r[i];
    }
    if (!mat_inverse_3x3(ekf->s, inv_s)) {
        finish_prediction_only(ekf);
        update_outputs(ekf);
        ekf->update_count++;
        return;
    }

    float hq[3];
    float residual[3];
    predict_gravity_from_q(ekf->x_minus, hq);
    residual[0] = z[0] - hq[0];
    residual[1] = z[1] - hq[1];
    residual[2] = z[2] - hq[2];

    float invs_residual[3];
    mat_mul(inv_s, 3, 3, residual, 1, invs_residual);
    ekf->chi_square = residual[0] * invs_residual[0] +
                      residual[1] * invs_residual[1] +
                      residual[2] * invs_residual[2];

    if (ekf->chi_square < 0.5f * ekf->chi_square_threshold) {
        ekf->converge_flag = 1;
    }

    uint8_t skip_covariance_update = 0;
    if (ekf->chi_square > ekf->chi_square_threshold && ekf->converge_flag) {
        if (ekf->stable_flag) {
            ekf->error_count++;
        } else {
            ekf->error_count = 0;
        }

        if (ekf->error_count > 50U) {
            ekf->converge_flag = 0;
        } else {
            finish_prediction_only(ekf);
            skip_covariance_update = 1;
        }
    } else {
        if (ekf->chi_square > 0.1f * ekf->chi_square_threshold && ekf->converge_flag) {
            ekf->adaptive_gain_scale =
                (ekf->chi_square_threshold - ekf->chi_square) /
                (0.9f * ekf->chi_square_threshold);
        } else {
            ekf->adaptive_gain_scale = 1.0f;
        }
        ekf->error_count = 0;
    }

    if (!skip_covariance_update) {
        float pht[18];
        mat_mul(ekf->p_minus, 6, 6, ht, 3, pht);
        mat_mul(pht, 6, 3, inv_s, 3, ekf->k);

        const float orientation_cosine[3] = {
            acosf(clampf_local(fabsf(hq[0]), 0.0f, 1.0f)),
            acosf(clampf_local(fabsf(hq[1]), 0.0f, 1.0f)),
            acosf(clampf_local(fabsf(hq[2]), 0.0f, 1.0f)),
        };

        for (uint8_t i = 0; i < 18; ++i) {
            ekf->k[i] *= ekf->adaptive_gain_scale;
        }
        for (uint8_t i = 4; i < 6; ++i) {
            const float scale = orientation_cosine[i - 4] / (ATTITUDE_EKF_PI * 0.5f);
            for (uint8_t j = 0; j < 3; ++j) {
                ekf->k[i * 3 + j] *= scale;
            }
        }

        float dx[6];
        mat_mul(ekf->k, 6, 3, residual, 1, dx);

        if (ekf->converge_flag) {
            for (uint8_t i = 4; i < 6; ++i) {
                const float limit = 1.0e-2f * ekf->dt;
                dx[i] = clampf_local(dx[i], -limit, limit);
            }
        }

        dx[3] = 0.0f;
        for (uint8_t i = 0; i < 6; ++i) {
            ekf->x[i] = ekf->x_minus[i] + dx[i];
        }

        float kh[36];
        float khp[36];
        mat_mul(ekf->k, 6, 3, ekf->h, 6, kh);
        mat_mul(kh, 6, 6, ekf->p_minus, 6, khp);
        for (uint8_t i = 0; i < 36; ++i) {
            ekf->p[i] = ekf->p_minus[i] - khp[i];
        }
    }

    update_outputs(ekf);
    ekf->update_count++;
}
