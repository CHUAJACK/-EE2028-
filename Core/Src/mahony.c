/*
 * mahony.c
 *
 *  Created on: Oct 13, 2026
 *      Author: jp16
 */

#include "mahony.h"
#include <math.h>
#include <stddef.h>


void mahony_init(mahony_t *f, float kp, float ki)
{
    f->q0 = 1.0f; f->q1 = 0.0f; f->q2 = 0.0f; f->q3 = 0.0f;
    f->kp = kp;
    f->ki = ki;
    f->ix = f->iy = f->iz = 0.0f;
    f->acc_gate = 0.15f;    /* TODO: tune for your vibration level */
}

void mahony_update(mahony_t *f,
                   const float gyro[3],
                   const float acc[3],    // value passed in is in g/ms^2
                   const float mag[3],
                   float dt)
{
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    /* Repeated quaternion products */
    const float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    const float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
    const float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

    /* Estimated gravity direction in body frame (half-scaled) */
    const float vx = q1q3 - q0q2;
    const float vy = q0q1 + q2q3;
    const float vz = q0q0 - 0.5f + q3q3;

    /* Accumulated orientation error (body frame, half-scaled) */
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    /* ---- Accelerometer: corrects roll/pitch ---- */
    float ax = acc[0], ay = acc[1], az = acc[2];
    float anorm = sqrtf(ax*ax + ay*ay + az*az);
    if (anorm > 1e-6f && fabsf(anorm - 1.0f) < f->acc_gate) {
        const float inv = 1.0f / anorm;
        ax *= inv; ay *= inv; az *= inv;

        ex += ay*vz - az*vy;    /* cross(a, v) */
        ey += az*vx - ax*vz;
        ez += ax*vy - ay*vx;
    }

    /* ---- Magnetometer: corrects yaw ---- */
    if (mag != NULL) {
        float mx = mag[0], my = mag[1], mz = mag[2];
        float mnorm = sqrtf(mx*mx + my*my + mz*mz);
        if (mnorm > 1e-6f) {
            const float inv = 1.0f / mnorm;
            mx *= inv; my *= inv; mz *= inv;

            /* Rotate measured field into the earth frame, then collapse the
             * horizontal part to a single axis (bx) so only the field's
             * inclination (bz) and magnitude matter, not magnetic north. */
            const float hx = 2.0f*(mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
            const float hy = 2.0f*(mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
            const float bx = sqrtf(hx*hx + hy*hy);
            const float bz = 2.0f*(mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

            /* Estimated field direction in body frame (half-scaled) */
            const float wx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
            const float wy = bx*(q1q2 - q0q3)        + bz*(q0q1 + q2q3);
            const float wz = bx*(q0q2 + q1q3)        + bz*(0.5f - q1q1 - q2q2);

            float mex = my*wz - mz*wy;  /* cross(m, w) */
            float mey = mz*wx - mx*wz;
            float mez = mx*wy - my*wx;

#ifdef MAHONY_MAG_YAW_ONLY
            /* Project the mag error onto the estimated vertical axis */
            const float d = 4.0f*(mex*vx + mey*vy + mez*vz);
            mex = d*vx; mey = d*vy; mez = d*vz;
#endif
            ex += mex; ey += mey; ez += mez;
        }
    }

    /* ---- PI feedback into the gyro ---- */
    if (f->ki > 0.0f) {
        f->ix += 2.0f * f->ki * ex * dt;
        f->iy += 2.0f * f->ki * ey * dt;
        f->iz += 2.0f * f->ki * ez * dt;
        /* TODO: clamp ix/iy/iz to a sane bias range (e.g. +/- 0.1 rad/s) */
        gx += f->ix; gy += f->iy; gz += f->iz;
    } else {
        f->ix = f->iy = f->iz = 0.0f;
    }
    gx += 2.0f * f->kp * ex;
    gy += 2.0f * f->kp * ey;
    gz += 2.0f * f->kp * ez;

    /* ---- Integrate quaternion rate ---- */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    const float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    /* Normalize */
    const float n = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    f->q0 = q0*n; f->q1 = q1*n; f->q2 = q2*n; f->q3 = q3*n;
}

void mahony_get_euler(const mahony_t *f, float *roll, float *pitch, float *yaw)
{
    const float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;
    float s = 2.0f*(q0*q2 - q3*q1);
    if (s >  1.0f) s =  1.0f;   /* guard asinf against rounding */
    if (s < -1.0f) s = -1.0f;

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2))*180/(3.1415926535);
    *pitch = asinf(s)*180/(3.1415926535);
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3))*180/(3.1415926535);
}
