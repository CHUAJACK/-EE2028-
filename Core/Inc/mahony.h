/*
 * mahony.h
 *
 *  Created on: Oct 13, 2026
 *      Author: jp16
 */

#ifndef MAHONY_H
#define MAHONY_H
/*
 * Mahony complementary filter (MARG: gyro + accel + mag).
 *
 * Conventions (all three sensors must be in the SAME right-handed body frame):
 *   - gyro : rad/s   (convert from deg/s first)
 *   - accel: g       (reads +1 g on +Z when the board is flat, Z up)
 *   - mag  : any unit (only direction is used)
 *   - dt   : seconds, measured, not nominal
 *
 * Usage:
 *   Mahony f;
 *   mahony_init(&f, 1.0f, 0.02f);
 *   ...
 *   // every IMU sample:
 *   mahony_update(&f, gyro, acc, mag, dt);   // mag may be NULL -> IMU-only
 *   mahony_get_euler(&f, &roll, &pitch, &yaw);
 */

typedef struct {
    float q0, q1, q2, q3;   /* quaternion (w, x, y, z), body -> world */
    float kp, ki;           /* proportional / integral gains */
    float ix, iy, iz;       /* integral term = gyro bias estimate (rad/s) */
    float acc_gate;         /* accel correction only if | |a| - 1 g | < acc_gate */
} mahony_t;

void mahony_init(mahony_t *f, float kp, float ki);

void mahony_update(mahony_t *f,
                   const float gyro[3],
                   const float acc[3],
                   const float mag[3],   /* NULL = no mag */
                   float dt);

void mahony_get_euler(const mahony_t *f, float *roll, float *pitch, float *yaw);

#endif /* MAHONY_H */
