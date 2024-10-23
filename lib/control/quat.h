#ifndef QUAT_H
#define QUAT_H

#include "vector.h"

typedef struct {
    float w, x, y, z;
} Quaternion;

#define QUAT_ZERO (Quaternion{0, 0, 0, 0})

Quaternion *quat_copy(const Quaternion *quat, Quaternion *q_out);
Quaternion *quat_from_vec(const Vector *vec, Quaternion *q_out);
Vector *quat_to_vec(const Quaternion *quat, Vector *v_out);
Quaternion *quat_add(const Quaternion *q1, const Quaternion *q2,
                     Quaternion *q_out);
float quat_dot(const Quaternion *q1, const Quaternion *q2);
float quat_mag_2(const Quaternion *quat);
float quat_mag(const Quaternion *quat);
Quaternion *quat_scale(const Quaternion *quat, float scalar, Quaternion *q_out);
Quaternion *quat_normalize(const Quaternion *quat, Quaternion *q_out);
Quaternion *quat_conj(const Quaternion *quat, Quaternion *q_out);
Vector *quat_rot(const Vector *vec, const Quaternion *quat, Vector *v_out);
Vector *quat_rot_inv(const Vector *vec, const Quaternion *quat, Vector *v_out);
Quaternion *quat_step(const Quaternion *quat_init, const Vector *ang_vel,
                      float dt, Quaternion *q_out);
float quat_angle_from_vertical(const Quaternion *quat);

#endif