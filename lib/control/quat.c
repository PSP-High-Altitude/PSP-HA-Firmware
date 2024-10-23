#include "quat.h"

#include <math.h>

Quaternion* quat_copy(const Quaternion* quat, Quaternion* q_out) {
    q_out->w = quat->w;
    q_out->x = quat->x;
    q_out->y = quat->y;
    q_out->z = quat->z;
    return q_out;
}

Quaternion* quat_from_vec(const Vector* vec, Quaternion* q_out) {
    q_out->w = 0;
    q_out->x = vec->x;
    q_out->y = vec->y;
    q_out->z = vec->z;
    return q_out;
}

Vector* quat_to_vec(const Quaternion* quat, Vector* v_out) {
    v_out->x = quat->x;
    v_out->y = quat->y;
    v_out->z = quat->z;
    return v_out;
}

Quaternion* quat_add(const Quaternion* q1, const Quaternion* q2,
                     Quaternion* q_out) {
    q_out->w = q1->w + q2->w;
    q_out->x = q1->x + q2->x;
    q_out->y = q1->y + q2->y;
    q_out->z = q1->z + q2->z;
    return q_out;
}

float quat_dot(const Quaternion* q1, const Quaternion* q2) {
    return q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;
}

float quat_mag_2(const Quaternion* quat) { return quat_dot(quat, quat); }

float quat_mag(const Quaternion* quat) { return sqrtf(quat_mag_2(quat)); }

Quaternion* quat_scale(const Quaternion* quat, float scalar,
                       Quaternion* q_out) {
    q_out->w = quat->w * scalar;
    q_out->x = quat->x * scalar;
    q_out->y = quat->y * scalar;
    q_out->z = quat->z * scalar;
    return q_out;
}

Quaternion* quat_normalize(const Quaternion* quat, Quaternion* q_out) {
    return quat_scale(quat, 1 / quat_mag(quat), q_out);
}

Quaternion* quat_conj(const Quaternion* quat, Quaternion* q_out) {
    q_out->w = quat->w;
    q_out->x = -quat->x;
    q_out->y = -quat->y;
    q_out->z = -quat->z;
    return q_out;
}

Vector* quat_rot(const Vector* vec, const Quaternion* quat, Vector* v_out) {
    Vector v_quat;
    Vector v_temp_1;
    Vector v_temp_2;
    Vector v_temp_3;
    quat_to_vec(quat, &v_quat);
    vec_scale(&v_quat, 2, &v_temp_1);
    vec_cross(&v_temp_1, vec, &v_temp_2);
    vec_cross(&v_quat, &v_temp_2, &v_temp_1);
    vec_scale(&v_temp_2, quat->w, &v_temp_3);
    vec_add(vec, &v_temp_3, &v_temp_3);
    vec_add(&v_temp_3, &v_temp_1, v_out);
    return v_out;
}

Vector* quat_rot_inv(const Vector* vec, const Quaternion* quat, Vector* v_out) {
    Quaternion q_conj;
    quat_conj(quat, &q_conj);
    return quat_rot(vec, &q_conj, v_out);
}

Quaternion* quat_step(const Quaternion* q_0, const Vector* ang_vel, float dt,
                      Quaternion* q_out) {
    Quaternion q_dot = {
        .w = -q_0->x * ang_vel->x - q_0->y * ang_vel->y - q_0->z * ang_vel->z,
        .x = q_0->w * ang_vel->x + q_0->y * ang_vel->z - q_0->z * ang_vel->y,
        .y = q_0->w * ang_vel->y + q_0->z * ang_vel->x - q_0->x * ang_vel->z,
        .z = q_0->w * ang_vel->z + q_0->x * ang_vel->y - q_0->y * ang_vel->x};
    quat_add(q_0, quat_scale(&q_dot, 0.5f * dt, &q_dot), q_out);
    return q_out;
}

float quat_angle_from_vertical(const Quaternion* quat) {
    return acosf((powf(quat->w, 2) + powf(quat->x, 2)) -
                 (powf(quat->y, 2) + powf(quat->z, 2)));
}
