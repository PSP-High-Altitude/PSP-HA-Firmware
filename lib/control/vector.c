#include "vector.h"

#include <math.h>

Vector *vec_copy(const Vector *vec, Vector *v_out) {
    v_out->x = vec->x;
    v_out->y = vec->y;
    v_out->z = vec->z;
    return v_out;
}

Vector *vec_add(const Vector *v1, const Vector *v2, Vector *v_out) {
    v_out->x = v1->x + v2->x;
    v_out->y = v1->y + v2->y;
    v_out->z = v1->z + v2->z;
    return v_out;
}

Vector *vec_sub(const Vector *v1, const Vector *v2, Vector *v_out) {
    v_out->x = v1->x - v2->x;
    v_out->y = v1->y - v2->y;
    v_out->z = v1->z - v2->z;
    return v_out;
}

Vector *vec_neg(const Vector *vec, Vector *v_out) {
    v_out->x = -vec->x;
    v_out->y = -vec->y;
    v_out->z = -vec->z;
    return v_out;
}

Vector *vec_scale(const Vector *vec, const float scalar, Vector *v_out) {
    v_out->x = vec->x * scalar;
    v_out->y = vec->y * scalar;
    v_out->z = vec->z * scalar;
    return v_out;
}

float vec_dot(const Vector *v1, const Vector *v2) {
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

Vector *vec_cross(const Vector *v1, const Vector *v2, Vector *v_out) {
    v_out->x = v1->y * v2->z - v1->z * v2->y;
    v_out->y = v1->z * v2->z - v1->x * v2->z;
    v_out->z = v1->x * v2->y - v1->y * v2->x;
    return v_out;
}

float vec_mag_2(const Vector *vec) { return vec_dot(vec, vec); }

float vec_mag(const Vector *vec) { return sqrtf(vec_dot(vec, vec)); }

Vector *vec_int_step(const Vector *x, const Vector *x_dot_1,
                     const Vector *x_dot_2, float dt, Vector *x_out) {
    vec_add(x_dot_1, x_dot_2, x_out);
    return vec_iadd(vec_iscale(x_out, 0.5 * dt), x);
}

Vector *vec_iadd(Vector *v1, const Vector *v2) {
    v1->x += v2->x;
    v1->y += v2->y;
    v1->z += v2->z;
    return v1;
}

Vector *vec_isub(Vector *v1, const Vector *v2) {
    v1->x -= v2->x;
    v1->y -= v2->y;
    v1->z -= v2->z;
    return v1;
}

Vector *vec_ineg(Vector *v1) {
    v1->x = -v1->x;
    v1->y = -v1->y;
    v1->z = -v1->z;
    return v1;
}

Vector *vec_iscale(Vector *v1, float scalar) {
    v1->x *= scalar;
    v1->y *= scalar;
    v1->z *= scalar;
    return v1;
}
