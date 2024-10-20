#ifndef VECTOR_H
#define VECTOR_H

typedef struct {
    float x, y, z;
} Vector;

#define VEC_ZERO ((Vector){0, 0, 0})

Vector *vec_copy(const Vector *vec, Vector *v_out);
Vector *vec_add(const Vector *v1, const Vector *v2, Vector *v_out);
Vector *vec_sub(const Vector *v1, const Vector *v2, Vector *v_out);
Vector *vec_neg(const Vector *vec, Vector *v_out);
Vector *vec_scale(const Vector *vec, const float scalar, Vector *v_out);
float vec_dot(const Vector *v1, const Vector *v2);
Vector *vec_cross(const Vector *v1, const Vector *v2, Vector *v_out);
float vec_mag_2(const Vector *vec);
float vec_mag(const Vector *vec);
Vector *vec_int_step(const Vector *x, const Vector *x_dot_1,
                     const Vector *x_dot_2, float dt, Vector *x_out);

Vector *vec_iadd(Vector *v1, const Vector *v2);
Vector *vec_isub(Vector *v1, const Vector *v2);
Vector *vec_ineg(Vector *v1);
Vector *vec_iscale(Vector *v1, float scalar);
#endif