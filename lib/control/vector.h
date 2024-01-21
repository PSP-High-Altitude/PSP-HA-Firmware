#ifndef STATEESTIMATION_LIBRARY_H
#define STATEESTIMATION_LIBRARY_H

typedef struct {
    float x;
    float y;
    float z;
} Vector;

typedef struct {
    float w;
    Vector v;
} Quaternion;

Vector v_zero(void);

Vector v_add(Vector, Vector);

Vector v_sub(Vector, Vector);

Vector v_mult(Vector, Vector);

float v_dot(Vector, Vector);

Vector v_cross(Vector, Vector);

float v_mag(Vector);

Vector v_rot(Vector, Quaternion);

Quaternion q_from_v(Vector);

Quaternion q_add(Quaternion, Quaternion);

Quaternion q_sub(Quaternion, Quaternion);

Quaternion q_mult(Quaternion, Quaternion);

Quaternion q_mag(Quaternion);

Quaternion q_step(Quaternion, Vector);

Quaternion q_normal(Quaternion);

#endif //STATEESTIMATION_LIBRARY_H