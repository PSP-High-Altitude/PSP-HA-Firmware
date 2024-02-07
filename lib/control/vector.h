#ifndef VECTOR_H
#define VECTOR_H

typedef struct {
    float x;
    float y;
    float z;
} Vector;

typedef struct {
    float w;
    Vector v;
} Quaternion;

Quaternion QuatStep(Quaternion q, Vector w, float dt);

float vdot(Vector v1, Vector v2);

Vector vcross(Vector v1, Vector v2);

Vector vscale(Vector v, float k);

Vector vadd(Vector v1, Vector v2);

Vector QuatRot(Vector v, Quaternion q);

Vector TrapInt(Vector vint, Vector v, Vector vprev, float dt);

void UpdatePose(Vector v_a, Vector *v_ac, Vector *v_v, Vector *v_d, Vector v_w,
                Quaternion *q, float dt);

Vector zeroVec();

Vector newVec(float x, float y, float z);

Vector createVectorFromStruct(void *ptr);

float vnorm(Vector v);

#endif  // VECTOR_H