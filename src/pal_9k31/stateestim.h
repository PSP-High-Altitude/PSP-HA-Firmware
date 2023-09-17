#ifndef STATEESTIM_H
#define STATEESTIM_H

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
Vector QuatRot(Vector v, Quaternion qn);
Vector TrapInt(Vector vint, Vector v, Vector vprev, float dt);

Quaternion DegEul2Quat(float roll, float pitch, float yaw);
void UpdatePose(Vector v_a, Vector v_w, float dt, Vector *v_ac, Vector *v_v,
                Vector *v_d, Quaternion *q_rot);
#endif