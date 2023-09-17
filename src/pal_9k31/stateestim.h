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

Quaternion deg_eul_to_quat(float roll, float pitch, float yaw);
void update_pose(Vector v_a, Vector v_w, float dt, Vector *v_ac, Vector *v_v,
                 Vector *v_d, Quaternion *q_rot);
#endif