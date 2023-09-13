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

void UpdatePose(Vector, Vector*, Vector*, Vector*, Vector, Quaternion*, float);

#endif