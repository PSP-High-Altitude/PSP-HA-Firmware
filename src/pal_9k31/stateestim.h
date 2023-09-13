#ifndef STATEESTIM_H
typedef struct {
    float x;
    float y;
    float z;
} Vector;

typedef struct {
    float w;
    Vector v;
} Quaternion;

static Quaternion QuatStep(Quaternion, Vector, float);
static float vdot(Vector, Vector);
static Vector vcross(Vector, Vector);
static Vector vscale(Vector, float);
static Vector vadd(Vector, Vector);
static Vector QuatRot(Vector, Quaternion);
static Vector TrapInt(Vector, Vector, Vector, float);

void UpdatePose(Vector, Vector*, Vector*, Vector*, Vector, Quaternion*, float);
#endif