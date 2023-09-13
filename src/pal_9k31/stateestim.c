#include "stateestim.h"

static Quaternion QuatStep(Quaternion q, Vector w, float dt) {
    Quaternion out = {
        .w = dt * (-w.x * q.v.x - w.y * q.v.y - w.z * q.v.z + q.w) / 2,
        .v = {.x = dt * (w.x * q.w - w.y * q.v.z + w.z * q.v.y + q.v.x) / 2,
              .y = dt * (w.x * q.v.z + w.y * q.w - w.z * q.v.x + q.v.y) / 2,
              .z = dt * (-w.x * q.v.y + w.y * q.v.x + w.z * q.w + q.v.z) / 2}};
    return out;
}

static float vdot(Vector v1, Vector v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

static Vector vcross(Vector v1, Vector v2) {
    Vector vout = {.x = v1.y * v2.z - v1.z * v2.y,
                   .y = v1.z * v2.x - v1.x * v2.z,
                   .z = v1.x * v2.y - v1.y * v2.x};
    return vout;
}

static Vector vscale(Vector v, float k) {
    Vector vout;
    vout.x = v.x * k;
    vout.y = v.y * k;
    vout.z = v.z * k;
    return vout;
}

static Vector vadd(Vector v1, Vector v2) {
    Vector vout = {.x = v1.x + v2.x, .y = v1.y + v2.y, .z = v1.z + v2.z};
    return vout;
}

static Vector QuatRot(Vector v, Quaternion q) {
    return vadd(vadd(vscale(q.v, 2 * vdot(q.v, v)),
                     vscale(v, q.w * q.w - vdot(q.v, q.v))),
                vscale(vcross(q.v, v), 2 * q.w));
}

static Vector TrapInt(Vector vint, Vector v, Vector vprev, float dt) {
    return vadd(vint, vscale(vadd(v, vprev), dt * .5));
}

void UpdatePose(Vector v_a, Vector *v_ac, Vector *v_v, Vector *v_d, Vector v_w,
                Quaternion *q, float dt) {
    // Update the rotational state quaternion using angular velocity
    *q = QuatStep(*q, v_w, dt);

    // Save the previous inertial acceleration for integration
    Vector aprev = *v_ac;
    // Correct current acceleration to the inertial frame
    *v_ac = QuatRot(v_a, *q);

    // Save the previous inertial velocity for integration
    Vector vprev = *v_v;
    // Integrate acceleration trapezoidally and save to velocity
    *v_v = TrapInt(*v_v, *v_ac, aprev, dt);
    // Integrate velocity trapezoidally and save to position
    *v_d = TrapInt(*v_d, *v_v, vprev, dt);
}