#include "stateestim.h"

#include "math.h"

#define PI 3.14159265359
#define G 9.80665

static Quaternion quat_step(Quaternion q, Vector w, float dt) {
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

static Vector quat_rot(Vector v, Quaternion q) {
    return vadd(vadd(vscale(q.v, 2 * vdot(q.v, v)),
                     vscale(v, q.w * q.w - vdot(q.v, q.v))),
                vscale(vcross(q.v, v), 2 * q.w));
}

static Vector trap_int(Vector vint, Vector v, Vector vprev, float dt) {
    return vadd(vint, vscale(vadd(v, vprev), dt * .5));
}

/// @brief Converts 321 Euler angles (in degrees) to quaternion
/// @param roll Roll (in degrees)
/// @param pitch Pitch (in degrees)
/// @param yaw Yaw (in degrees)
/// @return
Quaternion deg_eul_to_quat(float roll, float pitch, float yaw) {
    double cr = cos(roll * PI / 360);
    double sr = sin(roll * PI / 360);
    double cp = cos(pitch * PI / 360);
    double sp = sin(pitch * PI / 360);
    double cy = cos(yaw * PI / 360);
    double sy = sin(yaw * PI / 360);
    Quaternion out = {.w = cr * cp * cy + sr * sp * sy,
                      .v = {.x = sr * cp * cy - cr * sp * sy,
                            .y = cr * sp * cy + sr * cp * sy,
                            .z = cr * cp * sy - sr * sp * cy}};
    return out;
}

/// @brief Updates current inertial rotation, acceleration, velocity, and
/// position using current body acceleration and angular velocity.
/// @param v_a Current body acceleration vector (m/s^2)
/// @param v_w Current body angular velocity (in rad/s)
/// @param dt Time between measurements (in s)
/// @param *v_ac Pointer to inertial acceleration vector (m/s^2, updated in
/// loop)
/// @param *v_v Pointer to inertial velocity vector (in m/s, updated in loop)
/// @param *v_d Pointer to inertial position vector (in m, updated in loop)
/// @param *q_rot Pointer to stored rotation quaternion (updated in loop)
void update_pose(Vector v_a, Vector v_w, float dt, Vector *v_ac, Vector *v_v,
                 Vector *v_d, Quaternion *q_rot) {
    // Update the rotational state quaternion using angular velocity
    *q_rot = quat_step(*q_rot, v_w, dt);

    // Save the previous inertial acceleration for integration
    Vector aprev = *v_ac;
    // Correct current acceleration to the inertial frame
    *v_ac = quat_rot(v_a, *q_rot);
    v_ac->z = v_ac->z - G;

    // Save the previous inertial velocity for integration
    Vector vprev = *v_v;
    // Integrate acceleration trapezoidally and save to velocity
    *v_v = trap_int(*v_v, *v_ac, aprev, dt);
    // Integrate velocity trapezoidally and save to position
    *v_d = trap_int(*v_d, *v_v, vprev, dt);
}