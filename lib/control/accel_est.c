#include "accel_est.h"

// int main() {
//     // Example usage
//     accel_est obj;
//     init_accel_est(&obj, 10);

//     // Simulate updates
//     // update_accel_est(&obj, 1000, -9.81);  // Replace with your actual data
//     // Add more updates as needed

//     // Clean up (free allocated memory, etc.)
//     // ...

//     return 0;
// }

// void init_accel_est(accel_est* obj, int size) {
//     obj->times = (double*)malloc(size * sizeof(double));
//     obj->AccBodyY = (double*)malloc(size * sizeof(double));
//     obj->AccDown = (double*)malloc(size * sizeof(double));
//     obj->VelDown = (double*)malloc(size * sizeof(double));
//     obj->PosDown = (double*)malloc(size * sizeof(double));
//     obj->i = 1;
//     obj->g = -9.81;
//     obj->p = 0;
// }

void update_accel_est(StateEst* state, float dt, Vector up) {
    float a_up = vdot(state->accBody, up) *
                 -1;  // vertical accleration adjusted for g, m/s^2
    // float t;
    // y is up
    // state->times[state->i - 1] = t / 1000.0;  // convert to s

    // state->AccDown[state->i - 1] = -1 * state->AccBodyY[state->i - 1];

    // if (state->i <= 1) {
    //     state->VelDown[state->i - 1] = 0;
    //     state->PosDown[state->i - 1] = 0;
    // } else {
    // double timestep[2] = {state->times[state->i - 2], t / 1000.0};
    state->accNED.z = a_up * -1;
    state->velNED.z += state->accNED.z * dt;
    state->posNED.z += state->velNED.z * dt;
    //     state->VelDown[state->i - 2] +
    //     (timestep[1] *
    //      (state->AccDown[state->i - 2] + state->AccDown[state->i - 1]) / 2);
    // state->PosDown[state->i - 1] =
    //     state->PosDown[state->i - 2] +
    //     (timestep[1] *
    //      (state->VelDown[state->i - 2] + state->VelDown[state->i - 1]) / 2);
    // }

    // state->i++;
}
