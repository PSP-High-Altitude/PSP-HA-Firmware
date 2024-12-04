#include "kalman.h"

#include "arm_math.h"
#include "flight_control.h"
#include "state_estimation.h"

void init_state(State* state_ptr, int num_states, const float32_t* x0,
                const float32_t* P0) {
    arm_mat_init_f32(&(state_ptr->x), num_states, num_states, x0);
    arm_mat_init_f32(&(state_ptr->P), num_states, num_states, P0);
}
