#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"
#include "flight_control.h"
#include "state_estimation.h"

// TODO: Make all the matrices static variables

typedef struct {
    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 P;
    /* data */
} State;

void kf_init_state(int num_states, const float32_t* x0, const float32_t* P0);

void kf_init_diag(arm_matrix_instance_f32* mat_ptr, const float32_t* diag_vals);
bool kf_do_kf(StateEst* state_ptr, FlightPhase phase,
              const SensorFrame* sensor_frame);

void kf_predict(State* state_ptr, int dt);

void kf_update(State* state_ptr, const SensorFrame* sensor_frame,
               const float32_t* R_diag);

void kf_preprocess(const SensorFrame* sensor_frame, State* state_ptr,
                   FlightPhase phase);

arm_matrix_instance_f32 kf_Q_matrix(const float32_t* Q_diag, int dt);
arm_matrix_instance_f32 kf_F_matrix(int dt);
arm_matrix_instance_f32 kf_R_matrix(const float32_t* meas_vars);
arm_matrix_instance_f32 kf_H_matrix();

#endif  // KALMAN_H
