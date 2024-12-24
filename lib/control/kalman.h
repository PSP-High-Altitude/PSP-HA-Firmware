#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"
#include "flight_control.h"
#include "state_estimation.h"

#define NUM_KIN_STATES (3)  // non-rotational states (h,v,a)
#define NUM_ROT_STATES (4)  // rotation states (quaternion)
#define NUM_KIN_MEAS (3)
#define NUM_ROT_MEAS (3)
#define NUM_TOT_STATES (7)      // total states,
#define TIME_CONVERSION (1E6f)  //
// does time come in as seconds or ns

//////////// NOTES ON MATRICES ////////////
// The mat struct (arm_matrix_instance_f32) contains the number of rows, number
// of cols, and pointer to data. It does not allocate the space for data (see
// kf_matalloc for that)
// i = row
// j = col
// mat[i,j] = mat->pData[i*mat->numCols + j]
///////////////////////////////////////////

// STRUCT DEFINITIONS
typedef struct {
    mat* x;
    mat* P;
    /* data */
} KfState;

// change these if we want diferent sized floats
typedef arm_matrix_instance_f32 mat;
typedef float32_t mfloat;

// typedef struct {
//     arm_matrix_instance_f32 matrix
//     float32_t[] mat_ptr
// }

void kf_init_mats();
void kf_matalloc(mat* mat_ptr, uint16_t rows, uint16_t cols);
void kf_init_state(int num_states, const mfloat* x0, const mfloat* P0);
void kf_edit_mat(mat* mat_ptr, uint16_t i, uint16_t j, mfloat value);

void kf_diag(mat* mat_ptr, const mfloat* diag_vals, bool zeros);
bool kf_do_kf(StateEst* state_ptr, FlightPhase phase,
              const SensorFrame* sensor_frame);

void kf_predict(KfState* state_ptr, int dt);

void kf_update(KfState* state_ptr, const SensorFrame* sensor_frame,
               const mfloat* R_diag);

void kf_preprocess(const SensorFrame* sensor_frame, KfState* state_ptr,
                   FlightPhase phase);

void kf_Q_matrix(int dt);
void kf_F_matrix(int dt);
void kf_R_matrix(const mfloat* meas_vars);
void kf_H_matrix();
void kf_fx(mat* x, int dt, mat* w);

#endif  // KALMAN_H
