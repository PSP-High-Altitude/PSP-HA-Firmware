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
// does time come in as seconds or ns ?

//////////// NOTES ON MATRICES ////////////
// The mat struct (arm_matrix_instance_f32) contains the number of rows, number
// of cols, and pointer to data. It does not allocate the space for data (see
// mat_alloc for that)
// i = row
// j = col
// mat[i,j] = mat->pData[i*mat->numCols + j]
/* https://arm-software.github.io/CMSIS_5/DSP/html/group__groupMatrix.html */
// In the functions below, `const` is used to show arguments that aren't
// modified, but doesn't actually prevent the data in the matrices from being
// modified
///////////////////////////////////////////

// STRUCT DEFINITIONS
typedef struct {
    mat* x;
    mat* P;
    /* data */
} KfState;

// change these if we want diferent sized floats. You also must change the math
// functions
typedef arm_matrix_instance_f32 mat;
typedef float32_t mfloat;

// MATRIX FUNCTIONS
arm_status mat_copy(const mat* from, mat* to);
arm_status mat_edit(mat* mat_ptr, uint16_t i, uint16_t j, mfloat value);
void mat_diag(mat* mat_ptr, const mfloat* diag_vals, bool zeros);
int mat_size(mat* m);
void mat_alloc(mat* mat_ptr, uint16_t rows, uint16_t cols);
/**
 * @brief out = (A * B) * C. Make sure the dimensions work
 *
 * @param A
 * @param B
 * @param C
 * @param out Place to put result. Can be A or B, but NOT C!
 * @return arm_status
 */
arm_status mat_doubleMultiply(const mat* A, const mat* B, const mat* C,
                              mat* out);
/**
 * @brief out = A * B * A'.
 *
 * @param A m x n matrix
 * @param B n x n square matrix
 * @param out Place to put result. Can be A or B.
 * @return arm_status
 */
arm_status mat_transposeMultiply(const mat* A, const mat* B, mat* out);

/**
 * @brief Scales a matrix in place
 *
 * @param A Matrix to be scales
 * @param scale Value to scale by
 */
void mat_scale(mat* A, mfloat scale);

/**
 * @brief A += B; A and B must have same dimensions
 *
 * @param A
 * @param B
 * @return arm_status
 */
arm_status mat_addTo(mat* A, const mat* B);

// KF FUNCTIONS
void kf_init_mats();
void kf_init_state(int num_states, const mfloat* x0, const mfloat* P0);

bool kf_do_kf(StateEst* state_ptr, FlightPhase phase,
              const SensorFrame* sensor_frame);

void kf_predict(KfState* state_ptr, int dt, const mfloat* w);

void kf_update(KfState* state_ptr, const SensorFrame* sensor_frame,
               const mfloat* z, const mfloat* R_diag);

void kf_preprocess(const SensorFrame* sensor_frame, KfState* state_ptr,
                   FlightPhase phase);

void kf_Q_matrix(int dt);                   // process noise matrix
void kf_F_matrix(int dt);                   // state update matrix (linearized)
void kf_R_matrix(const mfloat* meas_vars);  // measurement noise matrix
void kf_H_matrix(const mat* x);             // measurement matrix (linearized)
/**
 * @brief
 *
 * @param x
 * @param dt
 * @param w
 */
void kf_fx(mat* x, int dt, const mat* w);  // state update function
/**
 * @brief Converts state to measurement frame (not linearized)
 *
 * @param x State mat (full state)
 * @param z Measurements vec (not rotation)
 * @param out Output, State in measurement frame (excluding rotation)
 */
void kf_hx(mat* x, mfloat* z, mat* out);  // measurement function

#endif  // KALMAN_H
