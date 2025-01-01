#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"
#include "flight_control.h"
#include "math.h"
#include "state_estimation.h"
#include "stdlib.h"

#define NUM_KIN_STATES (3)  // non-rotational states (h,v,a)
#define NUM_ROT_STATES (4)  // rotation states (quaternion)
#define NUM_KIN_MEAS (3)
#define NUM_ROT_MEAS (3)
#define NUM_TOT_STATES (7)      /** total states */
#define TIME_CONVERSION (1E6f)  //
#define G (9.81f)
#define SEA_LEVEL_PRESSURE (1013.25f) /** milibars */

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
typedef enum {  // TODO: Add more error types
    KF_SUCCESS = 0,
    KF_ERROR = 1,
} kf_status;

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

void mat_findNans(const mfloat* pData, int size, bool* out);

void mat_setSize(mat* mat, int rows,
                 int cols); /** Be careful, this doesn't resize memory! */

mfloat mat_val(mat* mat, int i, int j);

int mat_boolSum(bool* vec, int size);

// KF FUNCTIONS
void kf_init_mats();
void kf_free_mats();
void kf_init_state(int num_states, const mfloat* x0, const mfloat* P0);

kf_status kf_do_kf(StateEst* state_ptr, FlightPhase phase,
                   const SensorFrame* sensor_frame);

/**
 * @brief
 *
 * @param state_ptr
 * @param dt
 * @param w
 * @return kf_status
 */
kf_status kf_predict(mfloat dt, const mfloat* w);

/**
 * @brief
 *
 * @param state_ptr
 * @param sensor_frame
 * @param z
 * @param R_diag
 * @return kf_status
 */
kf_status kf_update(const mfloat* z, const mfloat* R_diag);

kf_status kf_preprocess(mfloat* z, mfloat* R_diag, FlightPhase phase);

void kf_Q_matrix(mfloat dt);  // process noise matrix
void kf_F_matrix(mfloat dt);  // state update matrix (linearized)
void kf_R_matrix(const mfloat* meas_vars,
                 const mfloat* z);  // measurement noise matrix
void kf_H_matrix(const mat* x,
                 const mfloat* z);  // measurement matrix (linearized)
/**
 * @brief
 *
 * @param x
 * @param dt
 * @param w
 */
void kf_fx(mat* x, mfloat dt, const mfloat* w);  // state update function

/**
 * @brief Converts state to measurement frame (not linearized)
 *
 * @param x State mat (full state)
 * @param z Measurements vec (not rotation)
 * @param out Output, State in measurement frame (excluding rotation)
 */
void kf_hx(mat* x, mfloat* z, mat* out);  // measurement function

void kf_resid(
    const mat* x, const mfloat* z,
    mat* out);  // Computes residual y = z - h(x), size adjusted for NaNs

mfloat kf_altToPressure(mfloat alt);

#endif  // KALMAN_H
