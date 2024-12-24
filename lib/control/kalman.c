#include "kalman.h"

#include "arm_math.h"
#include "flight_control.h"
#include "state_estimation.h"
#include "stdlib.h"

// MATRICES (STATIC)
static KfState state;  // might not need this
static mat x;          // state
static mat P;          // state covar mat
static mat F;          // state update mat
static mat Q;          // process noise mat
static mat H;          // state to meas mat
static mat R;          // meas noise mat
static mat y;          // residual
static mat z;          // measurements excluding rotation
static mat w;          // angular velocity measurements

static mfloat Q_vars = {1., 1., 1., 1., 1., 1., 1.};

// FUNCTION DEFINITIONS
void init_mats() {
    // there's probably a less mallocy way to do this but for now this should
    // work (The arm_matrix_instance_f32 just points to an array of the data, so
    // it needs to be allocated somewhere)
    kf_matalloc(&x, NUM_TOT_STATES, 1);
    kf_matalloc(&P, NUM_TOT_STATES, NUM_TOT_STATES);
    kf_matalloc(&F, NUM_TOT_STATES, NUM_TOT_STATES);
    kf_matalloc(&Q, NUM_TOT_STATES, NUM_TOT_STATES);
    kf_matalloc(&H, NUM_KIN_MEAS, NUM_TOT_STATES);
    kf_matalloc(&R, NUM_KIN_MEAS, NUM_KIN_MEAS);
    kf_matalloc(&y, NUM_KIN_MEAS, 1);
    kf_matalloc(&z, NUM_KIN_MEAS, 1);
    kf_matalloc(&w, 4, 1);  // 4 because quaternions
    state.x = &x;
    state.P = &P;
}
void kf_matalloc(mat* mat_ptr, uint16_t rows, uint16_t cols) {
    // allocates memory for the matrix
    // yes it's malloc but idk a better way, if someone else does feel free to
    // change it
    arm_mat_init_f32(mat_ptr, rows, cols, malloc(sizeof(mfloat) * rows * cols));
}

void kf_edit_mat(mat* mat_ptr, uint16_t i, uint16_t j, mfloat value) {
    mat_ptr->pData[i * mat_ptr->numCols + j] = value;
}

void kf_init_state(int num_states, const mfloat* x0, const mfloat* P0) {
    arm_mat_init_f32(&(state.x), num_states, num_states, x0);
    arm_mat_init_f32(&(state.P), num_states, num_states, P0);
}

void kf_diag(mat* mptr, const mfloat* diag_vals, bool zeros) {
    // puts diag vals on the diagonal of the matrix. If zeros is true it fills
    // the rest with zeros. Otherwise it leaves the other elements alone.
    for (int i = 0; i < mptr->numRows; i++) {
        for (int j = 0; j < mptr->numCols; j++) {
            if (i == j) {
                kf_edit_mat(mptr, i, j, diag_vals[i]);
            } else if (zeros) {
                kf_edit_mat(mptr, i, j, 0);
            }
        }
    }
}
bool kf_do_kf(StateEst* state_ptr, FlightPhase phase,
              const SensorFrame* sensor_frame);

void kf_predict(KfState* state_ptr, int dt) {
    // kf_Q_matrix(dt)
    kf_F_matrix(dt);    // update F matrix with dt
    kf_fx(&x, dt, &w);  // calculate (integrate) new state

    // REF
    // self.Q = np.diag(self.Q_var*dt)
    // F = self.F(dt)

    // self.x = self.f(self.x, dt, w=w) # use f(x) for EKF
    // self.P = F @ self.P @ F.T + self.Q
}

void kf_update(KfState* state_ptr, const SensorFrame* sensor_frame,
               const mfloat* R_diag);

void kf_preprocess(const SensorFrame* sensor_frame, KfState* state_ptr,
                   FlightPhase phase);

void kf_Q_matrix(int dt);
void kf_F_matrix(int dt) {}
void kf_R_matrix(const mfloat* meas_vars);
void kf_H_matrix();
