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
static mat S;          // ??? (intermediate update step)
static mat K;          // Kalman Gain (intermediate update step)
// static mat z;          // measurements excluding rotation
// static mat w;          // angular velocity measurements

static mfloat Q_vars = {1., 1., 1., 1., 1., 1., 1.};

// MATRIX DEFINITIONS

void mat_alloc(mat* mat_ptr, uint16_t rows, uint16_t cols) {
    // allocates memory for the matrix
    // yes it's malloc but idk a better way, if someone else does feel free to
    // change it
    arm_mat_init_f32(mat_ptr, rows, cols, malloc(sizeof(mfloat) * rows * cols));
}

arm_status mat_edit(mat* mat_ptr, uint16_t i, uint16_t j, mfloat value) {
    mat_ptr->pData[i * mat_ptr->numCols + j] = value;
    return ARM_MATH_SUCCESS;
}

arm_status mat_copy(const mat* from, mat* to) {
    // could use arm_copy_f32() instead
    if (mat_size(from) == mat_size(to)) {
        memcpy(to, from, (mat_size(from) * sizeof(mfloat)));
        return ARM_MATH_SUCCESS;
    } else {
        return ARM_MATH_SIZE_MISMATCH;
    }
}

arm_status mat_doubleMultiply(const mat* A, const mat* B, const mat* C,
                              mat* out) {
    // TODO: add size check
    mfloat* tempspace[A->numRows * B->numCols];  // temporary space
    mat temp = {A->numRows, B->numCols, &tempspace};
    arm_mat_mult_f32(A, B, &temp);
    arm_mat_mult_f32(&temp, C, out);
    return ARM_MATH_SUCCESS;
}

arm_status mat_transposeMultiply(const mat* A, const mat* B, mat* out) {
    // A*B*A'
    // TODO: add size/shape check
    mfloat* At_space[mat_size(A)];  // A transpose
    mat At = {A->numCols, A->numRows, &At_space};
    arm_mat_trans_f32(A, &At);
    return mat_doubleMultiply(A, B, &At, out);
}

void mat_scale(mat* A, mfloat scale) {
    int n = sizeof(A);
    for (int i = 0; i < n; i++) {
        (A->pData)[i] *= scale;
    }
}

arm_status mat_addTo(mat* A, const mat* B) {
    if (A->numCols == B->numCols && A->numRows == B->numRows) {
        int n = mat_size(A);
        for (int i = 0; i < n; i++) {
            (A->pData)[i] += (B->pData)[i];
        }
        return ARM_MATH_SUCCESS;
    } else {
        return ARM_MATH_SIZE_MISMATCH;
    }
}

int mat_size(mat* m) {
    // Number of elements in matrix
    return m->numCols * m->numRows;
}

// KF FUNCTIONS
void kf_init_mats() {
    // there's probably a less mallocy way to do this but for now this should
    // work (The arm_matrix_instance_f32 just points to an array of the data, so
    // it needs to be allocated somewhere)
    mat_alloc(&x, NUM_TOT_STATES, 1);
    mat_alloc(&P, NUM_TOT_STATES, NUM_TOT_STATES);
    mat_alloc(&F, NUM_TOT_STATES, NUM_TOT_STATES);
    mat_alloc(&Q, NUM_TOT_STATES, NUM_TOT_STATES);
    mat_alloc(&H, NUM_KIN_MEAS, NUM_TOT_STATES);
    mat_alloc(&R, NUM_KIN_MEAS, NUM_KIN_MEAS);
    mat_alloc(&y, NUM_KIN_MEAS, 1);
    // mat_alloc(&z, NUM_KIN_MEAS, 1);
    // mat_alloc(&w, 4, 1);  // 4 because quaternions
    mat_alloc(&S, NUM_KIN_MEAS, NUM_KIN_MEAS);
    mat_alloc(&K, NUM_TOT_STATES, NUM_KIN_MEAS);

    state.x = &x;
    state.P = &P;
}

void kf_init_state(int num_states, const mfloat* x0, const mfloat* P0) {
    arm_mat_init_f32(&(state.x), num_states, num_states, x0);
    arm_mat_init_f32(&(state.P), num_states, num_states, P0);
}

void mat_diag(mat* mptr, const mfloat* diag_vals, bool zeros) {
    // puts diag vals on the diagonal of the matrix. If zeros is true it fills
    // the rest with zeros. Otherwise it leaves the other elements alone.
    // TODO: Optimize with arm_fill_f32()
    for (int i = 0; i < mptr->numRows; i++) {
        for (int j = 0; j < mptr->numCols; j++) {
            if (i == j) {
                mat_edit(mptr, i, j, diag_vals[i]);
            } else if (zeros) {
                mat_edit(mptr, i, j, 0);
            }
        }
    }
}

bool kf_do_kf(StateEst* state_ptr, FlightPhase phase,
              const SensorFrame* sensor_frame);

void kf_predict(KfState* state_ptr, int dt, const mfloat* w) {
    // kf_Q_matrix(dt)
    kf_F_matrix(dt);    // update F matrix with dt
    // self.x = self.f(self.x, dt, w=w) # use f(x) for EKF
    kf_fx(&x, dt, w);  // calculate (integrate) new state

    mfloat* temp_storage[mat_size(&Q)];  // temporary space to build Q
    // NOTE: It might be more efficicient to permanently (staticly?) allocate
    // space for temporary arrays that all functions can use
    mat temp_square = {Q.numRows, Q.numCols, &temp_storage};

    // Q = np.diag(self.Q_var*dt)
    mat_diag(&temp_square, &Q_vars, true);  // make Q in temp_square
    arm_mat_scale_f32(&temp_square, dt,
                      &Q);  // scale by dt and copy to static Q

    // P = F @ self.P @ F.T + self.Q
    kf_F_matrix(dt);                              // update F with dt
    mat_transposeMultiply(&F, &P, &temp_square);  // FPF'
    arm_mat_add_f32(&temp_square, &Q, &P);        // P = FPF' + Q
}

void kf_update(KfState* state_ptr, const SensorFrame* sensor_frame,
               const mfloat* z, const mfloat* R_diag) {
    mfloat*
        temp_space[mat_size(&P)];  // space for all temporary matrices used in
                                   // this function (only 1 is needed at a time)
    mat hx = {NUM_KIN_MEAS, 1, &temp_space};

    kf_H_matrix(&x);  // Make H Matrix (linearized h(x))

    // y = z - hx # use h(x) for EKF
    kf_hx(&x, z, &hx);            // hx = h(x) (state to meas frame)
    mat_scale(&hx, -1);           // -hx
    arm_mat_add_f32(z, &hx, &y);  // y = z + -hx

    // S = H @ self.P @ H.T + R # system uncertainty (in measurement space)
    kf_R_matrix(R_diag);                // Make R matrix
    mat_transposeMultiply(&H, &P, &S);  // HPH'
    mat_addTo(&S, &R);                  // S = HPH' + R

    // K = self.P @ H.T @ inv(S) # Kalman Gain
    mat temp = {S.numCols, S.numRows, &temp_space};
    arm_mat_inverse_f32(&S, &temp);                // inv(S)
    mat_copy(&temp, &S);                           // S = inv(S)
    mat Ht = {H.numCols, H.numRows, &temp_space};  // H'
    arm_mat_trans_f32(&H, &Ht);                    // H'
    mat_doubleMultiply(&P, &Ht, &S, &K);           // K = P*H'*inv(S)

    // x += K @ self.y # update state
    mat Ky = {NUM_TOT_STATES, 1, &temp_space};
    arm_mat_mult_f32(&K, &y, &Ky);  // Ky = K*y'
    mat_addTo(&x, &Ky);             // x += Ky

    // P = (np.eye(n) - K @ H) @ self.P @ (np.eye(n) - K @ H).T + K @ R @ K.T
    // ^more numerically stable version of P = P - KHP
    mat KHP = {K.numRows, P.numCols,
               &temp_space};  // NOTE: temp_space might be too small if there
                              // are more measurements than states
    mat_doubleMultiply(&K, &H, &P, &KHP);  // K*H*P
    mat_scale(&KHP, -1);                   // -KHP
    mat_addTo(&P, &KHP);                   // P = P + -KHP
    // TODO: Replace this with the more numerically stable version
}

void kf_preprocess(const SensorFrame* sensor_frame, KfState* state_ptr,
                   FlightPhase phase);

void kf_Q_matrix(int dt);
void kf_F_matrix(int dt) {}
void kf_R_matrix(const mfloat* meas_vars) {
    mat_diag(&R, meas_vars, true);  // Make R matrix
}
void kf_H_matrix(const mat* x);
void kf_fx(mat* x, int dt, const mat* w);  // state update function
void kf_hx(mat* x, mfloat* z, mat* out);   // measurement function