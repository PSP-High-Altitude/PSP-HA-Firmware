#include "kalman.h"

// #include "arm_math.h"
// #include "flight_control.h"
#include "math.h"
// #include "state_estimation.h"
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
static mat w;  // angular velocity measurements

// static mfloat Q_vars[] = {1., 1., 1., 1., 1., 1., 1.};

static mfloat Q_vars[NUM_TOT_STATES];  // get set in preprocess
static mfloat R_diag[NUM_KIN_MEAS];    // get set in preprocess

static mfloat time = -1;  // in seconds

static arm_status math_status;   // Watch this for math debugging
static kf_status filter_status;  // Watch this for kf debugging

// TODO: Better place for all this config stuff
// These copied from the values I used during python testing
static const mfloat Q_VARS_1[NUM_TOT_STATES] = {
    5.0e-01, 1.6e-03, 4.0e+00, 1.0e-01, 1.0e-01, 1.0e-01, 1.0e-01};
static const mfloat Q_VARS_2[NUM_TOT_STATES] = {10., 1.,  1., 0.1,
                                                0.1, 0.1, 0.1};
static const mfloat R_DIAG_1[NUM_KIN_MEAS] = {5.e-01, 3.e-04, 3.e-04};
static const mfloat R_DIAG_2[NUM_KIN_MEAS] = {0.5, 1., 1.};
// static const int up_axis = 1;

// MATRIX DEFINITIONS

void mat_alloc(mat* mat_ptr, uint16_t rows, uint16_t cols) {
    // allocates memory for the matrix
    // yes it's malloc but idk a better way, if someone else does feel free to
    // change it
    // arm_mat_init_f32(mat_ptr, rows, cols, malloc(sizeof(mfloat) * rows *
    // cols));
    mat_ptr->numRows = rows;
    mat_ptr->numCols = cols;
    mat_ptr->pData = malloc(sizeof(mfloat) * rows * cols);
}

arm_status mat_edit(mat* mat_ptr, uint16_t i, uint16_t j, mfloat value) {
    mat_ptr->pData[i * mat_ptr->numCols + j] = value;
    return ARM_MATH_SUCCESS;
}

arm_status mat_copy(const mat* from, mat* to) {
    if (mat_size(from) == mat_size(to)) {
        arm_copy_f32(from->pData, to->pData, mat_size(from));
        return ARM_MATH_SUCCESS;
    } else {
        return ARM_MATH_SIZE_MISMATCH;
    }
}

arm_status mat_doubleMultiply(const mat* A, const mat* B, const mat* C,
                              mat* out) {
    mfloat tempspace[A->numRows * B->numCols];  // temporary space
    mat temp = {A->numRows, B->numCols, tempspace};
    arm_status status;
    status = arm_mat_mult_f32(A, B, &temp);
    if (status != ARM_MATH_SUCCESS) {
        return status;
    }
    status = arm_mat_mult_f32(&temp, C, out);
    return status;
}

arm_status mat_transposeMultiply(const mat* A, const mat* B, mat* out) {
    // A*B*A'
    mfloat At_space[mat_size(A)];  // A transpose
    mat At = {A->numCols, A->numRows, At_space};
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

int mat_size(const mat* m) {
    // Number of elements in matrix
    return m->numCols * m->numRows;
}

int mat_findNans(const mfloat* pData, int size, bool* out) {
    int count = 0;
    for (int i = 0; i < size; i++) {
        out[i] = isnan(pData[i]);
        if (isnan(pData[i])) {
            count++;
        }
    }
    return count;
}

mfloat mat_val(const mat* mat, int i, int j) {
    return mat->pData[i * mat->numCols + j];
}

void mat_diag(mat* mptr, const mfloat* diag_vals, bool zeros) {
    // puts diag vals on the diagonal of the matrix. If zeros is true it fills
    // the rest with zeros. Otherwise it leaves the other elements alone.
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

void mat_setSize(mat* mat, int rows, int cols) {
    mat->numRows = rows;
    mat->numCols = cols;
}

int mat_boolSum(bool* vec, int size) {
    int count = 0;
    for (int i = 0; i < size; i++) {
        if (vec[i]) {
            count++;
        }
    }
    return count;
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
    mat_alloc(&w, 3, 1);  //
    mat_alloc(&S, NUM_KIN_MEAS, NUM_KIN_MEAS);
    mat_alloc(&K, NUM_TOT_STATES, NUM_KIN_MEAS);

    state.x = &x;
    state.P = &P;
}

void kf_free_mats() {
    free(x.pData);
    free(P.pData);
    free(F.pData);
    free(Q.pData);
    free(H.pData);
    free(R.pData);
    free(y.pData);
    free(S.pData);
    free(K.pData);
}

void kf_init_state(const mfloat* x0, const mfloat* P0_diag) {
    mat_setSize(&x, NUM_TOT_STATES, 1);
    arm_copy_f32(x0, x.pData, mat_size(&x));
    mat_setSize(&P, NUM_TOT_STATES, NUM_TOT_STATES);
    mat_diag(&P, P0_diag, true);
}

kf_status kf_do_kf(void* state_ptr, FlightPhase phase,
                   const SensorFrame* sensor_frame) {
    mfloat dt;
    if (time > 0) {
        dt = sensor_frame->timestamp / TIME_CONVERSION - time;
    } else {
        dt = 0;
    }
    time = sensor_frame->timestamp / TIME_CONVERSION;

    // measurments
    // TODO: Get the correct up axis - using y for now from skyshot test
    mfloat z[NUM_KIN_MEAS] = {sensor_frame->pressure, sensor_frame->acc_h_y,
                              sensor_frame->acc_h_y};
    // TODO: How does up axis affect oreintation? Do they need reordered?
    mfloat w_meas[NUM_ROT_MEAS] = {
        sensor_frame->rot_i_x, sensor_frame->rot_i_y,
        sensor_frame->rot_i_z};  // also check the order of these

    // Use static w so that old w is saved in case of a bad measurement
    // update w if there are no nans in measurement. Otherwise keep old value
    bool nans[3];
    if (!mat_findNans(w_meas, 3, nans)) {
        arm_copy_f32(w_meas, w.pData, 3);
    }

    // DO FILTER!
    kf_preprocess(z, R_diag, phase);  // adjust measurements and vars based on
                                      // current phase and state
    kf_predict(dt, w.pData);          // No NaNs csn go in here!
    kf_update(z, R_diag);             // z can contain NaNs

    // TODO: Error checking
    // These don't do anything now just had to make them used.
    math_status = ARM_MATH_SUCCESS;
    filter_status = KF_SUCCESS;
    return KF_SUCCESS;
}

kf_status kf_predict(mfloat dt, const mfloat* w) {
    // kf_Q_matrix(dt)
    kf_F_matrix(dt);  // update F matrix with dt. Do this before f(x)!!!
    // self.x = self.f(self.x, dt, w=w) # use f(x) for EKF
    kf_fx(&x, dt, w);  // calculate (integrate) new state

    mfloat temp_storage[mat_size(&Q)];  // temporary space to build Q
    // NOTE: It might be more efficicient to permanently (staticly?) allocate
    // space for temporary arrays that all functions can use
    mat temp_square = {Q.numRows, Q.numCols, temp_storage};

    // Q = np.diag(self.Q_var*dt)
    mat_diag(&Q, Q_vars, true);  // Make Q from Q_vars
    mat_scale(&Q, dt);           // multiply by dt

    // P = F @ self.P @ F.T + self.Q
    kf_F_matrix(dt);                              // update F with dt
    mat_transposeMultiply(&F, &P, &temp_square);  // FPF'
    arm_mat_add_f32(&temp_square, &Q, &P);        // P = FPF' + Q

    return KF_SUCCESS;
}

kf_status kf_update(const mfloat* z, const mfloat* R_diag) {
    kf_status status;
    mfloat
        temp_space[mat_size(&P)];  // space for all temporary matrices used in
                                   // this function (only 1 is needed at a time)
    // mat hx = {NUM_KIN_MEAS, 1, &temp_space};

    status = kf_H_matrix(&x, z);  // Make H Matrix (linearized h(x)), resized to
                                  // exclude NaN measurments
    if (status == KF_NO_VALID_MEAS) {
        return status;
    }
    kf_resid(&x, z, &y);  // Calc residual, resized to exclude NaN measurements

    // S = H @ self.P @ H.T + R # system uncertainty (in measurement space)
    kf_R_matrix(R_diag, z);                 // Make R matrix, resized for NaNs
    mat_setSize(&S, R.numRows, R.numCols);  // Resize S to match R
    mat_transposeMultiply(&H, &P, &S);      // HPH'
    mat_addTo(&S, &R);                      // S = HPH' + R

    // K = self.P @ H.T @ inv(S) # Kalman Gain
    mat_setSize(&K, NUM_TOT_STATES, y.numRows);  // Resize K for NaNs
    mat temp = {S.numCols, S.numRows, temp_space};
    arm_mat_inverse_f32(&S, &temp);               // inv(S)
    mat_copy(&temp, &S);                          // S = inv(S)
    mat Ht = {H.numCols, H.numRows, temp_space};  // H'
    arm_mat_trans_f32(&H, &Ht);                   // H'
    mat_doubleMultiply(&P, &Ht, &S, &K);          // K = P*H'*inv(S)

    // x += K @ self.y # update state
    mat Ky = {NUM_TOT_STATES, 1, temp_space};
    arm_mat_mult_f32(&K, &y, &Ky);  // Ky = K*y'
    mat_addTo(&x, &Ky);             // x += Ky

    // P = (np.eye(n) - K @ H) @ self.P @ (np.eye(n) - K @ H).T + K @ R @ K.T
    // ^more numerically stable version of P = P - KHP
    mat KHP = {K.numRows, P.numCols,
               temp_space};  // NOTE: temp_space might be too small if there
                             // are more measurements than states
    mat_doubleMultiply(&K, &H, &P, &KHP);  // K*H*P
    mat_scale(&KHP, -1);                   // -KHP
    mat_addTo(&P, &KHP);                   // P = P + -KHP
    // TODO: Replace this with the more numerically stable version
    return KF_SUCCESS;
}

kf_status kf_preprocess(mfloat* z, mfloat* R_diag, FlightPhase phase) {
    // TODO: Copying is a bad and inefficient way to do this but I don't want to
    // deal with pointers rn
    if (!(x.pData[1] <
          5)) {  // TODO: This logic shouldn't happen here, for now it's just
                 // for independednt testing and making variables not unused.
        arm_copy_f32(R_DIAG_1, R_diag, NUM_KIN_STATES);
        arm_copy_f32(Q_VARS_1, Q_vars, NUM_TOT_STATES);
    } else {
        arm_copy_f32(R_DIAG_2, R_diag, NUM_KIN_STATES);
        arm_copy_f32(Q_VARS_2, Q_vars, NUM_TOT_STATES);
    }
    // R_diag = R_DIAG_1;
    // Q_vars = Q_VARS_1;

    // TODO: This (post apo stuff)
    return KF_SUCCESS;
}

void kf_Q_matrix(mfloat dt) { mat_diag(&Q, Q_vars, true); }

void kf_F_matrix(mfloat dt) {
    // No rotation
    // NOTE: Remakes entire matrix, could be made more efficient by just
    // updating dt values
    mfloat diag[NUM_TOT_STATES];
    arm_fill_f32(1, diag, NUM_TOT_STATES);  // making identity matrix
    mat_diag(&F, diag, true);
    // upper left: kinematic equations
    mat_edit(&F, 0, 1, dt);
    mat_edit(&F, 0, 2, .5 * dt * dt);
    mat_edit(&F, 1, 2, dt);
}
void kf_R_matrix(const mfloat* meas_vars, const mfloat* z) {
    mat_diag(&R, meas_vars, true);  // Make R matrix
}

kf_status kf_H_matrix(const mat* x, const mfloat* z) {
    mfloat a = 44330;
    mfloat b = 5.25588;
    mfloat h = MAX(mat_val(x, 0, 0), 0);  // mat to ensure alt isn't negative
    mfloat dpdh = (-b * SEA_LEVEL_PRESSURE * pow((a - h), (b - 1))) / pow(a, b);

    // Resize H for NaNs
    bool nans[NUM_KIN_MEAS];
    mat_findNans(z, NUM_KIN_MEAS, nans);
    int nanCount = mat_boolSum(nans, NUM_KIN_MEAS);

    if (nanCount == NUM_KIN_MEAS) {  // they're all nans
        return KF_NO_VALID_MEAS;
    }

    // Fill H
    mat_setSize(&H, NUM_TOT_STATES, NUM_KIN_MEAS - nanCount);
    arm_fill_f32(0, H.pData, mat_size(&H));
    int row = 0;
    if (!nans[0]) {
        mat_edit(&H, row, 0, dpdh);
        row++;
    }
    if (!nans[1]) {
        mat_edit(&H, row, 3, 1 / G);
        row++;
    }
    if (!nans[2]) {
        mat_edit(&H, row, 3, 1 / G);
        row++;
    }
    return KF_SUCCESS;
}

void kf_fx(mat* x, mfloat dt, const mfloat* w) {  // state update function
    // kf_F() should be called before outside this function so F is updated, so
    // I won't call it again here
    mfloat tempspace[4 * 4];  // space for w matrix and temp x
    mat x_temp = {NUM_TOT_STATES, 1, tempspace};
    arm_mat_mult_f32(&F, x, &x_temp);  // integrate kinematic states
    mat_copy(&x_temp, x);              // copy back into x

    // Make w mat
    mat w_mat = {4, 4, tempspace};
    mfloat diag[] = {1, 1, 1, 1};
    mat_diag(&w_mat, diag, false);  // put ones on diagonal
    // put values in. This is such a terrible way to write a matrix
    mat_edit(&w_mat, 0, 1, .5 * dt * (-w[0]));
    mat_edit(&w_mat, 0, 2, .5 * dt * (-w[1]));
    mat_edit(&w_mat, 0, 3, .5 * dt * (-w[2]));
    mat_edit(&w_mat, 1, 0, .5 * dt * (w[0]));
    mat_edit(&w_mat, 1, 2, .5 * dt * (w[2]));
    mat_edit(&w_mat, 1, 3, .5 * dt * (-w[1]));
    mat_edit(&w_mat, 2, 0, .5 * dt * (w[1]));
    mat_edit(&w_mat, 2, 1, .5 * dt * (-w[2]));
    mat_edit(&w_mat, 2, 3, .5 * dt * (w[0]));
    mat_edit(&w_mat, 3, 0, .5 * dt * (w[2]));
    mat_edit(&w_mat, 3, 1, .5 * dt * (w[1]));
    mat_edit(&w_mat, 3, 2, .5 * dt * (-w[0]));

    // do that quat integration
    mfloat qspace[] = {x->pData[3], x->pData[4], x->pData[5],
                       x->pData[6]};  // pull quaternion out of current state
    mat quat = {4, 1, qspace};
    mat q_out = {4, 1, &(x->pData[3])};  // points to the memory in x
    arm_mat_mult_f32(&w_mat, &quat, &q_out);
}

void kf_hx(const mat* x, const mfloat* z, mat* out) {
    mat_edit(out, 0, 0, kf_altToPressure(mat_val(x, 0, 0)));
    mat_edit(out, 1, 0, mat_val(x, 1, 0) / G + 1);  // acc 1
    mat_edit(out, 2, 0, mat_val(x, 2, 0) / G + 1);  // acc 2
    // TODO: Check up direction
}

void kf_resid(const mat* x, const mfloat* z, mat* out) {
    mfloat temp_space[NUM_KIN_MEAS];
    mat hx_mat = {NUM_KIN_MEAS, 1, temp_space};
    // y = z - hx # use h(x) for EKF
    kf_hx(x, z, &hx_mat);    // hx = h(x) (state to meas frame)
    mat_scale(&hx_mat, -1);  // -hx
    arm_add_f32(z, hx_mat.pData, out->pData, NUM_KIN_MEAS);  // y = z + -hx
}

mfloat kf_altToPressure(mfloat alt) {
    // #assert alt> 0 #if altitude is negative it breaks
    // #alt = np.max([ alt, [0] ]) #it breaks with negative alt.
    mfloat p = (pow((1 - alt / 44330), 5.25588)) * SEA_LEVEL_PRESSURE;
    return p;
    // TODO: Checks for negative alt make NaN pressure
}

KfState kf_getState() {
    KfState state = {time, &x, &P, NUM_TOT_STATES};
    return state;
}
