#include <kalman.h>
#include <stdio.h>

int get_data(FILE* file, SensorFrame* sf);
int save_state(FILE* file, uint64_t timestamp, KfState* state,
               FlightPhase phase);
int print_mat(const mat* m);

int main() {
    char* strrr = "test\nStarting\n";
    printf(strrr);
    // mfloat test = .0394568657645342;
    // printf("%f mfloat\n", test);

    // printf("Starting\n");
    SensorFrame data = {3135426224, 27.62, 983.41, 0.033,  1.07,
                        0.043,      0.015, 1.004,  -0.003, 0.018,
                        -0.333,     -0.35, 3.895,  -4.941, -5.142};
    // 'x_0' :     np.vstack([vec([0., 0, 0]), q0]),
    // 'P_0' :     np.diag([opt["q_h"]/10, opt["q_v"]*s, opt["q_a"], 1, 1, 1,
    // 1]), # guess 'gyro_var': vec([0.5, 0.5, 0.5]), 'Q_var' :
    // np.array([opt["q_h"],opt["q_v"],opt["q_a"], .1, .1, .1, .1]),
    // 'measurement_vars': np.array([opt["p_var"], opt["a_var"], opt["a_var"],
    // .1, .1, .1]), 'Q_var2' :   np.array([apo["q_h"],apo["q_v"],apo["q_a"],
    // .1, .1, .1, .1]), 'measurement_vars2': np.array([apo["p_var"],
    // apo["a_var"], apo["a_var"], .1, .1, .1]),
    mfloat x0[] = {251.40557961, 0, 0, 1, 0, 0, 0}; // set for pal test
    mfloat P0_diag[] = {5.0e-02, 3.2e-03, 4.0e+00, 1.0e+00, 1.0e+00, 1.0e+00, 1.0e+00};

    // initialize kf
    kf_init_mats();
    kf_init_state(x0, P0_diag);

    // TEST SOME MATH
    // mfloat mdata[9] = {1,2,3,4,5,6,7,8,9};
    // mat M = {3, 3, mdata};
    // mfloat edata[9] = {1,2,3,1,2,3,0,0,1};
    // mat eye = {2,3,edata};
    // mfloat odata[4];
    // mat out = {2,2,odata};
    // arm_status st = mat_transposeMultiply(&eye, &M, &out);
    // // arm_mat_trans_f32(&M, &eye);
    // print_mat(&out);
    printf("math test done\n");


    // const char* dataFoleder =
    //     "C:\\Users\\hkadl\\OneDrive -
    //     purdue.edu\\Documents\\PSP\\Python\\Data " "Files";
    // const char* filename = "\\pal_test\\pal_fsl_test_dat.csv";
    const char* path =
        "C:\\Users\\hkadl\\OneDrive - purdue.edu\\Documents\\PSP\\Python\\Data "
        "Files\\pal_test\\pal_fsl_test_dat.csv";
    // const char* outPath = "C:\\Users\\hkadl\\OneDrive -
    // purdue.edu\\Documents\\PSP\\Python\\Data
    // Files\\pal_test\\pal_fsl_test_fsl.csv"; const char* outPath =
    // "C:\\Users\\hkadl\\OneDrive -
    // purdue.edu\\Documents\\PSP\\Python\\pal_kf_dat\\pal_test_kf.csv";
    const char* outPath = "kf_out.csv";
    mfloat tstop = 3150;

    FILE* outFile = fopen(outPath, "w");
    if (!outFile) {
        printf("outfile error\n");
        fclose(outFile);
    }
    // const char* outheader = "";
    fprintf(outFile,
            "timestamp,flight_phase,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,acc_n,"
            "acc_e,acc_d,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,orient_x,orient_y,"
            "orient_z\n");  // header
    // fclose(outFile);

    FILE* file = fopen(path, "r");
    if (!file) {
        printf("data file error\n");
        fclose(file);
    }
    char tempBuffer[256];
    fscanf(file, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
           tempBuffer);  // header

    int nothing = 0;

    // do kf
    while (kf_getState().time < tstop) {
        get_data(file, &data);
        kf_do_kf(&nothing, FP_BOOST, &data);
        KfState state = kf_getState();
        save_state(outFile, data.timestamp, &state, FP_BOOST);
    }

    // end
    kf_free_mats();
    fclose(file);
    fclose(outFile);

    printf("Done\n");

    return 0;
}

int get_data(FILE* file, SensorFrame* sf) {
    // float32_t time;
    int num;
    // num = fscanf(file, "%d,", &sf->timestamp);
    num = fscanf(file, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                 &sf->timestamp, &sf->temperature, &sf->pressure, &sf->acc_h_x,
                 &sf->acc_h_y, &sf->acc_h_z, &sf->acc_i_x, &sf->acc_i_y,
                 &sf->acc_i_z, &sf->rot_i_x, &sf->rot_i_y, &sf->rot_i_z,
                 &sf->mag_i_x, &sf->mag_i_y, &sf->mag_i_z);  // 15 numbers
    // sf->timestamp = (int) time;
    return num;
}
int save_state(FILE* file, uint64_t timestamp, KfState* state,
               FlightPhase phase) {
    mfloat h = state->x->pData[0];
    mfloat v = state->x->pData[1];
    mfloat a = state->x->pData[2];
    // printf("ts: %llu, h: %f, v: %f, a: %f, phase: %d\n", timestamp, h, v, a,
    // (int) phase);
    int out;
    out = fprintf(file, "%llu,%d,0,0,%f,0,0,%f,0,0,%f,0,0,0,%f,%f,%f,0,0,0\n",
                  timestamp, (int)phase, -1 * h, -1 * v, -1 * a, 0.0, 0.0, 0.0);
    // out = printf("%llu,%d,0,0,%f,0,0,%f,0,0,%f,0,0,0,%f,%f,%f,0,0,0\n",
    // timestamp, (int) phase, -1*h, -1*v, -1*a, 0.0,0.0,0.0); fflush(stdout);
    // // makes stuff not wait until the end to print in debug mode
    return out;
}

int print_mat(const mat* m) {
    int n = mat_size(m);
    for (int i = 0; i<m->numRows; i++) {
        printf("r%d: ", i);
        for (int j = 0; j < m->numCols; j++) {
            printf("%f, ", mat_val(m,i,j));
        }
        printf("\n");
    }
    fflush(stdout);
    return n;
}