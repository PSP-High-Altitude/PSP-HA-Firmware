typedef enum {
    FP_INIT,
    FP_WAIT,    // not ready
    FP_READY,   // on the pad
    FP_BOOST,   // motor burning
    FP_COAST,   // waiting for apogee
    FP_DROGUE,  // under drogue
    FP_MAIN,    // under main
    FP_LANDED,
} FlightPhase;

/* Struct definitions */
typedef struct _SensorFrame {
    /* System timestamp (us since power on) */
    uint64_t timestamp;
    /* Barometer temperature (deg C) */
    float temperature;
    /* Barometer pressure (mbar) */
    float pressure;
    /* Accelerometer acceleration vector (g) */
    float acc_h_x;
    float acc_h_y;
    float acc_h_z;
    /* IMU acceleration vector (g) */
    float acc_i_x;
    float acc_i_y;
    float acc_i_z;
    /* IMU rotation vector (dps) */
    float rot_i_x;
    float rot_i_y;
    float rot_i_z;
    /* Magnetometer magnetic field vector (gauss) */
    float mag_i_x;
    float mag_i_y;
    float mag_i_z;
} SensorFrame;

// typedef struct {
//     float time;  // seconds

//     // Linear model outputs
//     float posVert;  // m
//     float velVert;  // m/s
//     float accVert;  // m/s^2

//     // Inertial model outputs
//     Vector posGeo;           // m
//     Vector velGeo;           // m/s
//     Vector accGeo;           // m/s^2
//     Vector angVelBody;       // rad/s
//     Quaternion orientation;  // rad (ish)

//     // Intermediate variables (NOT LOGGED)
//     float posImu;  // m
//     float velImu;  // m/s
//     float accImu;  // m/s^2

//     float posBaro;  // m
//     float velBaro;  // m/s
//     float accBaro;  // m/s^2
// } StateEst;

typedef enum {
    IMU_X_UP,
    IMU_Y_UP,
    IMU_Z_UP,
    IMU_X_DOWN,
    IMU_Y_DOWN,
    IMU_Z_DOWN
} SensorDirection;

typedef enum {
    STATUS_OK,
    STATUS_BUSY,
    STATUS_ERROR,
    STATUS_DATA_ERROR,
    STATUS_STATE_ERROR,
    STATUS_MEMORY_ERROR,
    STATUS_HARDWARE_ERROR,
    STATUS_TESTING_ERROR,
    STATUS_PARAMETER_ERROR,  // Error with invalid parameter being passed
    STATUS_TIMEOUT_ERROR,
} Status;

typedef struct {
    float time;  // seconds

    // Linear model outputs
    float posVert;  // m
    float velVert;  // m/s
    float accVert;  // m/s^2

    // // Inertial model outputs
    // Vector posGeo;           // m
    // Vector velGeo;           // m/s
    // Vector accGeo;           // m/s^2
    // Vector angVelBody;       // rad/s
    // Quaternion orientation;  // rad (ish)

    // EKF outputs
    float posEkf;
    float velEkf;
    float accEkf;

    float orientEkf1;
    float orientEkf2;
    float orientEkf3;
    float orientEkf4;

    float posVarEkf;
    float velVarEkf;
    float accVarEkf;

    float orientVarEkf1;
    float orientVarEkf2;
    float orientVarEkf3;
    float orientVarEkf4;

    // Intermediate variables (NOT LOGGED)
    float posImu;  // m
    float velImu;  // m/s
    float accImu;  // m/s^2

    float posBaro;  // m
    float velBaro;  // m/s
    float accBaro;  // m/s^2
} StateEst;
#define DEFAULT_ORIENTATION (IMU_Z_UP)