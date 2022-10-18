#ifndef DATA_H
#define DATA_H

#include <stdint.h>

// Barometer MS5637

typedef struct {
    float
        temperature;  // After we do the calculations, we should get a float for
                      // temp. The range would be between -40.00 to 85.00 C
    float pressure;   // After calculations. the pressure should be a float with
                      // a range of 10.00 to 1200.00 mbar
} BaroData;

// IMU LSM6

typedef struct {
    float accelX;  // linear acceleration sensor x-axis in g
    float accelY;  // linear acceleration sensor y-axis in g
    float accelZ;  // linear acceleration sensor z-axis in g
} Accel;

typedef struct {
    float gyroX;  // anglular rate sensor yaw x-axis in dps
    float gyroY;  // anglular rate sensor yaw y-axis in dps
    float gyroZ;  // anglular rate sensor yaw z-axis in dps
} Gyro;
#endif // DATA_H