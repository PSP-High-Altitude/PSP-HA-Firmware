#ifndef DATA_H
#define DATA_H

#include <stdint.h>

// Barometer MS5637

typedef struct {
    int16_t
        temperature; // After we do the calculations, we should get a signed int
                     // 16 for temp. The range would be between -4000 to 8500 C
} Temperature;

typedef struct {
    uint32_t pressure; // After calculations. the pressure should be a signed
                       // int 32 with a range of 1000 to 120000 mbar
} Pressure;

// IMU LSM6

typedef struct {
    uint16_t accelX; // 16 bit linear acceleration sensor x-axis in g
    uint16_t accelY; // 16 bit linear acceleration sensor y-axis in g
    uint16_t accelZ; // 16 bit linear acceleration sensor z-axis in g
} Accel;

typedef struct {
    uint16_t gyroX; // 16 bit anglular rate sensor yaw x-axis in dps
    uint16_t gyroY; // 16 bit anglular rate sensor yaw y-axis in dps
    uint16_t gyroZ; // 16 bit anglular rate sensor yaw z-axis in dps
} Gyro;
#endif // DATA_H