#include "pressure_altitude.h"

#include "flight_estimation.h"

// #define SIZE 5

// int main(void) {
//   double pressure[SIZE] = {1003, 800, 700, 675,
//                            650}; // Input pressure in millibars
//   double altitude[SIZE];         // Output associated altitude values in
//   meters

//   pressureToAltitude(pressure, altitude); // altitude in meters

//   printf("Pressure (mbar), Altitude (m):\n"); // Print Statements

//   for (int j = 0; j < SIZE; j++) {
//     printf("%f, %f\n", pressure[j], altitude[j]);
//   }

//   return 0;
// }

// Input: pressure [double] - in millibars
float pressureToAltitude(float pressure) {
    // Constants
    float seaLevelPressure =
        1013.25;  // Standard sea level pressure in millibars
    //   double lapseRate = 0.0065; // Standard temperature lapse rate in K/m

    // Calculate altitudes using barometric formula
    float altitude =
        44330 * (1 - pow((pressure / seaLevelPressure), 1 / 5.25588));

    return altitude;
}