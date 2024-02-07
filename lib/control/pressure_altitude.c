#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define SIZE 5

void pressureToAltitude(double *, double *);

int main(void) {
  double pressure[SIZE] = {1003, 800, 700, 675,
                           650}; // Input pressure in millibars
  double altitude[SIZE];         // Output associated altitude values in meters

  pressureToAltitude(pressure, altitude); // altitude in meters

  printf("Pressure (mbar), Altitude (m):\n"); // Print Statements

  for (int j = 0; j < SIZE; j++) {
    printf("%f, %f\n", pressure[j], altitude[j]);
  }

  return 0;
}

// Input: pressure [double] - in millibars
void pressureToAltitude(double pressure[], double altitude[]) {
  // Constants
  double seaLevelPressure = 1013.25; // Standard sea level pressure in millibars
  double lapseRate = 0.0065;         // Standard temperature lapse rate in K/m

  // Calculate altitudes using barometric formula
  for (int i = 0; i < SIZE; i++) {
    altitude[i] =
        44330 * (1 - pow((pressure[i] / seaLevelPressure), 1 / 5.25588));
  }

  return 0;
}