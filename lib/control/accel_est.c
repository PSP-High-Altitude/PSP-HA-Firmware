#include <stdio.h>

typedef struct {
    double* times;
    double* AccBodyY;
    double* AccDown;
    double* VelDown;
    double* PosDown;
    int i;
    double g;
    double p;
} accel_est;

void init_accel_est(accel_est* obj, int size);
void update_accel_est(accel_est* obj, double t, double ay);

int main() {
    // Example usage
    accel_est obj;
    init_accel_est(&obj, 10);

    // Simulate updates
    update_accel_est(&obj, 1000, -9.81);  // Replace with your actual data
    // Add more updates as needed

    // Clean up (free allocated memory, etc.)
    // ...

    return 0;
}

void init_accel_est(accel_est* obj, int size) {
    obj->times = (double*)malloc(size * sizeof(double));
    obj->AccBodyY = (double*)malloc(size * sizeof(double));
    obj->AccDown = (double*)malloc(size * sizeof(double));
    obj->VelDown = (double*)malloc(size * sizeof(double));
    obj->PosDown = (double*)malloc(size * sizeof(double));
    obj->i = 1;
    obj->g = -9.81;
    obj->p = 0;
}

void update_accel_est(accel_est* obj, double t, double ay) {
    // y is up
    obj->times[obj->i - 1] = t / 1000.0;  // convert to s
    obj->AccBodyY[obj->i - 1] =
        (ay - 1) * -1 * obj->g;  // convert to m/s^2 and correct 1g
    obj->AccDown[obj->i - 1] = -1 * obj->AccBodyY[obj->i - 1];

    if (obj->i <= 1) {
        obj->VelDown[obj->i - 1] = 0;
        obj->PosDown[obj->i - 1] = 0;
    } else {
        double timestep[2] = {obj->times[obj->i - 2], t / 1000.0};
        obj->VelDown[obj->i - 1] =
            obj->VelDown[obj->i - 2] +
            (timestep[1] *
             (obj->AccDown[obj->i - 2] + obj->AccDown[obj->i - 1]) / 2);
        obj->PosDown[obj->i - 1] =
            obj->PosDown[obj->i - 2] +
            (timestep[1] *
             (obj->VelDown[obj->i - 2] + obj->VelDown[obj->i - 1]) / 2);
    }

    obj->i++;
}
