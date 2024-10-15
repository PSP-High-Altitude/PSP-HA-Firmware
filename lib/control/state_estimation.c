#include "state_estimation.h"

StateEst* s_current_state = {0};

Status se_init() { return STATUS_OK; }

const StateEst* se_predict() { return s_current_state; }

Status se_update(FlightPhase phase, const SensorFrame* sensor_frame) {
    return STATUS_OK;
}