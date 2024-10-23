#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "data.h"
#include "status.h"

typedef enum {
    FP_INIT,
    FP_READY,  // on the pad
    FP_BOOST_1,
    FP_FAST_BOOST_1,
    FP_FAST_1,  // faster than mach 1
    FP_COAST_1,
    FP_STAGE,
    FP_IGNITE,
    FP_BOOST_2,
    FP_FAST_BOOST_2,
    FP_FAST_2,  // faster than mach 1
    FP_COAST_2,
    FP_DROGUE,  // trigger drogue
    FP_MAIN,    // trigger chute
    FP_LANDED,
} FlightPhase;

typedef enum { FP_STG_GO, FP_STG_NOGO, FP_STG_WAIT } FlightStageStatus;

Status fp_init();

FlightPhase fp_get();

Status fp_update(const SensorFrame* sensor_frame);

FlightPhase fp_update_init(const SensorFrame* sensor_frame);
FlightPhase fp_update_ready(const SensorFrame* sensor_frame);
FlightPhase fp_update_boost_1(const SensorFrame* sensor_frame);
FlightPhase fp_update_fast_boost_1(const SensorFrame* sensor_frame);
FlightPhase fp_update_fast_1(const SensorFrame* sensor_frame);
FlightPhase fp_update_coast_1(const SensorFrame* sensor_frame);
FlightPhase fp_update_stage(const SensorFrame* sensor_frame);
FlightPhase fp_update_ignite(const SensorFrame* sensor_frame);
FlightPhase fp_update_boost_2(const SensorFrame* sensor_frame);
FlightPhase fp_update_fast_boost_2(const SensorFrame* sensor_frame);
FlightPhase fp_update_fast_2(const SensorFrame* sensor_frame);
FlightPhase fp_update_coast_2(const SensorFrame* sensor_frame);
FlightPhase fp_update_drogue(const SensorFrame* sensor_frame);
FlightPhase fp_update_main(const SensorFrame* sensor_frame);
FlightPhase fp_update_landed(const SensorFrame* sensor_frame);
uint8_t fp_stage_check_sep_lockout(const SensorFrame* sensor_frame,
                                   const StateEst* state);
uint8_t fp_stage_check_ignite_lockout(const SensorFrame* sensor_frame,
                                      const StateEst* state);
uint8_t fp_check_grounded(const SensorFrame* sensorframe,
                          const StateEst* state);

#endif  // FLIGHT_CONTROL_H