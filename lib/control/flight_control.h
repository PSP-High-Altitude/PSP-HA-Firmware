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

#define fp_check_boost_fast_coast(is_fast, is_coast, value_if_both, \
                                  value_if_fast, value_if_coast,    \
                                  value_if_none)

#endif  // FLIGHT_CONTROL_H