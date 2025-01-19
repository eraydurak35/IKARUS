#ifndef SMALL_CONTROL_H
#define SMALL_CONTROL_H

#include <stdio.h>
#include "state_estimator.h"
#include "typedefs.h"
#include <hitl.h>
#include <setup.h>

#if SETUP_ENABLE_HITL == true
#define MAX_TARGET_THROTTLE 1023
#else
#define MAX_TARGET_THROTTLE 600
#endif

#define IDLE_THROTTLE 100

void small_drone_control_init(radio_control_t *rad, telemetry_small_integer_t *telemetry, flight_t *flight, target_t *target, states_t *state, config_t *config);
void small_drone_flight_control();

#endif