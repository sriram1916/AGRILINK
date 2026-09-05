#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"

namespace agri::simulation {

// Phase 5 — vehicle simulation.
//
// A deterministic simulated vehicle state that the control output
// influences. The simulation is intentionally a simple kinematic
// model (no realistic tractor dynamics) — enough to demonstrate
// the command -> control -> vehicle -> sensor feedback chain.

// Initialize the vehicle simulation subsystem.
void vehicle_simulation_init(void);

// Apply the validated control output to the simulated actuators.
// Stores the latest output; consumed by vehicle_simulation_tick().
void vehicle_simulation_apply_control(const ControlOutput &output);

// Advance the simulation by one fixed time step.
void vehicle_simulation_tick(void);

// Get a copy of the current simulated vehicle state.
void vehicle_simulation_get_state(VehicleSimState &out);

// FreeRTOS vehicle simulation task. Runs at AGRILINK_TASK_PRIORITY_SIMULATION.
void vVehicleSimulationTask(void *arg);

}  // namespace agri::simulation
