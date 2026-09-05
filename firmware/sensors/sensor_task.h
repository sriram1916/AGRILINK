#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"

namespace agri::sensors {

// Initialize the sensor subsystem. Must be called once before vSensorTask.
// The interface is intentionally hardware-agnostic so a real driver
// implementation can be substituted later without changing consumers.
void init_sensors(void);

// Acquire a fresh sensor snapshot. The default implementation is a
// deterministic software simulation. A real driver implementation would
// replace the body of this function while keeping its signature.
SensorSnapshot read_sensors(void);

// Get the most recent snapshot produced by the sensor task.
// Returns false if no snapshot has been produced yet (e.g. before init).
bool get_latest_snapshot(SensorSnapshot &out);

// Phase 7 — fault-injection hooks (deterministic, scoped).
// Each hook is set by the fault injector; read_sensors() honors the
// active flag and clears it after applying.
void mark_invalid_gps(void);
void mark_invalid_imu(void);
void mark_invalid_wheel(void);

// FreeRTOS sensor acquisition task. Runs at the priority declared in
// firmware/rtos/agri_rtos.h. Periodically updates the latest snapshot
// and logs the readings.
void vSensorTask(void *arg);

}  // namespace agri::sensors
