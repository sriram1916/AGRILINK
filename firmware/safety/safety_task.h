#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"

namespace agri::safety {

// Phase 6 — safety supervision.
//
// The supervisor observes existing sensor/comm/control/vehicle state
// and produces a SafetyDecision. The decision is consumed by the
// control layer to inhibit unsafe control outputs. This is a
// simulated prototype — not a certified safety controller.

// One-time initialization. Must be called before vSafetyTask.
void safety_init(void);

// Evaluate the current system state and produce a SafetyDecision.
// Reads Phase 3 sensor snapshot, Phase 4 comm stats, Phase 5 vehicle
// state, and the heartbeat's SystemState.
SafetyDecision safety_evaluate(void);

// Apply the most recent safety decision to a ControlOutput (may
// zero it if the decision is non-permissive). Returns true if the
// output was modified by safety.
bool safety_apply_to_control(ControlOutput &output);

// Get the most recent SafetyDecision (after safety_init has run).
SafetyDecision safety_get_last_decision(void);

// FreeRTOS safety task. Runs at AGRILINK_TASK_PRIORITY_SAFETY.
void vSafetyTask(void *arg);

}  // namespace agri::safety
