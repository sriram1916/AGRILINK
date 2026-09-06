#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"

namespace agri::control {

// Phase 5 - motion-control subsystem.
//
// The control layer is transport-agnostic; it produces commands,
// encodes them into CommMessages (type = COMMAND), and lets the
// Phase 4 communication layer transport them. The control task
// then decodes its own looped-back command, validates/clamps it,
// computes a ControlOutput, and submits it to the vehicle
// simulation. It also reads back the latest sensor snapshot for
// feedback logging.
//
// Control behavior is driven by this parameter set. The default
// values preserve the original Phase 5/6/7 control behavior.
struct ControlParameters {
    uint32_t control_period_ms;
    double max_speed_mps;
    double min_speed_mps;
    double max_steering_deg;
    double raw_reject_speed_mps;
};

// Return the active control parameters.
const ControlParameters& get_control_parameters(void);

// Initialize the control subsystem.
void control_init(void);

// Validate and clamp a raw command into a safe output. The raw
// command may contain unreasonable values; the output is always
// bounded by the active ControlParameters.
ControlOutput validate_and_clamp_command(const ControlCommandPayload &raw);

// Encode a control command into a CommMessage (id = AGRILINK_MSG_COMMAND,
// type = COMMAND). Returns true if packing succeeded.
bool control_pack_command(const ControlCommandPayload &cmd, CommMessage &out);

// Decode a CommMessage into a ControlCommandPayload. Returns true if
// the message is a COMMAND message and the payload fits.
bool control_unpack_command(const CommMessage &msg, ControlCommandPayload &out);

// FreeRTOS motion-control task. Runs at AGRILINK_TASK_PRIORITY_CONTROL.
void vMotionControlTask(void *arg);

// Phase 7 - fault-injection hook. When armed, the next command cycle
// emits a deliberately invalid (out-of-range) desired speed. The
// existing Phase 5 validator rejects it; the safety supervisor's
// allow_control remains true (the fault is detected at validation).
void inject_invalid_command(void);

}  // namespace agri::control
