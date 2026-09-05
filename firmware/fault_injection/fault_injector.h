#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"

namespace agri::fault_injection {

// Phase 7 — deterministic fault-injection framework.
//
// The injector is a TEST mechanism. It deliberately creates abnormal
// conditions in sensor / comm / control subsystems. The Phase 6
// Safety supervisor detects those conditions and makes the safety
// decision. The injector does NOT bypass safety — it only sets
// flags in the affected subsystem, which then surfaces through the
// normal safety_evaluate() path.

// One-time initialization. Must be called before vFaultInjectorTask.
void fault_injector_init(void);

// Schedule a deterministic fault injection. The fault becomes
// active at the next tick where now_ms >= schedule_at_ms, and is
// cleared automatically after duration_ms.
void fault_injector_schedule(FaultId id, uint32_t schedule_at_ms, uint32_t duration_ms);

// FreeRTOS fault-injector task. Runs at AGRILINK_TASK_PRIORITY_FAULT_INJECTION.
void vFaultInjectorTask(void *arg);

// Phase 8 — observation hook. Reports the most recent fault event
// produced by the injector (which FaultId was activated/cleared
// and when). Used by the test runner to verify the deterministic
// schedule without bypassing the injector.
struct FaultInjectorEvent {
    FaultId id{FaultId::NONE};
    bool    activated{false};   // true = ACTIVATE, false = CLEAR
    uint32_t timestamp_ms{0};
};
FaultInjectorEvent fault_injector_get_last_event(void);

}  // namespace agri::fault_injection
