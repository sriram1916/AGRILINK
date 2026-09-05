#include "firmware/safety/safety_task.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

#include "firmware/communication/communication.h"
#include "firmware/heartbeat/heartbeat.h"
#include "firmware/sensors/sensor_task.h"
#include "firmware/simulation/vehicle_simulation.h"

namespace agri::safety {
namespace {

const char *TAG = "safety_task";

constexpr uint32_t kSafetyTaskPeriodMs = 1000;

// Safety thresholds. These are intentionally simple; the goal is
// to demonstrate the supervision chain, not to be authoritative.
constexpr double kMaxSafeSpeedMps        = 10.0;  // hard ceiling
constexpr uint32_t kCommStaleMs           = 5000; // no RX in this window -> degraded
constexpr uint32_t kMaxCrcErrorsForHealth = 0;    // any CRC error is degraded

// Deterministic test window: between simulation seconds T_START and
// T_END the supervisor treats the simulated GPS validity as lost.
// This is NOT a fault-injection framework (Phase 7) — it is the
// smallest deterministic signal needed to demonstrate that the
// safety chain can detect, inhibit, and transition.
constexpr uint32_t kTestWindowStartMs = 12000;
constexpr uint32_t kTestWindowEndMs   = 15000;

static SafetyDecision    g_last_decision{};
static bool              g_initialized = false;

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

const char *status_to_string(SafetyStatus s) {
    switch (s) {
        case SafetyStatus::NORMAL:           return "NORMAL";
        case SafetyStatus::DEGRADED_SENSORS: return "DEGRADED_SENSORS";
        case SafetyStatus::DEGRADED_COMM:    return "DEGRADED_COMM";
        case SafetyStatus::SAFE_STOP:        return "SAFE_STOP";
        case SafetyStatus::EMERGENCY_STOP:   return "EMERGENCY_STOP";
        default:                              return "UNKNOWN";
    }
}

bool test_window_active(uint32_t now_ms, bool &log_enter, bool &log_exit) {
    static bool prev_active = false;
    const bool active = (now_ms >= kTestWindowStartMs) && (now_ms < kTestWindowEndMs);
    log_enter = active && !prev_active;
    log_exit  = !active && prev_active;
    prev_active = active;
    return active;
}

}  // namespace

void safety_init(void) {
    g_last_decision   = SafetyDecision{};
    g_last_decision.status        = SafetyStatus::NORMAL;
    g_last_decision.target_state  = VehicleState::READY;
    g_last_decision.allow_control = true;
    g_initialized = true;
    ESP_LOGI(TAG, "[AGRILINK] Safety subsystem initialized");
}

SafetyDecision safety_evaluate(void) {
    SafetyDecision d{};
    d.timestamp_ms = monotonic_ms();

    const uint32_t now_ms = d.timestamp_ms;

    // Collect current observable system state.
    SensorSnapshot snap{};
    const bool have_snap = sensors::get_latest_snapshot(snap);

    const CommStats stats = communication::comm_get_stats();

    VehicleSimState sim{};
    simulation::vehicle_simulation_get_state(sim);

    // ----- Condition 1: absolute vehicle speed ceiling.
    if (sim.speed_mps > kMaxSafeSpeedMps) {
        d.status             = SafetyStatus::EMERGENCY_STOP;
        d.target_state       = VehicleState::EMERGENCY_STOP;
        d.allow_control      = false;
        d.clamp_speed_to_zero = true;
        return d;
    }

    // ----- Condition 2: comm health (CRC errors, RX freshness).
    bool comm_unhealthy = false;
    if (stats.crc_errors > kMaxCrcErrorsForHealth) {
        comm_unhealthy = true;
    } else if (stats.last_rx_timestamp_ms != 0 &&
               (now_ms - stats.last_rx_timestamp_ms) > kCommStaleMs) {
        comm_unhealthy = true;
    }

    // ----- Condition 3: sensor validity.
    bool sensors_unhealthy = false;
    if (have_snap) {
        if (!snap.gps.valid || !snap.imu.valid || !snap.wheel.valid) {
            sensors_unhealthy = true;
        }
    }

    // ----- Condition 4: deterministic demonstration window.
    bool log_enter = false, log_exit = false;
    const bool in_test_window = test_window_active(now_ms, log_enter, log_exit);
    if (log_enter) {
        ESP_LOGW(TAG, "[AGRILINK] SAFETY TEST WINDOW active t=%u", now_ms);
    }
    if (log_exit) {
        ESP_LOGW(TAG, "[AGRILINK] SAFETY TEST WINDOW cleared t=%u", now_ms);
    }
    if (in_test_window) {
        sensors_unhealthy = true;  // simulate sensor validity loss in the window
    }

    // ----- Decision.
    if (comm_unhealthy && sensors_unhealthy) {
        d.status             = SafetyStatus::SAFE_STOP;
        d.target_state       = VehicleState::SAFE_STOP;
        d.allow_control      = false;
        d.clamp_speed_to_zero = true;
    } else if (comm_unhealthy) {
        d.status             = SafetyStatus::DEGRADED_COMM;
        d.target_state       = VehicleState::DEGRADED;
        d.allow_control      = false;
        d.clamp_speed_to_zero = true;
    } else if (sensors_unhealthy) {
        d.status             = SafetyStatus::DEGRADED_SENSORS;
        d.target_state       = VehicleState::DEGRADED;
        d.allow_control      = false;
        d.clamp_speed_to_zero = true;
    } else {
        d.status             = SafetyStatus::NORMAL;
        d.target_state       = VehicleState::READY;
        d.allow_control      = true;
        d.clamp_speed_to_zero = false;
    }

    g_last_decision = d;
    return d;
}

bool safety_apply_to_control(ControlOutput &output) {
    const SafetyDecision d = safety_evaluate();

    if (!d.allow_control || d.clamp_speed_to_zero) {
        output.commanded_speed_mps    = 0.0;
        output.commanded_steering_deg = 0.0;
        output.enable                 = 0;
        return true;
    }
    return false;
}

SafetyDecision safety_get_last_decision(void) {
    return g_last_decision;
}

void vSafetyTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Safety task started");

    SafetyDecision prev{};
    bool first_cycle = true;

    while (true) {
        const SafetyDecision d = safety_evaluate();

        if (first_cycle || d.status != prev.status || d.target_state != prev.target_state) {
            ESP_LOGI(TAG,
                     "[AGRILINK] SAFETY: status=%s target=%d allow_control=%u reason=evaluated",
                     status_to_string(d.status),
                     static_cast<int>(d.target_state),
                     static_cast<unsigned>(d.allow_control));
            if (d.target_state != prev.target_state) {
                ESP_LOGI(TAG,
                         "[AGRILINK] SAFETY: state -> %d",
                         static_cast<int>(d.target_state));
                // Reflect into the heartbeat module's SystemState so
                // it shows up in heartbeat logs and is observable.
                heartbeat::set_system_state(
                    d.target_state == VehicleState::READY
                        ? heartbeat::SystemState::READY
                    : d.target_state == VehicleState::DEGRADED
                        ? heartbeat::SystemState::WARNING
                    : heartbeat::SystemState::ERROR);
            }
            first_cycle = false;
            prev        = d;
        }

        vTaskDelay(pdMS_TO_TICKS(kSafetyTaskPeriodMs));
    }
}

}  // namespace agri::safety
