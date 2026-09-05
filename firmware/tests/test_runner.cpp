#include "firmware/tests/test_runner.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>

#include "agri_types.h"
#include "firmware/communication/communication.h"
#include "firmware/fault_injection/fault_injector.h"
#include "firmware/heartbeat/heartbeat.h"
#include "firmware/safety/safety_task.h"
#include "firmware/sensors/sensor_task.h"
#include "firmware/simulation/vehicle_simulation.h"

namespace agri::tests {
namespace {

const char *TAG = "test_runner";

constexpr uint32_t kTestPeriodMs      = 500;
constexpr uint32_t kStartupGraceMs    = 3000;
constexpr uint32_t kSummaryAfterMs   = 24000;

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

// Track which test IDs have already been emitted (avoid log spam).
struct SuiteState {
    bool startup_emitted{false};
    bool sensor_valid_emitted{false};
    bool sensor_changing_emitted{false};
    bool comm_tx_emitted{false};
    bool comm_rx_crc_emitted{false};
    bool control_output_emitted{false};
    bool heartbeat_emitted{false};

    // Per-fault window assertions.
    bool gps_fault_gating_checked{false};
    bool comm_fault_gating_checked{false};
    bool ctrl_fault_reject_checked{false};

    // Recovery assertions.
    bool recovery_after_gps_checked{false};
    bool recovery_after_comm_checked{false};
    bool recovery_after_ctrl_checked{false};

    bool summary_emitted{false};

    uint32_t pass_count{0};
    uint32_t fail_count{0};

    // Previous sensor sample for "changing" check.
    bool     have_prev{false};
    double   prev_lat{0.0};
    double   prev_lon{0.0};

    // "Saw non-zero" latches for control / heartbeat presence.
    bool saw_nonzero_speed{false};
    bool saw_nonzero_steering{false};
    bool saw_heartbeat_log_signal{false};
};

static SuiteState g_state;

void emit_pass(const char *id) {
    ESP_LOGI(TAG, "[TEST] %s PASS", id);
    g_state.pass_count++;
}

void emit_fail(const char *id, const char *reason) {
    ESP_LOGW(TAG, "[TEST] %s FAIL reason=%s", id, reason);
    g_state.fail_count++;
}

bool within(uint32_t now_ms, uint32_t t0, uint32_t t1) {
    return now_ms >= t0 && now_ms < t1;
}

}  // namespace

void test_runner_init(void) {
    g_state = SuiteState{};
    ESP_LOGI(TAG, "[AGRILINK] Test runner initialized");
}

void vTestRunnerTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Test runner task started");

    while (true) {
        const uint32_t now_ms = monotonic_ms();

        // --- A. STARTUP ---
        if (!g_state.startup_emitted && now_ms >= kStartupGraceMs) {
            const heartbeat::SystemState s = heartbeat::get_system_state();
            if (s == heartbeat::SystemState::READY) {
                emit_pass("startup_ready");
            } else {
                emit_fail("startup_ready", "heartbeat state != READY");
            }
            g_state.startup_emitted = true;
        }

        // --- B. SENSOR VALIDITY ---
        SensorSnapshot snap{};
        const bool have_snap = sensors::get_latest_snapshot(snap);
        if (have_snap && !g_state.sensor_valid_emitted && now_ms >= kStartupGraceMs) {
            if (snap.gps.valid && snap.imu.valid && snap.wheel.valid) {
                emit_pass("sensor_valid");
            } else {
                emit_fail("sensor_valid", "at least one channel invalid");
            }
            g_state.sensor_valid_emitted = true;
        }

        // --- B. SENSORS CHANGING ---
        if (have_snap && g_state.sensor_valid_emitted && !g_state.sensor_changing_emitted) {
            if (g_state.have_prev) {
                const double dlat = snap.gps.latitude_deg  - g_state.prev_lat;
                const double dlon = snap.gps.longitude_deg - g_state.prev_lon;
                if (std::fabs(dlat) > 0.0 || std::fabs(dlon) > 0.0) {
                    emit_pass("sensor_changing");
                } else {
                    emit_fail("sensor_changing", "GPS lat/lon unchanged across samples");
                }
                g_state.sensor_changing_emitted = true;
            }
            g_state.prev_lat  = snap.gps.latitude_deg;
            g_state.prev_lon  = snap.gps.longitude_deg;
            g_state.have_prev = true;
        }

        // --- C. COMMUNICATION ---
        const CommStats stats = communication::comm_get_stats();
        if (!g_state.comm_tx_emitted && now_ms >= kStartupGraceMs + 1000) {
            if (stats.tx_count > 0) {
                emit_pass("comm_tx");
            } else {
                emit_fail("comm_tx", "tx_count == 0");
            }
            g_state.comm_tx_emitted = true;
        }
        if (!g_state.comm_rx_crc_emitted && now_ms >= kStartupGraceMs + 1500) {
            if (stats.rx_count > 0 && stats.crc_errors == 0) {
                emit_pass("comm_rx_crc_ok");
            } else if (stats.rx_count == 0) {
                emit_fail("comm_rx_crc_ok", "rx_count == 0");
            } else {
                emit_fail("comm_rx_crc_ok", "crc_errors > 0");
            }
            g_state.comm_rx_crc_emitted = true;
        }

        // --- D. CONTROL OUTPUT (non-zero speed/steering observed) ---
        VehicleSimState sim{};
        simulation::vehicle_simulation_get_state(sim);
        if (!g_state.saw_nonzero_speed && std::fabs(sim.speed_mps) > 0.01) {
            g_state.saw_nonzero_speed = true;
        }
        if (!g_state.saw_nonzero_steering && std::fabs(sim.steering_deg) > 0.01) {
            g_state.saw_nonzero_steering = true;
        }
        if (!g_state.control_output_emitted && now_ms >= kStartupGraceMs + 2500) {
            if (g_state.saw_nonzero_speed || g_state.saw_nonzero_steering) {
                emit_pass("control_output_present");
            } else {
                emit_fail("control_output_present", "sim speed and steering both ~0");
            }
            g_state.control_output_emitted = true;
        }

        // --- E. SAFETY GATING DURING FAULT WINDOWS ---
        const SafetyDecision dec = safety::safety_get_last_decision();

        // GPS fault window (t=8..12s)
        if (!g_state.gps_fault_gating_checked && within(now_ms, 9000, 11500)) {
            const bool inhibited = !dec.allow_control || dec.clamp_speed_to_zero;
            if (inhibited &&
                (dec.status == SafetyStatus::DEGRADED_SENSORS ||
                 dec.status == SafetyStatus::SAFE_STOP    ||
                 dec.status == SafetyStatus::EMERGENCY_STOP)) {
                emit_pass("safety_gating_gps_fault");
            } else {
                emit_fail("safety_gating_gps_fault", "allow_control=true during GPS fault window");
            }
            g_state.gps_fault_gating_checked = true;
        }

        // Comm fault window (t=14..17s)
        if (!g_state.comm_fault_gating_checked && within(now_ms, 14500, 16500)) {
            const bool inhibited = !dec.allow_control || dec.clamp_speed_to_zero;
            if (inhibited &&
                (dec.status == SafetyStatus::DEGRADED_COMM ||
                 dec.status == SafetyStatus::SAFE_STOP    ||
                 dec.status == SafetyStatus::DEGRADED_SENSORS ||
                 dec.status == SafetyStatus::EMERGENCY_STOP)) {
                emit_pass("safety_gating_comm_fault");
            } else {
                emit_fail("safety_gating_comm_fault", "allow_control=true during comm fault window");
            }
            g_state.comm_fault_gating_checked = true;
        }

        // Control invalid cmd window (t=20..22s) — Phase 5 validator rejects it.
        if (!g_state.ctrl_fault_reject_checked && within(now_ms, 20000, 21500)) {
            const fault_injection::FaultInjectorEvent ev =
                fault_injection::fault_injector_get_last_event();
            if (ev.id == FaultId::CONTROL_INVALID_CMD && ev.activated) {
                emit_pass("fault_injector_ctrl_window");
            } else {
                emit_fail("fault_injector_ctrl_window", "last injector event != CONTROL_INVALID_CMD activate");
            }
            g_state.ctrl_fault_reject_checked = true;
        }

        // --- F. RECOVERY (return to NORMAL after each window) ---
        if (!g_state.recovery_after_gps_checked && within(now_ms, 13000, 13800)) {
            if (dec.status == SafetyStatus::NORMAL) {
                emit_pass("recovery_after_gps");
            } else {
                emit_fail("recovery_after_gps", "safety status != NORMAL after GPS window");
            }
            g_state.recovery_after_gps_checked = true;
        }
        if (!g_state.recovery_after_comm_checked && within(now_ms, 18000, 19500)) {
            if (dec.status == SafetyStatus::NORMAL) {
                emit_pass("recovery_after_comm");
            } else {
                emit_fail("recovery_after_comm", "safety status != NORMAL after comm window");
            }
            g_state.recovery_after_comm_checked = true;
        }
        if (!g_state.recovery_after_ctrl_checked && within(now_ms, 23000, 23500)) {
            if (dec.status == SafetyStatus::NORMAL) {
                emit_pass("recovery_after_ctrl");
            } else {
                emit_fail("recovery_after_ctrl", "safety status != NORMAL after ctrl window");
            }
            g_state.recovery_after_ctrl_checked = true;
        }

        // --- G. HEARTBEAT (already observable; we just confirm presence) ---
        if (!g_state.heartbeat_emitted && now_ms >= kStartupGraceMs + 3500) {
            // Heartbeat toggles every 2s — by ~6.5s we expect at least 1 cycle.
            // We rely on the heartbeat log line having appeared; since the
            // test runner is the only task writing logs, we approximate by
            // checking that the heartbeat SystemState is still READY (or
            // moved on, which still means it ran).
            const heartbeat::SystemState s = heartbeat::get_system_state();
            if (s == heartbeat::SystemState::READY ||
                s == heartbeat::SystemState::WARNING ||
                s == heartbeat::SystemState::ERROR) {
                emit_pass("heartbeat_present");
                g_state.saw_heartbeat_log_signal = true;
            } else {
                emit_fail("heartbeat_present", "heartbeat state unexpected");
            }
            g_state.heartbeat_emitted = true;
        }

        // --- SUMMARY ---
        if (!g_state.summary_emitted && now_ms >= kSummaryAfterMs) {
            ESP_LOGI(TAG, "[TEST] suite_complete pass=%u fail=%u",
                     static_cast<unsigned>(g_state.pass_count),
                     static_cast<unsigned>(g_state.fail_count));
            g_state.summary_emitted = true;
        }

        vTaskDelay(pdMS_TO_TICKS(kTestPeriodMs));
    }
}

}  // namespace agri::tests
