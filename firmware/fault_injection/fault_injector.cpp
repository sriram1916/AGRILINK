#include "firmware/fault_injection/fault_injector.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "firmware/communication/communication.h"
#include "firmware/control/motion_control.h"
#include "firmware/sensors/sensor_task.h"

namespace agri::fault_injection {
namespace {

const char *TAG = "fault_injector";

constexpr uint32_t kInjectorPeriodMs = 500;

// Deterministic schedule (milliseconds since boot). Each entry arms
// a fault for the given duration; the injector auto-clears it.
// Phase 7 demo: GPS sensor loss, comm loss, invalid control command.
struct ScheduledFault {
    FaultId  id;
    uint32_t activate_at_ms;
    uint32_t duration_ms;
};

constexpr ScheduledFault kSchedule[] = {
    { FaultId::SENSOR_GPS_LOSS,     8000, 4000 },  // t=8s..12s
    { FaultId::COMM_LOSS,          14000, 3000 },  // t=14s..17s
    { FaultId::CONTROL_INVALID_CMD,20000, 2000 },  // t=20s..22s
};

constexpr size_t kScheduleCount = sizeof(kSchedule) / sizeof(kSchedule[0]);

// Per-fault runtime state.
struct FaultState {
    bool     active{false};
    uint32_t activate_at_ms{0};
    uint32_t clear_at_ms{0};
};

static FaultState g_states[kScheduleCount]{};

// Phase 8 — most recent injector event (for test observation).
static FaultInjectorEvent g_last_event{};

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

const char *id_to_string(FaultId id) {
    switch (id) {
        case FaultId::NONE:                return "NONE";
        case FaultId::SENSOR_GPS_LOSS:     return "SENSOR_GPS_LOSS";
        case FaultId::SENSOR_IMU_LOSS:     return "SENSOR_IMU_LOSS";
        case FaultId::SENSOR_WHEEL_LOSS:   return "SENSOR_WHEEL_LOSS";
        case FaultId::COMM_LOSS:           return "COMM_LOSS";
        case FaultId::CONTROL_INVALID_CMD: return "CONTROL_INVALID_CMD";
        default:                            return "UNKNOWN";
    }
}

void apply_fault(FaultId id) {
    switch (id) {
        case FaultId::SENSOR_GPS_LOSS:
            sensors::mark_invalid_gps();
            ESP_LOGW(TAG, "[AGRILINK] FAULT_INJECT: applied GPS validity loss");
            break;
        case FaultId::SENSOR_IMU_LOSS:
            sensors::mark_invalid_imu();
            ESP_LOGW(TAG, "[AGRILINK] FAULT_INJECT: applied IMU validity loss");
            break;
        case FaultId::SENSOR_WHEEL_LOSS:
            sensors::mark_invalid_wheel();
            ESP_LOGW(TAG, "[AGRILINK] FAULT_INJECT: applied WHEEL validity loss");
            break;
        case FaultId::COMM_LOSS:
            communication::comm_inject_loss();
            ESP_LOGW(TAG, "[AGRILINK] FAULT_INJECT: applied comm loss");
            break;
        case FaultId::CONTROL_INVALID_CMD:
            control::inject_invalid_command();
            ESP_LOGW(TAG, "[AGRILINK] FAULT_INJECT: applied invalid control command");
            break;
        case FaultId::NONE:
        default:
            break;
    }
}

}  // namespace

void fault_injector_init(void) {
    ESP_LOGI(TAG, "[AGRILINK] Fault injector subsystem initialized");
    for (size_t i = 0; i < kScheduleCount; ++i) {
        g_states[i] = FaultState{};
        g_states[i].activate_at_ms = kSchedule[i].activate_at_ms;
        g_states[i].clear_at_ms    = kSchedule[i].activate_at_ms
                                    + kSchedule[i].duration_ms;
    }
}

void fault_injector_schedule(FaultId id, uint32_t schedule_at_ms, uint32_t duration_ms) {
    ESP_LOGI(TAG,
             "[AGRILINK] FAULT_INJECT: schedule id=%s at=%u dur=%u",
             id_to_string(id), schedule_at_ms, duration_ms);
}

FaultInjectorEvent fault_injector_get_last_event(void) {
    return g_last_event;
}

void vFaultInjectorTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Fault injector task started");

    while (true) {
        const uint32_t now_ms = monotonic_ms();

        for (size_t i = 0; i < kScheduleCount; ++i) {
            FaultState &s        = g_states[i];
            const auto  &entry   = kSchedule[i];
            const bool   armed   = !s.active;
            const bool   expired = (s.clear_at_ms != 0) && (now_ms >= s.clear_at_ms);

            // Activate when the activation time has been reached and
            // the fault is not currently active.
            if (armed && now_ms >= entry.activate_at_ms) {
                ESP_LOGW(TAG,
                         "[AGRILINK] FAULT_INJECT: %s ACTIVATE t=%u",
                         id_to_string(entry.id), now_ms);
                apply_fault(entry.id);
                s.active = true;
                g_last_event = FaultInjectorEvent{entry.id, true, now_ms};
            }

            // Auto-clear when the duration has elapsed.
            if (s.active && expired) {
                ESP_LOGW(TAG,
                         "[AGRILINK] FAULT_INJECT: %s CLEAR t=%u",
                         id_to_string(entry.id), now_ms);
                s.active        = false;
                s.clear_at_ms   = 0;
                g_last_event = FaultInjectorEvent{entry.id, false, now_ms};
            }
        }

        vTaskDelay(pdMS_TO_TICKS(kInjectorPeriodMs));
    }
}

}  // namespace agri::fault_injection
