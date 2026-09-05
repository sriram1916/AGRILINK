#include "firmware/simulation/vehicle_simulation.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cmath>

namespace agri::simulation {
namespace {

const char *TAG = "vehicle_simulation";

// Simulation parameters.
constexpr double kWheelbaseM     = 1.6;     // notional, just for stability
constexpr double kSpeedAlpha     = 0.30;    // 1st-order lag coefficient
constexpr double kSteeringAlpha  = 0.50;
constexpr double kTickPeriodS    = 1.0;
constexpr uint32_t kSimTaskPeriodMs = 1000;

// Shared state, mutex-protected so the control task and the
// simulation task can access it concurrently.
static VehicleSimState  g_state{};
static ControlOutput    g_last_output{};
static SemaphoreHandle_t g_state_mutex = nullptr;

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

double deg_to_rad(double deg) {
    return deg * (3.14159265358979323846 / 180.0);
}

}  // namespace

void vehicle_simulation_init(void) {
    if (g_state_mutex == nullptr) {
        g_state_mutex = xSemaphoreCreateMutex();
    }
    ESP_LOGI(TAG, "[AGRILINK] Vehicle simulation subsystem initialized");
}

void vehicle_simulation_apply_control(const ControlOutput &output) {
    if (g_state_mutex == nullptr) {
        return;
    }
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_last_output = output;
        xSemaphoreGive(g_state_mutex);
    }
}

void vehicle_simulation_tick(void) {
    if (g_state_mutex == nullptr) {
        return;
    }
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    ControlOutput out = g_last_output;

    if (out.enable) {
        // First-order lag toward commanded speed.
        g_state.speed_mps += kSpeedAlpha * (out.commanded_speed_mps - g_state.speed_mps);
        g_state.steering_deg += kSteeringAlpha *
                               (out.commanded_steering_deg - g_state.steering_deg);
    } else {
        // Coast to a stop.
        g_state.speed_mps    *= (1.0 - kSpeedAlpha);
        g_state.steering_deg *= (1.0 - kSteeringAlpha);
    }

    // Kinematic update (no-slip bicycle model, simple and bounded).
    const double speed   = g_state.speed_mps;
    const double steering_rad = deg_to_rad(g_state.steering_deg);
    const double heading_rad  = deg_to_rad(g_state.heading_deg);

    // Avoid division blow-up at extreme steering; tan is bounded.
    const double dheading = (speed / kWheelbaseM) * std::tan(steering_rad) * kTickPeriodS;

    g_state.position_x_m += speed * std::cos(heading_rad) * kTickPeriodS;
    g_state.position_y_m += speed * std::sin(heading_rad) * kTickPeriodS;
    g_state.heading_deg   = std::fmod(g_state.heading_deg +
                                      (dheading * 180.0 / 3.14159265358979323846),
                                      360.0);
    if (g_state.heading_deg < 0.0) {
        g_state.heading_deg += 360.0;
    }
    g_state.timestamp_ms = monotonic_ms();

    VehicleSimState snapshot = g_state;
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI(TAG,
             "[AGRILINK] VEHICLE TICK spd=%.2f steer=%.2f x=%.2f y=%.2f hdg=%.2f t=%u",
             snapshot.speed_mps,
             snapshot.steering_deg,
             snapshot.position_x_m,
             snapshot.position_y_m,
             snapshot.heading_deg,
             snapshot.timestamp_ms);
}

void vehicle_simulation_get_state(VehicleSimState &out) {
    out = VehicleSimState{};
    if (g_state_mutex == nullptr) {
        return;
    }
    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
        out = g_state;
        xSemaphoreGive(g_state_mutex);
    }
}

void vVehicleSimulationTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Vehicle simulation task started");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kSimTaskPeriodMs));
    }
}

}  // namespace agri::simulation
