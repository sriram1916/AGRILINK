#include "firmware/sensors/sensor_task.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>

namespace agri::sensors {
namespace {

const char *TAG = "sensors";

// Configuration for the deterministic sensor simulation.
constexpr uint32_t kSensorPeriodMs = 1000;
constexpr double   kPi             = 3.14159265358979323846;

constexpr double kBaseLatitude   = 37.7749;   // arbitrary reference origin
constexpr double kBaseLongitude  = -122.4194;
constexpr double kBaseAltitudeM  = 15.0;
constexpr double kAmplitudeLat   = 0.001;     // small wandering amplitude
constexpr double kAmplitudeLon   = 0.001;
constexpr double kAmplitudeAltM  = 0.5;
constexpr double kAmplitudeRoll  = 2.0;       // deg
constexpr double kAmplitudePitch = 1.5;
constexpr double kAmplitudeYaw   = 30.0;      // continuous yaw drift
constexpr double kBaseSpeedMps   = 1.5;       // nominal cruise
constexpr double kAmplitudeSpeed = 0.5;
constexpr double kWheelRadiusM   = 0.35;

// Shared state for the sensor task. Mutex-protected so that future
// consumer tasks (navigation, diagnostics) can sample get_latest_snapshot()
// concurrently without racing.
static SensorSnapshot g_latest_snapshot{};
static bool          g_snapshot_ready = false;
static SemaphoreHandle_t g_snapshot_mutex = nullptr;

// Phase 7 — deterministic fault-injection flags. Set by the fault
// injector; cleared after read_sensors() applies them so they don't
// leak past the fault window.
static volatile bool g_inject_gps_invalid   = false;
static volatile bool g_inject_imu_invalid   = false;
static volatile bool g_inject_wheel_invalid = false;

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

}  // namespace

void init_sensors(void) {
    ESP_LOGI(TAG, "[AGRILINK] Sensor subsystem initialized (deterministic software source)");

    if (g_snapshot_mutex == nullptr) {
        g_snapshot_mutex = xSemaphoreCreateMutex();
    }
}

SensorSnapshot read_sensors(void) {
    SensorSnapshot s{};
    s.timestamp_ms = monotonic_ms();

    // Slow sinusoidal wander around a fixed base point. Time t is in
    // seconds, derived deterministically from the monotonic tick count.
    const double t = static_cast<double>(s.timestamp_ms) / 1000.0;

    s.gps.timestamp_ms    = s.timestamp_ms;
    s.gps.latitude_deg    = kBaseLatitude
                          + kAmplitudeLat * std::sin(t * (2.0 * kPi / 30.0));
    s.gps.longitude_deg   = kBaseLongitude
                          + kAmplitudeLon * std::cos(t * (2.0 * kPi / 45.0));
    s.gps.altitude_m      = kBaseAltitudeM
                          + kAmplitudeAltM * std::sin(t * (2.0 * kPi / 20.0));
    s.gps.valid           = true;

    s.imu.timestamp_ms    = s.timestamp_ms;
    s.imu.roll_deg        = kAmplitudeRoll  * std::sin(t * (2.0 * kPi / 7.0));
    s.imu.pitch_deg       = kAmplitudePitch * std::sin(t * (2.0 * kPi / 5.0));
    s.imu.yaw_deg         = std::fmod(kAmplitudeYaw * t / 60.0, 360.0);
    s.imu.valid           = true;

    s.wheel.timestamp_ms  = s.timestamp_ms;
    s.wheel.linear_speed_mps = kBaseSpeedMps
                             + kAmplitudeSpeed * std::sin(t * (2.0 * kPi / 10.0));
    // omega = v / r  ->  rpm = omega * 60 / (2 pi)
    s.wheel.wheel_rpm     = (s.wheel.linear_speed_mps / kWheelRadiusM)
                             * (60.0 / (2.0 * kPi));
    s.wheel.valid         = true;

    // Phase 7 — apply deterministic fault-injection flags, then clear
    // them so the abnormal condition is visible for exactly one read.
    if (g_inject_gps_invalid) {
        s.gps.valid   = false;
        g_inject_gps_invalid = false;
    }
    if (g_inject_imu_invalid) {
        s.imu.valid   = false;
        g_inject_imu_invalid = false;
    }
    if (g_inject_wheel_invalid) {
        s.wheel.valid = false;
        g_inject_wheel_invalid = false;
    }

    return s;
}

bool get_latest_snapshot(SensorSnapshot &out) {
    if (g_snapshot_mutex == nullptr) {
        return false;
    }
    if (xSemaphoreTake(g_snapshot_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    const bool ready = g_snapshot_ready;
    out = g_latest_snapshot;
    xSemaphoreGive(g_snapshot_mutex);
    return ready;
}

void vSensorTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Sensor task started");

    while (true) {
        SensorSnapshot s = read_sensors();

        if (g_snapshot_mutex != nullptr) {
            if (xSemaphoreTake(g_snapshot_mutex, portMAX_DELAY) == pdTRUE) {
                g_latest_snapshot = s;
                g_snapshot_ready  = true;
                xSemaphoreGive(g_snapshot_mutex);
            }
        }

        ESP_LOGI(TAG,
                 "[AGRILINK] Sensors: "
                 "GPS lat=%.4f lon=%.4f alt=%.2fm valid=%d | "
                 "IMU roll=%.2f pitch=%.2f yaw=%.2f valid=%d | "
                 "WHEEL vel=%.2fmps rpm=%.1f valid=%d | t=%u",
                 s.gps.latitude_deg, s.gps.longitude_deg, s.gps.altitude_m, s.gps.valid ? 1 : 0,
                 s.imu.roll_deg, s.imu.pitch_deg, s.imu.yaw_deg, s.imu.valid ? 1 : 0,
                 s.wheel.linear_speed_mps, s.wheel.wheel_rpm, s.wheel.valid ? 1 : 0,
                 s.timestamp_ms);

        vTaskDelay(pdMS_TO_TICKS(kSensorPeriodMs));
    }
}

void mark_invalid_gps(void)   { g_inject_gps_invalid   = true; }
void mark_invalid_imu(void)   { g_inject_imu_invalid   = true; }
void mark_invalid_wheel(void) { g_inject_wheel_invalid = true; }

}  // namespace agri::sensors
