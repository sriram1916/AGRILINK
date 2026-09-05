#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"
#include "firmware/rtos/agri_rtos.h"
#include "firmware/heartbeat/heartbeat.h"
#include "firmware/sensors/sensor_task.h"
#include "firmware/communication/communication.h"
#include "firmware/simulation/vehicle_simulation.h"
#include "firmware/safety/safety_task.h"
#include "firmware/fault_injection/fault_injector.h"
#include "firmware/tests/test_runner.h"
#include "firmware/mission/mission_manager.h"
#include "firmware/navigation/navigation_system.h"
#include "firmware/control/motion_control.h"
#include "firmware/communication/communication.h"
#include "firmware/diagnostics/diagnostics.h"
#include "firmware/simulation/vehicle_simulation.h"

namespace {
const char *TAG = "agri_main";

void vSystemBootTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] System boot - Phase 1: Initializing hardware interfaces");

    // The heartbeat subsystem is initialized by app_main() before any task is
    // created, so the mutex and LED are already available here.

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "[AGRILINK] System boot - Phase 2: Core foundation initialized");
    ESP_LOGI(TAG, "[AGRILINK] Vehicle ECU state: INIT");
    ESP_LOGI(TAG, "[AGRILINK] Safety supervision is configured as highest-priority task");

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "[AGRILINK] System boot - Phase 3: All subsystem tasks scheduled");
    agri::heartbeat::set_system_state(agri::heartbeat::SystemState::READY);
    ESP_LOGI(TAG, "[AGRILINK] Vehicle ECU state: READY");

    vTaskDelete(nullptr);
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "[AGRILINK] Starting Vehicle Control ECU");

    // Initialize the heartbeat subsystem (mutex + GPIO2 LED) BEFORE creating
    // any task that may run vHeartbeatTask, so the mutex is guaranteed to
    // exist when the heartbeat task first takes it.
    agri::heartbeat::init_heartbeat();
    agri::heartbeat::set_system_state(agri::heartbeat::SystemState::INIT);

    // Initialize the sensor subsystem (Phase 3) BEFORE creating the
    // sensor task. Same ordering rule as the heartbeat subsystem.
    agri::sensors::init_sensors();

    // Initialize the communication subsystem (Phase 4) and register a
    // Wokwi-friendly loopback transport BEFORE the communication task
    // starts, so its first comm_send() / comm_process() calls succeed.
    agri::communication::comm_init();
    agri::communication::comm_register_loopback_transport();

    // Initialize the vehicle simulation subsystem (Phase 5) BEFORE
    // the control and simulation tasks start, so the simulated state
    // is ready before any control output is submitted.
    agri::simulation::vehicle_simulation_init();

    // Initialize the safety supervisor (Phase 6) BEFORE the control
    // task runs, so its first safety_evaluate() call has access to
    // the initialized decision state.
    agri::safety::safety_init();

    // Initialize the fault injector (Phase 7). It schedules faults
    // deterministically; it is a TEST mechanism and does not perform
    // safety responses itself.
    agri::fault_injection::fault_injector_init();

    // Initialize the test runner (Phase 8). It only observes existing
    // subsystem state and emits [TEST] log lines; it does not modify
    // any other module's behavior.
    agri::tests::test_runner_init();

    xTaskCreate(vSystemBootTask,
                "boot_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_BOOT,
                nullptr);

    xTaskCreate(agri::heartbeat::vHeartbeatTask,
                "heartbeat_task",
                configMINIMAL_STACK_SIZE + 1024,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_BOOT + 1,  // Slightly higher than boot
                nullptr);

    xTaskCreate(agri::sensors::vSensorTask,
                "sensor_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_SENSORS,
                nullptr);

    xTaskCreate(agri::safety::vSafetyTask,
                "safety_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_SAFETY,
                nullptr);

    xTaskCreate(agri::mission::vMissionManagerTask,
                "mission_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_MISSION,
                nullptr);

    xTaskCreate(agri::navigation::vNavigationTask,
                "navigation_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_NAVIGATION,
                nullptr);

    xTaskCreate(agri::control::vMotionControlTask,
                "motion_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_CONTROL,
                nullptr);

    xTaskCreate(agri::communication::vCommunicationTask,
                "comm_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_COMMUNICATION,
                nullptr);

    xTaskCreate(agri::diagnostics::vDiagnosticsTask,
                "diagnostics_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_DIAGNOSTICS,
                nullptr);

    xTaskCreate(agri::simulation::vVehicleSimulationTask,
                "simulation_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_SIMULATION,
                nullptr);

    xTaskCreate(agri::fault_injection::vFaultInjectorTask,
                "fault_injector_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_FAULT_INJECTION,
                nullptr);

    xTaskCreate(agri::tests::vTestRunnerTask,
                "test_runner_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                agri::AGRILINK_TASK_PRIORITY_TESTS,
                nullptr);

    ESP_LOGI(TAG, "[AGRILINK] All RTOS tasks have been created and scheduled");
}
