#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"
#include "firmware/rtos/agri_rtos.h"
#include "firmware/safety/safety_task.h"
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
    ESP_LOGI(TAG, "AgriLink boot sequence started.");
    ESP_LOGI(TAG, "Vehicle ECU is initialized in INIT state.");
    ESP_LOGI(TAG, "Safety supervision is intentionally configured as the highest-priority task.");

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Core system foundation is active. Subsystems are placeholder interfaces only.");
    vTaskDelete(nullptr);
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting AgriLink Vehicle Control ECU.");

    xTaskCreate(vSystemBootTask,
                "boot_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_BOOT,
                nullptr);

    xTaskCreate(agri::safety::vSafetyTask,
                "safety_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_SAFETY,
                nullptr);

    xTaskCreate(agri::mission::vMissionManagerTask,
                "mission_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_MISSION,
                nullptr);

    xTaskCreate(agri::navigation::vNavigationTask,
                "navigation_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_NAVIGATION,
                nullptr);

    xTaskCreate(agri::control::vMotionControlTask,
                "motion_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_CONTROL,
                nullptr);

    xTaskCreate(agri::communication::vCommunicationTask,
                "comm_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_COMMUNICATION,
                nullptr);

    xTaskCreate(agri::diagnostics::vDiagnosticsTask,
                "diagnostics_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_DIAGNOSTICS,
                nullptr);

    xTaskCreate(agri::simulation::vVehicleSimulationTask,
                "simulation_task",
                configMINIMAL_STACK_SIZE + 2048,
                nullptr,
                AGRILINK_TASK_PRIORITY_SIMULATION,
                nullptr);

    ESP_LOGI(TAG, "All placeholder RTOS tasks have been scheduled.");
}
