#include "firmware/simulation/vehicle_simulation.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::simulation {
namespace {
const char *TAG = "vehicle_simulation";
}

void vVehicleSimulationTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Vehicle simulation task initialized. Vehicle model is placeholder only.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::simulation
