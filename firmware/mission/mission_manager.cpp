#include "firmware/mission/mission_manager.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::mission {
namespace {
const char *TAG = "mission_manager";
}

void vMissionManagerTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Mission manager task initialized. Mission orchestration is intentionally stubbed.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::mission
