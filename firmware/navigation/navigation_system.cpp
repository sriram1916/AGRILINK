#include "firmware/navigation/navigation_system.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::navigation {
namespace {
const char *TAG = "navigation";
}

void vNavigationTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Navigation task initialized. Path planning is intentionally stubbed.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::navigation
