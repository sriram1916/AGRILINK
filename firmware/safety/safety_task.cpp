#include "firmware/safety/safety_task.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::safety {
namespace {
const char *TAG = "safety_task";
}

void vSafetyTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Safety task initialized. This is a placeholder for safety supervision.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::safety
