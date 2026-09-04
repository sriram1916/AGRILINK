#include "firmware/control/motion_control.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::control {
namespace {
const char *TAG = "motion_control";
}

void vMotionControlTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Motion control task initialized. Commands are placeholders only.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::control
