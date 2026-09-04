#include "firmware/communication/communication.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::communication {
namespace {
const char *TAG = "communication";
}

void vCommunicationTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Communication task initialized. Wi-Fi and TWAI-inspired messaging are placeholders.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::communication
