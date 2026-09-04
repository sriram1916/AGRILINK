#include "firmware/diagnostics/diagnostics.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::diagnostics {
namespace {
const char *TAG = "diagnostics";
}

void vDiagnosticsTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Diagnostics task initialized. Monitoring interfaces are placeholders.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}  // namespace agri::diagnostics
