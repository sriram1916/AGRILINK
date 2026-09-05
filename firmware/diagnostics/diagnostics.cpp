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
    ESP_LOGI(TAG, "[AGRILINK] Diagnostics task initialized");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

}  // namespace agri::diagnostics
