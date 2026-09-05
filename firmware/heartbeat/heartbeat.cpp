#include "firmware/heartbeat/heartbeat.h"

#include <driver/gpio.h>
#include <freertos/semphr.h>

namespace agri::heartbeat {
namespace {
const char *TAG = "heartbeat";

constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_2;

static SystemState g_system_state = SystemState::INIT;
static SemaphoreHandle_t g_state_mutex = nullptr;

const char* state_to_string(SystemState state) {
    switch (state) {
        case SystemState::INIT:      return "INIT";
        case SystemState::READY:     return "READY";
        case SystemState::RUNNING:   return "RUNNING";
        case SystemState::WARNING:   return "WARNING";
        case SystemState::ERROR:     return "ERROR";
        case SystemState::SHUTDOWN:  return "SHUTDOWN";
        default:                     return "UNKNOWN";
    }
}

}  // namespace

void init_heartbeat(void) {
    ESP_LOGI(TAG, "[AGRILINK] Initializing heartbeat system");

    g_state_mutex = xSemaphoreCreateMutex();
    if (!g_state_mutex) {
        ESP_LOGE(TAG, "[AGRILINK] Failed to create state mutex");
        return;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << STATUS_LED_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "[AGRILINK] Failed to configure status LED pin");
        return;
    }

    gpio_set_level(STATUS_LED_PIN, 0);
    ESP_LOGI(TAG, "[AGRILINK] Heartbeat system initialized, LED on GPIO %d", STATUS_LED_PIN);
}

void vHeartbeatTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Heartbeat task started");

    uint32_t cycle = 0;
    bool led_state = false;

    while (true) {
        led_state = !led_state;
        gpio_set_level(STATUS_LED_PIN, led_state ? 1 : 0);

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        SystemState current_state = g_system_state;
        xSemaphoreGive(g_state_mutex);

        ESP_LOGI(TAG, "[AGRILINK] Heartbeat %u - State: %s, LED: %s",
                 cycle, state_to_string(current_state), led_state ? "ON" : "OFF");

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void set_system_state(SystemState state) {
    if (g_state_mutex) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        SystemState old_state = g_system_state;
        g_system_state = state;
        xSemaphoreGive(g_state_mutex);

        if (old_state != state) {
            ESP_LOGI(TAG, "[AGRILINK] State transition: %s -> %s",
                     state_to_string(old_state), state_to_string(state));
        }
    }
}

SystemState get_system_state(void) {
    SystemState state = SystemState::INIT;
    if (g_state_mutex) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        state = g_system_state;
        xSemaphoreGive(g_state_mutex);
    }
    return state;
}

}  // namespace agri::heartbeat
