#include "firmware/communication/communication.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cstring>

#include "firmware/sensors/sensor_task.h"

namespace agri::communication {
namespace {

const char *TAG = "communication";

constexpr uint32_t kCommTaskPeriodMs = 1000;
constexpr UBaseType_t kRxQueueLength = 8;

// Loopback transport state: a single FreeRTOS queue that serves as
// both the TX output and the RX input. comm_send() copies the message
// into the queue; comm_process() drains it.
static QueueHandle_t     g_loopback_queue = nullptr;
static bool             g_transport_registered = false;
static CommStats        g_stats{};
static SemaphoreHandle_t g_stats_mutex = nullptr;

// Phase 7 — deterministic comm-loss flag. Set by the fault injector;
// honored by comm_send() (suppress) and the safety supervisor's RX
// freshness check (stale).
static volatile bool g_inject_comm_loss = false;

// Fletcher-16 checksum over the payload. Simple and deterministic.
uint16_t fletcher16(const uint8_t *data, uint16_t len) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (uint16_t i = 0; i < len; ++i) {
        sum1 = static_cast<uint16_t>((sum1 + data[i]) & 0xFF);
        sum2 = static_cast<uint16_t>((sum2 + sum1) & 0xFF);
    }
    return static_cast<uint16_t>((sum2 << 8) | sum1);
}

uint16_t compute_crc(const CommMessage &msg) {
    uint8_t header[6];
    header[0] = static_cast<uint8_t>(msg.id & 0xFF);
    header[1] = static_cast<uint8_t>((msg.id >> 8) & 0xFF);
    header[2] = static_cast<uint8_t>(msg.length & 0xFF);
    header[3] = static_cast<uint8_t>((msg.length >> 8) & 0xFF);
    header[4] = static_cast<uint8_t>(static_cast<uint8_t>(msg.type));
    header[5] = 0;
    uint16_t c1 = fletcher16(header, sizeof(header));
    uint16_t c2 = fletcher16(msg.payload, msg.length);
    return static_cast<uint16_t>(c1 ^ c2);
}

void update_stats_locked(const CommStats &delta) {
    if (g_stats_mutex == nullptr) {
        return;
    }
    if (xSemaphoreTake(g_stats_mutex, portMAX_DELAY) == pdTRUE) {
        g_stats.tx_count          += delta.tx_count;
        g_stats.rx_count          += delta.rx_count;
        g_stats.crc_errors        += delta.crc_errors;
        g_stats.tx_errors         += delta.tx_errors;
        if (delta.last_tx_timestamp_ms) {
            g_stats.last_tx_timestamp_ms = delta.last_tx_timestamp_ms;
        }
        if (delta.last_rx_timestamp_ms) {
            g_stats.last_rx_timestamp_ms = delta.last_rx_timestamp_ms;
        }
        xSemaphoreGive(g_stats_mutex);
    }
}

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

}  // namespace

void comm_init(void) {
    ESP_LOGI(TAG, "[AGRILINK] Communication subsystem initialized");

    if (g_stats_mutex == nullptr) {
        g_stats_mutex = xSemaphoreCreateMutex();
    }
}

void comm_register_loopback_transport(void) {
    if (g_transport_registered) {
        return;
    }
    g_loopback_queue = xQueueCreate(kRxQueueLength, sizeof(CommMessage));
    if (g_loopback_queue == nullptr) {
        ESP_LOGE(TAG, "[AGRILINK] Communication: loopback queue allocation failed");
        return;
    }
    g_transport_registered = true;
    ESP_LOGI(TAG, "[AGRILINK] Communication: loopback transport registered");
}

bool comm_send(const CommMessage &msg) {
    if (!g_transport_registered || g_loopback_queue == nullptr) {
        update_stats_locked(CommStats{0, 0, 0, 1, 0, 0});
        return false;
    }

    // Phase 7 — if the fault injector has armed a comm-loss fault,
    // suppress this send and count it as a tx error. The flag is
    // auto-cleared so the fault window is exactly one cycle.
    if (g_inject_comm_loss) {
        g_inject_comm_loss = false;
        update_stats_locked(CommStats{0, 0, 1, 1, 0, 0});
        ESP_LOGW(TAG, "[AGRILINK] COMM FAULT: send suppressed (injected loss)");
        return false;
    }

    CommMessage framed = msg;
    framed.crc          = compute_crc(framed);
    framed.timestamp_ms = monotonic_ms();

    // Loopback: the queue serves as both the wire and the RX input.
    if (xQueueSend(g_loopback_queue, &framed, 0) != pdTRUE) {
        update_stats_locked(CommStats{0, 0, 0, 1, 0, 0});
        return false;
    }

    ESP_LOGI(TAG,
             "[AGRILINK] COMM TX id=0x%04x type=0x%02x len=%u crc=0x%04x t=%u",
             framed.id,
             static_cast<uint8_t>(framed.type),
             framed.length,
             framed.crc,
             framed.timestamp_ms);

    update_stats_locked(CommStats{1, 0, 0, 0, framed.timestamp_ms, 0});
    return true;
}

void comm_process(void) {
    if (!g_transport_registered || g_loopback_queue == nullptr) {
        return;
    }

    CommMessage received{};
    while (xQueueReceive(g_loopback_queue, &received, 0) == pdTRUE) {
        const uint16_t expected = compute_crc(received);
        const bool     ok       = (expected == received.crc);

        if (!ok) {
            ESP_LOGW(TAG,
                     "[AGRILINK] COMM RX id=0x%04x len=%u crc=0x%04x (expected 0x%04x) valid=0",
                     received.id, received.length, received.crc, expected);
            update_stats_locked(CommStats{0, 0, 1, 0, 0, received.timestamp_ms});
            continue;
        }

        ESP_LOGI(TAG,
                 "[AGRILINK] COMM RX id=0x%04x type=0x%02x len=%u crc=OK valid=1 t=%u",
                 received.id,
                 static_cast<uint8_t>(received.type),
                 received.length,
                 received.timestamp_ms);

        update_stats_locked(CommStats{0, 1, 0, 0, 0, received.timestamp_ms});
    }
}

CommStats comm_get_stats(void) {
    CommStats out{};
    if (g_stats_mutex == nullptr) {
        return out;
    }
    if (xSemaphoreTake(g_stats_mutex, portMAX_DELAY) == pdTRUE) {
        out = g_stats;
        xSemaphoreGive(g_stats_mutex);
    }
    return out;
}

bool comm_pack_telemetry(const SensorSnapshot &snapshot, CommMessage &out) {
    static_assert(sizeof(CommTelemetryPayload) <= kCommMaxPayloadBytes,
                  "Telemetry payload exceeds CommMessage payload buffer");

    CommTelemetryPayload tp{};
    tp.gps_lat_deg   = snapshot.gps.latitude_deg;
    tp.gps_lon_deg   = snapshot.gps.longitude_deg;
    tp.gps_alt_m     = snapshot.gps.altitude_m;
    tp.imu_roll_deg  = snapshot.imu.roll_deg;
    tp.imu_pitch_deg = snapshot.imu.pitch_deg;
    tp.imu_yaw_deg   = snapshot.imu.yaw_deg;
    tp.wheel_speed_mps = snapshot.wheel.linear_speed_mps;
    tp.wheel_rpm     = snapshot.wheel.wheel_rpm;
    tp.gps_valid     = snapshot.gps.valid ? 1 : 0;
    tp.imu_valid     = snapshot.imu.valid ? 1 : 0;
    tp.wheel_valid   = snapshot.wheel.valid ? 1 : 0;

    out = CommMessage{};
    out.id     = AGRILINK_MSG_TELEMETRY;
    out.type   = CommMessageType::TELEMETRY;
    out.length = sizeof(CommTelemetryPayload);
    std::memcpy(out.payload, &tp, sizeof(CommTelemetryPayload));
    return true;
}

void vCommunicationTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Communication task started");

    uint32_t cycle = 0;
    while (true) {
        // Drain any messages that the transport has already injected
        // (loopback re-injects what we transmitted on the previous tick).
        comm_process();

        // Pack the latest sensor snapshot into a telemetry message and
        // transmit it. Future phases may add additional message types
        // (commands, acks, etc.) here.
        SensorSnapshot snap{};
        const bool have_snapshot = sensors::get_latest_snapshot(snap);

        if (have_snapshot) {
            CommMessage msg{};
            if (comm_pack_telemetry(snap, msg) && comm_send(msg)) {
                comm_process();
            } else {
                ESP_LOGW(TAG, "[AGRILINK] Communication: telemetry send skipped");
            }
        } else {
            ESP_LOGW(TAG, "[AGRILINK] Communication: no sensor snapshot available yet");
        }

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(kCommTaskPeriodMs));
    }
}

void comm_inject_loss(void) { g_inject_comm_loss = true; }

}  // namespace agri::communication
