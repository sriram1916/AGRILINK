#include "firmware/communication/transport.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace agri::communication {
namespace {

constexpr UBaseType_t kRxQueueLength = 8;

TransportType g_transport_type = TransportType::LOOPBACK;
QueueHandle_t g_loopback_queue = nullptr;

}  // namespace

bool transport_init(TransportType type) {
    if (type != TransportType::LOOPBACK) {
        return false;
    }

    if (g_loopback_queue == nullptr) {
        g_loopback_queue = xQueueCreate(kRxQueueLength, sizeof(CommMessage));
        if (g_loopback_queue == nullptr) {
            return false;
        }
    }

    g_transport_type = type;
    return true;
}

bool transport_send(const CommMessage &msg) {
    if (g_transport_type != TransportType::LOOPBACK ||
        g_loopback_queue == nullptr) {
        return false;
    }

    return xQueueSend(g_loopback_queue, &msg, 0) == pdTRUE;
}

bool transport_receive(CommMessage &msg) {
    if (g_transport_type != TransportType::LOOPBACK ||
        g_loopback_queue == nullptr) {
        return false;
    }

    return xQueueReceive(g_loopback_queue, &msg, 0) == pdTRUE;
}

TransportType transport_get_type(void) {
    return g_transport_type;
}

}  // namespace agri::communication