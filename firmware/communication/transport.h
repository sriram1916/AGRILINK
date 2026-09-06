#pragma once

#include "agri_types.h"

namespace agri::communication {

enum class TransportType : uint8_t {
    LOOPBACK = 0,
    UART = 1,
    TWAI = 2
};

// Initialize the selected transport.
// The default implementation remains loopback for Wokwi.
bool transport_init(TransportType type);

// Transmit one communication message using the active transport.
bool transport_send(const CommMessage &msg);

// Receive one communication message from the active transport.
// Returns false when no message is available.
bool transport_receive(CommMessage &msg);

// Return the currently selected transport.
TransportType transport_get_type(void);

}  // namespace agri::communication