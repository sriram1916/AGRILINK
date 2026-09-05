#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "agri_types.h"

namespace agri::communication {

// Phase 4 — communication subsystem.
//
// The subsystem is transport-agnostic. A concrete transport
// (loopback for Wokwi, CAN/TWAI/Wi-Fi for real hardware) is
// registered with comm_register_loopback_transport() (or a future
// comm_register_can_transport() etc.). Messages are placed into the
// active transport's receive queue by the transport itself; the
// communication task drains the queue, validates checksums, and
// exposes stats.

// One-time initialization. Must be called before vCommunicationTask.
void comm_init(void);

// Register the Wokwi-friendly loopback transport. The loopback
// transport immediately re-injects transmitted messages into its
// own receive queue, so TX and RX are observable on the same node.
// A real hardware transport would replace this call.
void comm_register_loopback_transport(void);

// Send a message via the active transport. Returns true on success.
bool comm_send(const CommMessage &msg);

// Drain the receive queue, validate each message, and update stats.
// Safe to call repeatedly. Called periodically from vCommunicationTask.
void comm_process(void);

// Snapshot of communication statistics.
CommStats comm_get_stats(void);

// Pack a sensor snapshot into a TELEMETRY CommMessage. Returns true
// if packing succeeded (payload fits).
bool comm_pack_telemetry(const SensorSnapshot &snapshot,
                         CommMessage         &out);

// FreeRTOS communication task. Runs at AGRILINK_TASK_PRIORITY_COMMUNICATION.
void vCommunicationTask(void *arg);

// Phase 7 — fault-injection hook. When active, the next comm_send()
// returns false (counted as a tx_error) and the supervisor observes
// stale RX. The hook auto-clears after one cycle so the fault window
// is exactly one cycle long unless re-armed.
void comm_inject_loss(void);

}  // namespace agri::communication
