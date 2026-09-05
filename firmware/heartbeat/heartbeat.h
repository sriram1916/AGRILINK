#pragma once

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::heartbeat {

enum class SystemState {
    INIT,
    READY,
    RUNNING,
    WARNING,
    ERROR,
    SHUTDOWN
};

// Simple getter/setter for system state
SystemState get_system_state(void);
void set_system_state(SystemState state);

// Heartbeat task
void vHeartbeatTask(void *arg);

// Initialize heartbeat
void init_heartbeat(void);

}  // namespace agri::heartbeat

