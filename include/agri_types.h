#pragma once

namespace agri {

enum class VehicleState {
    INIT,
    READY,
    AUTONOMOUS,
    DEGRADED,
    FAULT,
    SAFE_STOP,
    EMERGENCY_STOP,
    RECOVERY
};

enum class FaultCode {
    GPS_LOSS,
    COMMUNICATION_LOSS,
    WIFI_LOSS,
    OBSTACLE_DETECTED,
    SENSOR_DISAGREEMENT,
    ACTUATOR_MISMATCH,
    WATCHDOG_FAILURE,
    INVALID_COMMAND
};

struct VehicleHealth {
    VehicleState state{VehicleState::INIT};
    bool gps_valid{false};
    bool imu_valid{false};
    bool wheel_encoder_valid{false};
    bool communication_link_ok{false};
    bool wifi_link_ok{false};
    bool emergency_stop{false};
};

}  // namespace agri
