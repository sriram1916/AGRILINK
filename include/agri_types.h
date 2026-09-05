#pragma once

#include <cstdint>

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

// Phase 3 — sensor data structures.
//
// The sensor layer produces a deterministic snapshot of three independent
// channels. These types are deliberately framework-light so that the
// simulator stub used here can later be replaced with real drivers
// (u-blox GPS over UART, MPU-style IMU over I2C, quadrature encoder
// counters) without changing the consumers.

struct GpsReading {
    double   latitude_deg{0.0};
    double   longitude_deg{0.0};
    double   altitude_m{0.0};
    uint32_t timestamp_ms{0};
    bool     valid{false};
};

struct ImuReading {
    double   roll_deg{0.0};
    double   pitch_deg{0.0};
    double   yaw_deg{0.0};
    uint32_t timestamp_ms{0};
    bool     valid{false};
};

struct WheelEncoderReading {
    double   linear_speed_mps{0.0};
    double   wheel_rpm{0.0};
    uint32_t timestamp_ms{0};
    bool     valid{false};
};

struct SensorSnapshot {
    GpsReading          gps{};
    ImuReading          imu{};
    WheelEncoderReading wheel{};
    uint32_t            timestamp_ms{0};
};

// Phase 4 — communication message types and structures.
//
// The communication layer is intentionally transport-agnostic. A
// CommMessage wraps an opaque payload plus a checksum. A concrete
// transport (loopback for Wokwi, CAN/TWAI/Wi-Fi for real hardware)
// is registered at init time.

enum class CommMessageType : uint8_t {
    TELEMETRY = 0x01,
    COMMAND   = 0x02,
    ACK       = 0x03,
    HEARTBEAT = 0x04,
    RESERVED  = 0xFF,
};

constexpr uint16_t AGRILINK_MSG_TELEMETRY = 0x0001;
constexpr uint16_t AGRILINK_MSG_COMMAND   = 0x0002;
constexpr uint16_t AGRILINK_MSG_ACK       = 0x0003;
constexpr uint16_t AGRILINK_MSG_HEARTBEAT = 0x0004;

constexpr uint16_t kCommMaxPayloadBytes = 96;

struct CommMessage {
    uint16_t             id{0};
    CommMessageType      type{CommMessageType::RESERVED};
    uint16_t             length{0};
    uint8_t              payload[kCommMaxPayloadBytes]{};
    uint16_t             crc{0};
    uint32_t             timestamp_ms{0};
};

// A compact telemetry payload — a meaningful subset of the Phase 3
// sensor snapshot, suitable for transmission over a CAN/ISOBUS-like
// frame once the transport is upgraded to a real bus.
struct CommTelemetryPayload {
    double   gps_lat_deg{0.0};
    double   gps_lon_deg{0.0};
    double   gps_alt_m{0.0};
    double   imu_roll_deg{0.0};
    double   imu_pitch_deg{0.0};
    double   imu_yaw_deg{0.0};
    double   wheel_speed_mps{0.0};
    double   wheel_rpm{0.0};
    uint8_t  gps_valid{0};
    uint8_t  imu_valid{0};
    uint8_t  wheel_valid{0};
};

struct CommStats {
    uint32_t tx_count{0};
    uint32_t rx_count{0};
    uint32_t crc_errors{0};
    uint32_t tx_errors{0};
    uint32_t last_tx_timestamp_ms{0};
    uint32_t last_rx_timestamp_ms{0};
};

// Phase 5 — control + vehicle simulation data structures.
//
// ControlCommand is what a higher-level planner / operator would issue.
// ControlOutput is what the (currently open-loop) controller produces
// after validation and clamping. VehicleSimState holds the deterministic
// simulated vehicle state that the simulation task advances each tick.

struct ControlCommandPayload {
    double  desired_speed_mps{0.0};   // signed longitudinal speed
    double  steering_deg{0.0};        // signed steering angle
    uint8_t enable{0};                // 1 = apply, 0 = coast / disable
    uint8_t reserved[7]{};
};

struct ControlOutput {
    double commanded_speed_mps{0.0};
    double commanded_steering_deg{0.0};
    uint8_t enable{0};
    uint8_t clamped_speed{0};         // set if speed was clamped to limits
    uint8_t clamped_steering{0};      // set if steering was clamped to limits
};

struct VehicleSimState {
    double speed_mps{0.0};
    double steering_deg{0.0};
    double position_x_m{0.0};
    double position_y_m{0.0};
    double heading_deg{0.0};
    uint32_t timestamp_ms{0};
};

// Phase 6 — safety supervision output.
//
// The safety supervisor reads sensor/comm/control/vehicle state and
// produces a SafetyDecision describing whether the control layer is
// permitted to apply the next control output, and what state the
// vehicle should be in. Reasons are coarse on purpose — this is a
// simulated prototype, not a certified safety controller.

enum class SafetyStatus : uint8_t {
    NORMAL            = 0,
    DEGRADED_SENSORS  = 1,
    DEGRADED_COMM     = 2,
    SAFE_STOP         = 3,
    EMERGENCY_STOP    = 4,
};

struct SafetyDecision {
    SafetyStatus status{SafetyStatus::NORMAL};
    VehicleState target_state{VehicleState::READY};
    bool         allow_control{true};
    bool         clamp_speed_to_zero{false};
    uint32_t     timestamp_ms{0};
};

// Phase 7 — fault identifiers.
//
// Deterministic fault identifiers used by the fault-injection framework.
// Each ID maps to a single abnormal condition that exercises a
// specific layer (sensor / comm / control).
enum class FaultId : uint8_t {
    NONE               = 0,
    SENSOR_GPS_LOSS    = 1,
    SENSOR_IMU_LOSS    = 2,
    SENSOR_WHEEL_LOSS  = 3,
    COMM_LOSS          = 4,
    CONTROL_INVALID_CMD = 5,
};

}  // namespace agri
