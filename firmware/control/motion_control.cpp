#include "firmware/control/motion_control.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstring>

#include "firmware/communication/communication.h"
#include "firmware/safety/safety_task.h"
#include "firmware/sensors/sensor_task.h"
#include "firmware/simulation/vehicle_simulation.h"

namespace agri::control {
namespace {

const char *TAG = "motion_control";

constexpr ControlParameters kDefaultControlParameters{
    1000,   // control_period_ms
    5.0,    // max_speed_mps
    -3.0,   // min_speed_mps
    30.0,   // max_steering_deg
    10.0    // raw_reject_speed_mps
};

ControlParameters g_control_parameters = kDefaultControlParameters;

// Phase 7 - deterministic control-fault flag. Set by the fault
// injector; honored by generate_next_command() to emit an invalid
// command (out-of-range raw speed).
static volatile bool g_inject_invalid_cmd = false;

// Deterministic command generator: slow alternating speed profile
// and a slow steering sweep. Time-driven from xTaskGetTickCount()
// so the demo is reproducible across runs.
ControlCommandPayload generate_next_command(uint32_t tick_ms) {
    // Phase 7 - fault-injection hook. When armed, emit a deliberately
    // out-of-range raw speed so the validator rejects it. Auto-clear
    // so the fault window is exactly one cycle.
    if (g_inject_invalid_cmd) {
        g_inject_invalid_cmd = false;
        ControlCommandPayload bad{};
        bad.desired_speed_mps = 50.0;   // Deliberately exceeds the raw rejection limit.
        bad.steering_deg      = 0.0;
        bad.enable            = 1;
        return bad;
    }

    const double t = static_cast<double>(tick_ms) / 1000.0;

    ControlCommandPayload c{};
    c.desired_speed_mps = 2.0 + 1.5 * std::sin(t * (2.0 * 3.14159265 / 20.0));
    c.steering_deg      = 10.0 * std::sin(t * (2.0 * 3.14159265 / 15.0));
    c.enable            = 1;
    return c;
}

uint32_t monotonic_ms(void) {
    return static_cast<uint32_t>(xTaskGetTickCount()) *
           static_cast<uint32_t>(portTICK_PERIOD_MS);
}

}  // namespace

const ControlParameters& get_control_parameters(void) {
    return g_control_parameters;
}

void control_init(void) {
    ESP_LOGI(TAG, "[AGRILINK] Control subsystem initialized");
    ESP_LOGI(TAG,
             "[AGRILINK] Control parameters: period=%lu ms max_speed=%.2f min_speed=%.2f max_steer=%.2f raw_reject=%.2f",
             static_cast<unsigned long>(g_control_parameters.control_period_ms),
             g_control_parameters.max_speed_mps,
             g_control_parameters.min_speed_mps,
             g_control_parameters.max_steering_deg,
             g_control_parameters.raw_reject_speed_mps);
}

ControlOutput validate_and_clamp_command(const ControlCommandPayload &raw) {
    ControlOutput out{};

    const ControlParameters& params = g_control_parameters;

    // NaN / Inf guard.
    if (!std::isfinite(raw.desired_speed_mps) || !std::isfinite(raw.steering_deg)) {
        ESP_LOGW(TAG, "[AGRILINK] CONTROL REJECTED: non-finite command");
        out.commanded_speed_mps    = 0.0;
        out.commanded_steering_deg = 0.0;
        out.enable                 = 0;
        return out;
    }

    // Unreasonable raw value rejection.
    if (raw.desired_speed_mps > params.raw_reject_speed_mps ||
        raw.desired_speed_mps < -params.raw_reject_speed_mps) {
        ESP_LOGW(TAG,
                 "[AGRILINK] CONTROL REJECTED: raw speed=%.2f out of bounds",
                 raw.desired_speed_mps);
        out.commanded_speed_mps    = 0.0;
        out.commanded_steering_deg = 0.0;
        out.enable                 = 0;
        return out;
    }

    // Clamp speed.
    if (raw.desired_speed_mps > params.max_speed_mps) {
        out.commanded_speed_mps = params.max_speed_mps;
        out.clamped_speed       = 1;
    } else if (raw.desired_speed_mps < params.min_speed_mps) {
        out.commanded_speed_mps = params.min_speed_mps;
        out.clamped_speed       = 1;
    } else {
        out.commanded_speed_mps = raw.desired_speed_mps;
    }

    // Clamp steering.
    if (raw.steering_deg > params.max_steering_deg) {
        out.commanded_steering_deg = params.max_steering_deg;
        out.clamped_steering       = 1;
    } else if (raw.steering_deg < -params.max_steering_deg) {
        out.commanded_steering_deg = -params.max_steering_deg;
        out.clamped_steering       = 1;
    } else {
        out.commanded_steering_deg = raw.steering_deg;
    }

    out.enable = raw.enable ? 1 : 0;
    return out;
}

bool control_pack_command(const ControlCommandPayload &cmd, CommMessage &out) {
    static_assert(sizeof(ControlCommandPayload) <= kCommMaxPayloadBytes,
                  "ControlCommand payload exceeds CommMessage payload buffer");

    out            = CommMessage{};
    out.id         = AGRILINK_MSG_COMMAND;
    out.type       = CommMessageType::COMMAND;
    out.length     = sizeof(ControlCommandPayload);
    std::memcpy(out.payload, &cmd, sizeof(ControlCommandPayload));
    return true;
}

bool control_unpack_command(const CommMessage &msg, ControlCommandPayload &out) {
    if (msg.type != CommMessageType::COMMAND) {
        return false;
    }
    if (msg.id != AGRILINK_MSG_COMMAND) {
        return false;
    }
    if (msg.length != sizeof(ControlCommandPayload)) {
        return false;
    }
    std::memcpy(&out, msg.payload, sizeof(ControlCommandPayload));
    return true;
}

void vMotionControlTask(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "[AGRILINK] Motion control task started");

    uint32_t cycle = 0;
    while (true) {
        // 1) Generate the next deterministic command.
        const uint32_t now_ms = monotonic_ms();
        ControlCommandPayload raw_cmd = generate_next_command(now_ms);

        ESP_LOGI(TAG,
                 "[AGRILINK] CONTROL CMD raw spd=%.2f steer=%.2f en=%u",
                 raw_cmd.desired_speed_mps,
                 raw_cmd.steering_deg,
                 static_cast<unsigned>(raw_cmd.enable));

        // 2) Encode into a CommMessage and transmit via the
        //    Phase 4 communication layer (loopback transport).
        CommMessage msg{};
        if (control_pack_command(raw_cmd, msg)) {
            if (communication::comm_send(msg)) {
                // The loopback transport re-injects synchronously;
                // process it so stats + RX logs are up to date.
                communication::comm_process();
            }
        }

        // 3) Decode the looped-back command and run validation.
        //    The transport's RX queue is FIFO; the last message
        //    dequeued is our own command.
        bool decoded = false;
        if (communication::comm_get_stats().rx_count > 0) {
            // Peek is not in the API; we rely on comm_process() having
            // already drained the queue. Re-create a minimal decode
            // path: pack + send already exercised the round trip;
            // for the validate/clamp demo we validate the original
            // raw command here (which is what would arrive after the
            // round-trip too).
            decoded = true;
        }

        ControlOutput out_cmd{};
        if (decoded) {
            out_cmd = validate_and_clamp_command(raw_cmd);
            ESP_LOGI(TAG,
                     "[AGRILINK] CONTROL VALID spd=%.2f steer=%.2f en=%u clamp_spd=%u clamp_str=%u",
                     out_cmd.commanded_speed_mps,
                     out_cmd.commanded_steering_deg,
                     static_cast<unsigned>(out_cmd.enable),
                     static_cast<unsigned>(out_cmd.clamped_speed),
                     static_cast<unsigned>(out_cmd.clamped_steering));
        } else {
            ESP_LOGW(TAG, "[AGRILINK] CONTROL: no command decoded");
        }

        // 4) Submit the validated control output to the vehicle
        //    simulation and let it advance one tick.
        //
        //    Safety supervision (Phase 6) gates the output before
        //    it reaches the vehicle simulation. If the supervisor
        //    detects an unsafe condition it zeroes the output here.
        const bool inhibited = safety::safety_apply_to_control(out_cmd);
        if (inhibited) {
            ESP_LOGW(TAG, "[AGRILINK] SAFETY inhibited control output");
        }
        simulation::vehicle_simulation_apply_control(out_cmd);
        simulation::vehicle_simulation_tick();

        // 5) Read back the latest simulated vehicle state and the
        //    latest sensor snapshot for feedback logging.
        VehicleSimState sim{};
        simulation::vehicle_simulation_get_state(sim);

        SensorSnapshot snap{};
        const bool have_snap = sensors::get_latest_snapshot(snap);

        ESP_LOGI(TAG,
                 "[AGRILINK] CONTROL OUTPUT -> VEHICLE spd=%.2f steer=%.2f en=%u",
                 out_cmd.commanded_speed_mps,
                 out_cmd.commanded_steering_deg,
                 static_cast<unsigned>(out_cmd.enable));

        ESP_LOGI(TAG,
                 "[AGRILINK] VEHICLE STATE spd=%.2f steer=%.2f x=%.2f y=%.2f hdg=%.2f",
                 sim.speed_mps,
                 sim.steering_deg,
                 sim.position_x_m,
                 sim.position_y_m,
                 sim.heading_deg);

        if (have_snap) {
            ESP_LOGI(TAG,
                     "[AGRILINK] CONTROL FEEDBACK cmd_spd=%.2f actual_spd=%.2f cmd_steer=%.2f actual_steer=%.2f",
                     out_cmd.commanded_speed_mps,
                     snap.wheel.linear_speed_mps,
                     out_cmd.commanded_steering_deg,
                     sim.steering_deg);
        }

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(g_control_parameters.control_period_ms));
    }
}

void inject_invalid_command(void) { g_inject_invalid_cmd = true; }

}  // namespace agri::control
