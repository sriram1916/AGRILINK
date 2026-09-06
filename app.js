/*
 * AgriLink Demo Dashboard — app logic.
 *
 * Standalone browser simulation: replays the AgriLink data model defined in
 * include/agri_types.h with simulated values. It does NOT connect to the ESP32
 * ECU (no serial/WebSerial, no Wi-Fi, no backend server). All behavior is a
 * client-side model of the documented firmware behavior, for demonstration.
 */
(() => {
    "use strict";

    /* ---- AgriLink enums (mirror include/agri_types.h) ---- */
    const VehicleState = {
        INIT: 0, READY: 1, AUTONOMOUS: 2, DEGRADED: 3,
        FAULT: 4, SAFE_STOP: 5, EMERGENCY_STOP: 6, RECOVERY: 7
    };
    const VehicleStateLabel = ["INIT", "READY", "AUTONOMOUS", "DEGRADED", "FAULT", "SAFE_STOP", "EMERGENCY_STOP", "RECOVERY"];
    const SafetyStatus = { NORMAL: 0, DEGRADED_SENSORS: 1, DEGRADED_COMM: 2, SAFE_STOP: 3, EMERGENCY_STOP: 4 };
    const SafetyStatusLabel = ["NORMAL", "DEGRADED_SENSORS", "DEGRADED_COMM", "SAFE_STOP", "EMERGENCY_STOP"];
    const FaultId = {
        NONE: 0, GPS_LOSS: 1, IMU_LOSS: 2, WHEEL_LOSS: 3,
        COMM_LOSS: 4, INVALID_COMMAND: 5
    };
    const CommMessageType = { TELEMETRY: 0x01, COMMAND: 0x02, ACK: 0x03, HEARTBEAT: 0x04 };

    /* ---- Deterministic PRNG so the demo is reproducible ---- */
    let rngState = 42;
    function rand() {
        rngState = (rngState * 1664525 + 1013904223) >>> 0;
        return ((rngState >> 16) & 0x7fff) / 0x7fff;
    }
    function rndSigned(range) { return (rand() - 0.5) * 2 * range; }

    /* ---- Fletcher-16 (mirrors firmware compute_crc) ---- */
    function fletcher16(data) {
        let s1 = 0, s2 = 0;
        for (let i = 0; i < data.length; i++) {
            s1 = (s1 + data[i]) & 0xff;
            s2 = (s2 + s1) & 0xff;
        }
        return ((s2 << 8) | s1) & 0xffff;
    }

    /* ---- Application state ---- */
    const state = {
        uptime: 0,
        startedAt: Date.now(),
        vehicleState: VehicleState.READY,
        activeFaults: new Set(),
        lastFaultEvent: 0,

        // sim vehicle state (meters, local tangent plane)
        x: 0, y: 0, headingDeg: 0, speed: 2.6, steering: 0,

        // telemetry
        gps: { lat: 37.7749, lon: -122.4194, alt: 5.0, valid: true },
        imu: { roll: 0, pitch: 0, yaw: 0, valid: true },
        wheel: { speed: 0, rpm: 0, valid: true },

        // control output
        ctrl: { speed: 0, steering: 0, enable: false, clampSpeed: false, clampSteer: false },

        // communication stats
        comm: { tx: 0, rx: 0, crcOk: 0, crcErr: 0, txErr: 0, lastTx: 0, lastRx: 0 },

        // safety decision
        safety: { status: SafetyStatus.NORMAL, targetState: VehicleState.AUTONOMOUS,
                  allowControl: true, clampSpeedToZero: false },

        commLog: []
    };

    /* ---- DOM helpers ---- */
    const $ = (id) => document.getElementById(id);
    function setText(id, v) { if ($(id)) $(id).textContent = v; }
    function setTextHtml(id, html) { if ($(id)) $(id).innerHTML = html; }
    function badge(ok, okTxt, badTxt) {
        return ok ? `<span class="badge badge-ok">${okTxt}</span>` : `<span class="badge badge-bad">${badTxt}</span>`;
    }

    /* ---- Vehicle simulation ---- */
    function updateVehicle(deltaMs) {
        // When safety clamps to zero, the vehicle holds position.
        if (state.safety.clampSpeedToZero) {
            state.speed *= 0.85;
            if (state.speed < 0.02) state.speed = 0;
        } else if (state.vehicleState === VehicleState.SAFE_STOP ||
                   state.vehicleState === VehicleState.EMERGENCY_STOP) {
            state.speed = 0;
        } else {
            // Gentle maneuver: oscillate steering, hold a mild forward speed.
            state.steering = 12 * Math.sin(state.uptime / 6000);
            state.speed = Math.max(0.4, 2.4 + 0.6 * Math.sin(state.uptime / 4500));
        }

        const dt = deltaMs / 1000;
        const yaw = headingToRad(state.headingDeg);
        state.x += state.speed * Math.cos(yaw) * dt;
        state.y += state.speed * Math.sin(yaw) * dt;
        state.headingDeg = (state.headingDeg + state.steering * 0.6 * dt + 360) % 360;

        // Telemetry sources. Faults blank out their sources to mirror ECU behavior.
        const gpsLoss = state.activeFaults.has("GPS_LOSS");
        const imuLoss = state.activeFaults.has("IMU_LOSS");
        const wheelLoss = state.activeFaults.has("WHEEL_LOSS");

        state.gps.valid = !gpsLoss;
        if (!gpsLoss) {
            state.gps.lat = 37.7749 + state.y * 8.98e-6;
            state.gps.lon = -122.4194 + state.x * 8.98e-6 / Math.cos(headingToRad(37.7749));
            state.gps.alt = 5.0 + state.y * 0.0002;
        }

        state.imu.valid = !imuLoss;
        if (!imuLoss) {
            state.imu.roll = rndSigned(0.6) + 1.6 * Math.sin(state.uptime / 1100);
            state.imu.pitch = rndSigned(0.4) - 0.9 * Math.cos(state.uptime / 900);
            state.imu.yaw = state.headingDeg;
        }

        state.wheel.valid = !wheelLoss;
        if (!wheelLoss) {
            const wspeed = state.speed + rndSigned(0.05);
            state.wheel.speed = wspeed;
            state.wheel.rpm = wspeed * 1000 / (2 * Math.PI * 0.32); // ~32cm tire
        }
    }
    function headingToRad(h) { return h * Math.PI / 180; }

    /* ---- Communication (loopback telemetry) ---- */
    function tickComm() {
        const commLoss = state.activeFaults.has("COMMUNICATION_LOSS");
        const msg = buildTelemetryMessage();
        state.comm.lastTx = state.uptime;

        if (commLoss) {
            // Mirror firmware fault-injection: send suppressed, counted as tx error.
            state.comm.txErr++;
            state.commLog.push(logLine("COMM FAULT: send suppressed (injected loss)", "warn"));
            // A fraction of the time the lossy link still records a corrupt frame.
            if (rand() < 0.35) { state.comm.crcErr++; }
            return;
        }

        state.comm.tx++;
        // Loopback: the TX frame is echoed as an RX frame on this node.
        state.comm.rx++;
        state.comm.lastRx = state.uptime;
        const ok = true; // clean loopback path -> CRC validates
        state.comm.crcOk++;
        state.commLog.push(logLine(
            `COMM TX id=0x${msg.id.toString(16).padStart(4, "0")} type=0x${msg.type.toString(16)} ` +
            `len=${msg.length} crc=0x${msg.crc.toString(16).padStart(4, "0")} t=${msg.timestamp_ms}`, "ok"
        ));
        state.commLog.push(logLine(
            `COMM RX id=0x${msg.id.toString(16).padStart(4, "0")} len=${msg.length} crc=OK valid=1 t=${msg.timestamp_ms}`, "ok"
        ));
        if (state.commLog.length > 120) state.commLog.shift();
    }

    function buildTelemetryMessage() {
        // CommTelemetryPayload is 8 doubles (64 B) + 3 status bytes + 5 B
        // padding => sizeof == 72, which is the wire "length" the firmware
        // reports (out.length = sizeof(CommTelemetryPayload)). Mirror that.
        const payload = new Uint8Array(72);
        let off = 0;
        const putDouble = (v) => { new Float64Array(payload.buffer, off, 1)[0] = v; off += 8; };
        putDouble(state.gps.lat);
        putDouble(state.gps.lon);
        putDouble(state.gps.alt);
        putDouble(state.imu.roll);
        putDouble(state.imu.pitch);
        putDouble(state.imu.yaw);
        putDouble(state.wheel.speed);
        putDouble(state.wheel.rpm);
        payload[64] = state.gps.valid ? 1 : 0;
        payload[65] = state.imu.valid ? 1 : 0;
        payload[66] = state.wheel.valid ? 1 : 0;
        // bytes 67..71 are zero padding (struct padding)

        const msg = {
            id: 0x0001,
            type: CommMessageType.TELEMETRY,
            length: 72,
            payload,
            crc: 0,
            timestamp_ms: Math.round(state.uptime)
        };
        msg.crc = computeCrc(msg);
        return msg;
    }
    function computeCrc(msg) {
        const header = new Uint8Array([
            msg.id & 0xff, (msg.id >> 8) & 0xff,
            msg.length & 0xff, (msg.length >> 8) & 0xff,
            msg.type, 0x00
        ]);
        return fletcher16([...header, ...Array.from(msg.payload)]);
    }

    /* ---- Control output ---- */
    function updateControl() {
        const invalid = state.activeFaults.has("INVALID_COMMAND");
        state.ctrl.enable = state.safety.allowControl && !state.safety.clampSpeedToZero;

        if (invalid) {
            state.ctrl.speed = 0;
            state.ctrl.steering = 0;
            state.ctrl.clampSpeed = true;
            state.ctrl.clampSteer = true;
            state.activeFaults.add("__INVALID_CMD_ACTIVE__");
        } else {
            // Apply steering; speed follows safety decision.
            state.ctrl.steering = state.steering;
            state.ctrl.speed = state.safety.clampSpeedToZero ? 0 : state.speed;
            state.ctrl.clampSpeed = state.safety.clampSpeedToZero;
            state.ctrl.clampSteer = false;
        }
    }

    /* ---- Safety supervisor (qualitative mirror of safety supervisor) ---- */
    function evaluateSafety() {
        const faults = state.activeFaults;
        const hasComm = faults.has("COMMUNICATION_LOSS");
        const hasGps = faults.has("GPS_LOSS");
        const hasImu = faults.has("IMU_LOSS");
        const hasWheel = faults.has("WHEEL_LOSS");

        const sensorsOk = !(hasGps || hasImu || hasWheel);
        const commOk = !hasComm;

        if (hasComm && hasGps) {
            state.safety.status = SafetyStatus.EMERGENCY_STOP;
            state.safety.targetState = VehicleState.EMERGENCY_STOP;
            state.safety.allowControl = false;
            state.safety.clampSpeedToZero = true;
        } else if (hasComm) {
            state.safety.status = SafetyStatus.DEGRADED_COMM;
            state.safety.clampSpeedToZero = true;
            state.safety.targetState = VehicleState.SAFE_STOP;
            state.safety.allowControl = false;
        } else if (!sensorsOk) {
            state.safety.status = SafetyStatus.DEGRADED_SENSORS;
            state.safety.clampSpeedToZero = true;
            state.safety.targetState = VehicleState.DEGRADED;
            state.safety.allowControl = false;
        } else {
            state.safety.status = SafetyStatus.NORMAL;
            state.safety.targetState = VehicleState.AUTONOMOUS;
            state.safety.clampSpeedToZero = false;
            state.safety.allowControl = true;
        }

        // Vehicle state from safety decision.
        const ps = state.safety.status;
        if (ps === SafetyStatus.EMERGENCY_STOP) state.vehicleState = VehicleState.EMERGENCY_STOP;
        else if (ps === SafetyStatus.SAFE_STOP) state.vehicleState = VehicleState.SAFE_STOP;
        else if (ps === SafetyStatus.DEGRADED_SENSORS || ps === SafetyStatus.DEGRADED_COMM) state.vehicleState = VehicleState.DEGRADED;
        else state.vehicleState = VehicleState.AUTONOMOUS;
    }

    /* ---- Event log helpers ---- */
    function logLine(msg, cls) {
        const t = `[${fmtMs(state.uptime)}]`;
        return { html: `${t} ${msg}`, cls };
    }

    /* ---- Rendering ---- */
    function fmtMs(ms) {
        const total = ms / 1000;
        const mm = Math.floor(total / 60);
        const ss = Math.floor(total % 60);
        const mmm = Math.floor(ms % 1000);
        return `${String(mm).padStart(2, "0")}:${String(ss).padStart(2, "0")}.${String(mmm).padStart(3, "0")}`;
    }
    function fmtUptime(ms) {
        const s = Math.floor(ms / 1000);
        return `${String(Math.floor(s / 3600)).padStart(2, "0")}:${String(Math.floor((s % 3600) / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
    }
    function fmt(n, d = 2) { return (typeof n === "number") ? n.toFixed(d) : "--"; }

    function render() {
        // system status
        setBadge("vehicle-state", VehicleStateLabel[state.vehicleState], state.vehicleStateColor());
        setBadge("safety-status", SafetyStatusLabel[state.safety.status], state.safetyStatusColor());
        setBadge("ecu-health", state.safety.allowControl ? "HEALTHY" : "DEGRADED", state.safety.allowControl ? "ok" : "bad");
        setBadge("link-status", state.activeFaults.has("COMMUNICATION_LOSS") ? "LOST" : "OK", state.activeFaults.has("COMMUNICATION_LOSS") ? "bad" : "ok");
        setBadge("comm-ok", state.activeFaults.has("COMMUNICATION_LOSS") ? "DEGRADED" : "LINK OK", state.activeFaults.has("COMMUNICATION_LOSS") ? "bad" : "ok");
        setBadge("estop", state.vehicleState === VehicleState.EMERGENCY_STOP ? "ACTIVE" : "OK", state.vehicleState === VehicleState.EMERGENCY_STOP ? "bad" : "ok");

        // telemetry
        setText("gps-lat", state.gps.valid ? fmt(state.gps.lat, 6) : "— (no fix)");
        setText("gps-lon", state.gps.valid ? fmt(state.gps.lon, 6) : "— (no fix)");
        setText("gps-alt", state.gps.valid ? fmt(state.gps.alt, 3) + " m" : "—");
        setBadgeInline("gps-valid", state.gps.valid);

        setBadgeInline("imu-valid", state.imu.valid);
        setText("imu-roll", state.imu.valid ? fmt(state.imu.roll, 2) + "°" : "—");
        setText("imu-pitch", state.imu.valid ? fmt(state.imu.pitch, 2) + "°" : "—");
        setText("imu-yaw", state.imu.valid ? fmt(state.imu.yaw, 1) + "°" : "—");

        setBadgeInline("wheel-valid", state.wheel.valid);
        setText("wheel-speed", state.wheel.valid ? fmt(state.wheel.speed, 2) + " m/s" : "—");
        setText("wheel-rpm", state.wheel.valid ? fmt(state.wheel.rpm, 0) + " rpm" : "—");

        // vehicle sim
        setText("sim-x", fmt(state.x, 2));
        setText("sim-y", fmt(state.y, 2));
        setText("sim-speed", fmt(state.speed, 2) + " m/s");
        setText("sim-steer", fmt(state.steering, 2) + "°");
        setText("sim-heading", fmt(state.headingDeg % 360, 1) + "°");

        // comm
        setText("comm-tx", state.comm.tx);
        setText("comm-rx", state.comm.rx);
        setText("comm-crc-ok", state.comm.crcOk);
        setText("comm-crc-err", state.comm.crcErr);
        setText("comm-tx-err", state.comm.txErr);
        setText("comm-last-tx", state.comm.lastTx);
        setText("comm-last-rx", state.comm.lastRx);
        renderCommLog();

        // control output
        setText("ctrl-speed", fmt(state.ctrl.speed, 2) + " m/s");
        setText("ctrl-steer", fmt(state.ctrl.steering, 2) + "°");
        setBadge("ctrl-enable", state.ctrl.enable ? "ENABLED" : "DISABLED", state.ctrl.enable ? "ok" : "bad");
        setBadgeInline("ctrl-clamp-speed", state.ctrl.clampSpeed);
        setBadgeInline("ctrl-clamp-steer", state.ctrl.clampSteer);

        // safety
        setBadge("safety-status-v", SafetyStatusLabel[state.safety.status], state.safetyStatusColor());
        setBadge("safety-target", VehicleStateLabel[state.safety.targetState], "muted");
        setBadge("safety-allow", state.safety.allowControl ? "ALLOW" : "BLOCK", state.safety.allowControl ? "ok" : "bad");
        setBadgeInline("safety-clamp", state.safety.clampSpeedToZero);
        setText("active-faults", state.activeFaults.size ? [...state.activeFaults].join(", ") : "none");

        // uptime
        setText("uptime", "up " + fmtUptime(state.uptime));
        drawCanvas();
    }

    function setBadge(id, label, kind) {
        const el = $(id);
        if (!el) return;
        let cls = "badge badge-off";
        if (kind === "ok") cls = "badge badge-ok";
        else if (kind === "bad") cls = "badge badge-bad";
        else if (kind === "warn") cls = "badge badge-warn";
        el.innerHTML = `<span class="${cls}">${label}</span>`;
    }
    function setBadgeInline(id, on) {
        const el = $(id);
        if (!el) return;
        const cls = on ? "badge badge-ok" : "badge badge-off";
        el.innerHTML = `<span class="${cls}">${on ? "VALID" : "INVALID"}</span>`;
    }
    function renderCommLog() {
        const el = $("comm-log");
        if (!el) return;
        el.textContent = state.commLog.slice(-10).map(l => l.html).join("\n");
        el.scrollTop = el.scrollHeight;
    }
    function renderFaultLog() {
        const el = $("fault-log");
        if (!el) return;
        if (!state.recentEvents) state.recentEvents = [];
        el.innerHTML = state.recentEvents.slice(-60).reverse().map(e =>
            `<li class="${e.cls}">${e.html}</li>`).join("");
    }

    /* ---- vehicle state color helpers ---- */
    state.vehicleStateColor = function () {
        switch (state.vehicleState) {
            case VehicleState.AUTONOMOUS: return "ok";
            case VehicleState.READY: return "warn";
            case VehicleState.DEGRADED: return "warn";
            case VehicleState.FAULT:
            case VehicleState.SAFE_STOP: return "bad";
            case VehicleState.EMERGENCY_STOP: return "bad";
            default: return "off";
        }
    };
    state.safetyStatusColor = function () {
        switch (state.safety.status) {
            case SafetyStatus.NORMAL: return "ok";
            case SafetyStatus.DEGRADED_SENSORS: return "warn";
            case SafetyStatus.DEGRADED_COMM: return "warn";
            case SafetyStatus.SAFE_STOP: return "bad";
            case SafetyStatus.EMERGENCY_STOP: return "bad";
            default: return "off";
        }
    };

    /* ---- Canvas: vehicle position + heading ---- */
    let canvasPts = [];
    function drawCanvas() {
        const c = $("drive-canvas");
        if (!c) return;
        const ctx = c.getContext("2d");
        const w = c.width, h = c.height;
        ctx.clearRect(0, 0, w, h);
        // grid
        ctx.strokeStyle = "rgba(79,159,255,.06)";
        ctx.lineWidth = 1;
        for (let i = 0; i <= 8; i++) {
            ctx.beginPath(); ctx.moveTo(i * w / 8, 0); ctx.lineTo(i * w / 8, h); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(0, i * h / 8); ctx.lineTo(w, i * h / 8); ctx.stroke();
        }
        const scale = 6; // pixels per meter
        const cx = w / 2, cy = h / 2;
        // keep recent trail
        canvasPts.push({ x: state.x, y: state.y });
        if (canvasPts.length > 220) canvasPts.shift();
        // trail
        if (canvasPts.length > 1) {
            ctx.strokeStyle = "rgba(46,204,113,.35)";
            ctx.lineWidth = 2;
            ctx.beginPath();
            for (let i = 0; i < canvasPts.length; i++) {
                const px = cx + canvasPts[i].x * scale;
                const py = cy - canvasPts[i].y * scale;
                if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
            }
            ctx.stroke();
        }
        // center dot = current position
        const px = cx, py = cy;
        ctx.fillStyle = "#2ecc71";
        ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
        // heading arrow
        const a = headingToRad(state.headingDeg);
        ctx.strokeStyle = "#4f9fff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px + 18 * Math.cos(a), py - 18 * Math.sin(a));
        ctx.stroke();
    }

    /* ---- Event logging ---- */
    function pushEvent(html, cls) {
        if (!state.recentEvents) state.recentEvents = [];
        state.recentEvents.push({ html, cls, t: state.uptime });
        if (state.recentEvents.length > 200) state.recentEvents.shift();
    }
    function logLine(msg, cls) {
        const t = `[${fmtMs(state.uptime)}]`;
        return { html: `${t} ${msg}`, cls };
    }

    /* ---- Fault injection ---- */
    function injectFault(name) {
        if (state.activeFaults.has(name)) return;
        state.activeFaults.add(name);
        const labels = {
            GPS_LOSS: "GPS sensor loss (FaultId COMMLOSS=4)",
            IMU_LOSS: "IMU loss",
            WHEEL_LOSS: "Wheel encoder loss",
            COMMUNICATION_LOSS: "Communication loss (FaultId COMM_LOSS=4)",
            INVALID_COMMAND: "Invalid command (FaultId INVALID_COMMAND=5)"
        };
        pushEvent(`FAULT INJECTED: ${labels[name] || name}`, "fault");
    }
    function clearFaults() {
        const had = state.activeFaults.size;
        state.activeFaults.clear();
        state.activeFaults.delete("__INVALID_CMD_ACTIVE__");
        if (had) pushEvent("All injected faults cleared.", "ok");
    }

    /* ---- Main loop ---- */
    let lastTs = performance.now();
    function tick(now) {
        const deltaMs = Math.max(1, now - lastTs);
        lastTs = now;
        state.uptime += deltaMs;

        // periodic fault events for realism (only when no manual fault set)
        if (state.activeFaults.size === 0 && Math.floor(state.uptime / 15000) !== state.lastFaultEvent) {
            state.lastFaultEvent = Math.floor(state.uptime / 15000);
            if (rand() < 0.3) { injectFault("GPS_LOSS"); setTimeout(clearFaults, 4000); }
        }

        updateVehicle(deltaMs);
        evaluateSafety();
        updateControl();
        tickComm();
        // heartbeat-style periodic event
        if (Math.floor(state.uptime / 2000) !== Math.floor((state.uptime - deltaMs) / 2000)) {
            pushEvent(`HEARTBEAT id=0x${(0x0004).toString(16)} type=0x04 valid=1`, "ok");
        }

        render();
        renderFaultLog();
        requestAnimationFrame(tick);
    }

    /* ---- Init ---- */
    document.getElementById("clear-faults").addEventListener("click", clearFaults);
    document.querySelectorAll("[data-fault]").forEach(btn => {
        btn.addEventListener("click", () => injectFault(btn.getAttribute("data-fault")));
    });
    // seed-based jitter
    requestAnimationFrame(tick);
})();
