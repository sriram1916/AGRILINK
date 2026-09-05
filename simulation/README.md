# AgriLink Wokwi Simulation

This directory contains the local Wokwi simulation assets for the
AgriLink ESP32 Vehicle ECU prototype.

## Goal

Provide a working local Wokwi simulation that demonstrates:

- ESP32 boot
- FreeRTOS startup
- AgriLink system startup
- Periodic heartbeat
- System state logging (INIT -> READY)
- LED heartbeat on GPIO2
- Serial output via the Wokwi serial monitor

The simulation exercises the deterministic AgriLink firmware across
Phases 2–8: periodic heartbeat and INIT -> READY boot state transitions
(Phase 1–2), deterministic sensor snapshots (Phase 3), communication
loop-back (Phase 4), motion control with the kinematic vehicle simulation
(Phase 5), the safety supervisor (Phase 6), deterministic fault injection
(Phase 7), and the `[TEST]` observer suite (Phase 8).

## Files

- `diagram.json`   - Wokwi circuit: ESP32 DevKit, status LED on GPIO2,
                     push button on GPIO0.
- `wokwi.toml`     - Wokwi project config; loads the PlatformIO firmware
                    image `../.pio/build/esp32dev/firmware.bin` and exposes
                    `../.pio/build/esp32dev/firmware.elf` for debugging.

## Building the firmware

From the repository root:

```
pio run -e esp32dev
```

This produces:

- `.pio/build/esp32dev/firmware.elf`
- `.pio/build/esp32dev/firmware.bin`
- `.pio/build/esp32dev/bootloader.bin`
- `.pio/build/esp32dev/partitions.bin`

## Running the simulation

Open this folder in VS Code with the Wokwi extension installed.
Wokwi reads `wokwi.toml` and uses the diagram + ELF above.

Expected serial output (abridged):

```
[AGRILINK] System boot - Phase 1: Initializing hardware interfaces
[AGRILINK] Heartbeat system initialized, LED on GPIO 2
[AGRILINK] System boot - Phase 2: Core foundation initialized
[AGRILINK] Vehicle ECU state: INIT
[AGRILINK] System boot - Phase 3: All subsystem tasks scheduled
[AGRILINK] Vehicle ECU state: READY
[AGRILINK] All RTOS tasks have been created and scheduled
[AGRILINK] Heartbeat 0 - State: READY, LED: ON
[AGRILINK] Heartbeat 1 - State: READY, LED: OFF
...
```

The green LED on GPIO2 toggles every 2 s, providing a visible
heartbeat in the Wokwi UI.

## Honesty

This is a simulated prototype only. It does not exercise real
ISO 11783 / ISOBUS stacks, real agricultural sensors, or real
vehicle hardware.
