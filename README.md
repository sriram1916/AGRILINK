# AgriLink

AgriLink is a prototype embedded control and communication platform for an autonomous agricultural vehicle. The project models the software architecture of an ESP32-based Vehicle Control ECU and is intentionally designed as a simulation-facing prototype for research, engineering exploration, and early-stage system design.

This repository is not a production autonomous tractor control system, not a certified safety-critical implementation, and not a complete ISO 11783/ISOBUS stack. It is a simulation-oriented platform intended to explore real-time control, fault tolerance, communications, and subsystem orchestration in a farm-vehicle context.

## Project intent

The ESP32 DevKit acts as the Vehicle Control ECU for a simulated agricultural machine. The system is organized around a FreeRTOS-based architecture with multiple embedded tasks for sensors, localization, navigation, motion control, safety supervision, communication, diagnostics, and vehicle simulation.

The early architecture emphasizes:

- deterministic real-time task separation
- explicit safety override paths
- modular subsystem boundaries
- fault-tolerant control concepts
- prototype communication between a farm control center and the vehicle ECU
- ISOBUS-inspired communication patterns without claiming a full implementation

## Technical direction

- MCU: ESP32 DevKit
- Language: C/C++
- Framework: ESP-IDF via PlatformIO
- RTOS: FreeRTOS
- Simulation: Wokwi
- Development environment: VS Code
- Version control: Git
- Repository: GitHub

## System architecture status

This repository currently contains the project foundation and architecture scaffold only. Subsystem functionality is intentionally stubbed and not yet implemented as production logic.

The current tasks are limited to:

- project initialization
- ESP-IDF build configuration
- RTOS task layout definition
- placeholder module interfaces
- architecture documentation placeholders
- simulation configuration placeholders

## Major planned subsystems

- sensor subsystem
- localization
- navigation
- motion control
- safety supervisor
- communication
- diagnostics
- mission management
- vehicle/actuator simulation
- fault injection
- automated testing

## Safety principle

Safety decisions are expected to override navigation and normal control commands when a hazard, fault, or invalid operating condition is detected.

## Vehicle states

- INIT
- READY
- AUTONOMOUS
- DEGRADED
- FAULT
- SAFE_STOP
- EMERGENCY_STOP
- RECOVERY

## Planned fault scenarios

- GPS loss
- communication loss
- Wi-Fi loss
- obstacle detection
- sensor disagreement
- actuator mismatch
- watchdog/task failure
- invalid/unauthorized command

## Repository layout

```text
AgriLink/
├── README.md
├── CMakeLists.txt
├── platformio.ini
├── sdkconfig.esp32dev
├── .gitignore
├── include/
│   └── agri_types.h
├── src/
│   ├── CMakeLists.txt
│   └── main.cpp
├── firmware/
│   ├── heartbeat/
│   ├── sensors/
│   ├── communication/
│   ├── control/
│   ├── simulation/
│   ├── safety/
│   ├── mission/
│   ├── navigation/
│   ├── diagnostics/
│   ├── fault_injection/
│   ├── tests/
│   └── rtos/
├── scripts/
├── simulation/
├── docs/
├── architecture/
└── .vscode/
```

## Build status

This branch is intended to compile as the initial platform foundation. Complete subsystem behavior is not yet implemented.

## Wokwi readiness

The simulation directory includes placeholder Wokwi files so that later schematic and device configuration can be added without changing the core project layout.

## Disclaimer

AgriLink is an educational prototype and simulation-oriented embedded control architecture. It is not a deployment-ready autonomous tractor control platform and should not be treated as a safety-certified system for real-world agricultural machinery.
