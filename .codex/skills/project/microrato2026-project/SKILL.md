---
name: microrato2026-project
description: Use when working in the rato workspace on MicroRato2026 robot code, simulator integration, or Python agent structure. Treat ciberRatoTools as read-only reference material and do not modify files under ciberRatoTools.
---

# MicroRato2026 Project

Use this skill when the task is about the `rato` workspace, especially `MicroRato2026` Python robot code.

## Scope

- Put new robot code, scripts, docs, and tests under `MicroRato2026/`.
- Treat `ciberRatoTools/` as upstream tooling and reference code.
- Do not edit, rename, delete, or reformat anything under `ciberRatoTools/`.

## Working Rules

- Read `ciberRatoTools/pClient/mainRob.py` and `ciberRatoTools/pClient/croblink.py` first when building a Python robot.
- Reuse the simulator protocol and control loop shape from those files, but implement your own robot inside `MicroRato2026/`.
- If functionality from `ciberRatoTools/` is needed in your robot, prefer one of these approaches:
  1. Import it without modifying `ciberRatoTools/`.
  2. Copy the minimum required client code into `MicroRato2026/` and maintain it there.
- Keep launchers, configuration, and robot-specific assets in `MicroRato2026/`.

## Python Robot Minimum

A Python robot in `MicroRato2026/` should usually include:

- An entry script that parses CLI args like robot name, host, position, and optional map path.
- A robot class based on `CRobLinkAngs` or an equivalent local implementation of the same UDP/XML protocol.
- A `run()` loop that:
  - connects to the simulator,
  - calls `readSensors()`,
  - reacts to `start`, `stop`, and `endLed`,
  - sends actuator commands with `driveMotors()`,
  - handles visiting/returning LEDs if the challenge logic needs them.
- A navigation or behavior method such as `wander()` or a more advanced planner.
- Optional map parsing utilities if using the simulator lab XML files.

## Safe Default Layout

```text
MicroRato2026/
├── README.md
├── robot/
│   ├── __init__.py
│   ├── main.py
│   ├── agent.py
│   ├── protocol.py
│   └── map_parser.py
├── scripts/
│   └── run_robot.sh
└── tests/
```

## Decision Boundary

- If a requested change would touch `ciberRatoTools/`, stop and propose the equivalent change inside `MicroRato2026/` instead.
- Only inspect `ciberRatoTools/` to understand the simulator API, sample behavior, and launch conventions.
