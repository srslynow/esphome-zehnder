# Introduction

The purpose of this repository is to wirelessly control a Zehnder Comfofan / Comfoair mechanical ventilation unit. The repository contains C++ firmware-code for an esp32-c6 module, the esp32 is paired with an NRF905 radio module for wireless communication.

# Details

- C++ code, we use ESPHome for basics like wifi, integration with Home Assistant, etc.
- Modern and solid C++ code
- Modifications of code should be done thoroughly: not just adding code. Refactor existing functions if needed to nicely integrate code there if that's makes sense.

# Files and directories

.
├── CLAUDE.md
├── LICENSE
├── README.md
├── components
│   └── zehnder_fan
│       ├── __init__.py
│       ├── fan.py
│       ├── zehnder_fan.cpp
│       └── zehnder_fan.h
└── zehnder_fan_controller.yaml
