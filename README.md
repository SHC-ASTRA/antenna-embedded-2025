# rover-embedded-2025 (rover-Basestation-Antenna)

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

## Specs

- MCU: DOIT ESP32 Devkit V1
- Sensors:
    - GPS: Sparkfun NEO-M9N
    - IMU: Adafruit BNO-055
- Actuators:
    - Lynxmotion Smart Servo HT-1
- Communication:
    - Adafruit Ethernet Featherwing
    - UDP Protocol to/from Basestation

## Description

This Platform.io project is targeted for ASTRA's tracking antenna used at URC 2025. The DOIT ESP32 Devkit V1
controlled a Lynxmotion Smart Servo to rotate a turret that held both antenna dishes to point at the rover,
using sensor data from the on-PCB GPS and IMU. The ESP32 would receive information on the location
of the rover via basestation over raw UDP packets to the ethernet adapter. The ESP32 would send feedback
back to basestation using string JSON also via raw UDP packets.

The demise of this tracking antenna came from the load-bearing lid that softened and warped in the Utah
sun, moving the gears out of alignment and making the tracking antenna stationary.

## Primary Issues

- Mechanical
- The BNO-055 calibration was time-consuming, inconvenient, and unreliable.
- The LSS would not report its absolute position, making it difficult to constrain its movement in software.
- The raw UDP strings were janky and made it so communication relied on basestation running
- Progress was delayed, making proper integration and testing with basestation difficult
    - Tracking antenna was first functionally tested while packing for Utah...
