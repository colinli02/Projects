# Controls Projects

## SCADALite: A Lightweight Python based SCADA Dashboard (2026 April - 2026 May)

[![GitHub Repo](https://img.shields.io/badge/GitHub-SCADALite-blue?logo=github)](https://github.com/colinli02/SCADALite)

- Developed a lightweight Python-based SCADA dashboard for real-time telemetry visualization, alarm monitoring, serial/COM device integration, and historical data trending.

![SCADALite Dashboard](https://raw.githubusercontent.com/colinli02/SCADALite/main/Capture.PNG)

## Self Balancing 2 Wheel Driveable Bluetooth Robot (2025 April - 2025 May)

- Built a self-balancing, rideable two-wheel robot powered by an Arduino Nano 33 BLE Sense

- Implemented a real-time PID stabilization system for dynamic balance correction

- Developed firmware with runtime-tunable PID parameters for rapid iteration and optimization using Arduino CLI

- Added Bluetooth-based remote control and PID adjustment through a Flutter mobile app

- Implemented lightweight SCADA style real-time monitoring tool built in Python.

??? "Click to show Arduino code"
    ``` c title="PIDRobot.ino"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/refs/heads/main/Project_Repos/PIDRobot/PIDRobot.ino"
    ```

??? "Click to show Python code"
    ``` py linenums="1" title="display.py"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/refs/heads/main/Project_Repos/PIDRobot/display.py"

    ```

## Simulink Control System for SpO₂ Regulation (2026 March - 2026 April)

- Designed a PI-based control system in MATLAB/Simulink to regulate blood oxygen saturation (SpO₂) via FiO₂ under delay, nonlinearity, and disturbances (HR, RR).  
- Implemented feedforward compensation, anti-windup, dead zone, and filtering to improve stability and robustness.  
- Maintained SpO₂ within 90–94% and respected FiO₂ safety constraints during disturbances and transients.  
- Validated performance across varying plant parameters, demonstrating strong robustness to patient variability.

??? "Click for more details"
    **Control System**
    - PID controller (P, I, D tuned experimentally)
    - Anti-windup and output saturation handling
    - Stable real-time loop timing

    **Hardware / Integration**
    - Sensor input processing (filtered to reduce noise)
    - PWM motor control from PID output
    - Embedded C/C++ implementation on microcontroller

    **Performance**
    - Reduced oscillations and steady-state error
    - Fast settling time with stable tracking
    - Robust under varying conditions
