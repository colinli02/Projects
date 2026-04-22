# Controls Projects

## Self Balancing 2 Wheel Driveable Bluetooth Rrobot (2025 May)

- Built a self‑balancing, rideable two‑wheel robot using an Arduino Nano 33 BLE Sense
- Implemented a PID‑controlled stabilization system for real‑time balance correction
- Designed firmware with easily tunable PID parameters for rapid iteration and optimization
- Added Bluetooth control for remote driving and parameter adjustment

??? "Click to show code"
    ``` c title="PIDRobot.ino"
    --8<-- "https://raw.githubusercontent.com/colinli02/Projects/refs/heads/main/Project_Repos/PIDRobot/PIDRobot.ino"
    ```

## Simulink Control System for SpO₂ Regulation (2026 April)

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
