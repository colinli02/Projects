# Circuit Projects

## Multivibrator PCB Design with Altium

Designed and simulated a two-transistor audio-frequency oscillator circuit for using Altium Designer and SPICE simulation. 

Calculated and tested resistor and capacitor values to produce output frequencies within the human hearing range, verified through transient waveform analysis at the transistor collectors. 

Modified the schematic to include test pins for oscilloscope probing, performed design rule checks (DRC) to ensure manufacturability, and developed a hardware test plan using multimeters and oscilloscopes to validate fabrication quality, signal generation, and circuit stability. 

## LTSpice Simulation (2024 Jan - Apr)

CMOS / Transistor Simulations, AC & DC Small-Signal Analysis, and Evaluation of Transient Behavior

This section documents my work from where I designed, analyzed, and simulated multi-pole circuits using LTSpice.


### Project 1 — Analog Frequency Response Analysis
Designed and analyzed multi-pole RC and transistor amplifier circuits using LTSpice, including pole estimation, frequency response analysis, and simulation verification.

??? "Click to show project details"

    #### 3-pole system
    - Designed RC network to match a given 3-pole transfer function
    - Used OC/SC method to determine capacitor values
    - Verified response using LTSpice AC + Laplace simulations

    #### 4-pole system
    - Performed AC sweep analysis to identify poles
    - Varied capacitor values to observe pole movement
    - Compared simulated results with analytical approximations

    #### Transconductance amplifier
    - Calculated low/high-frequency poles using large capacitor and Miller approximations
    - Verified midband gain and pole locations in LTSpice

---

### Project 2 — BJT Characterization and Amplifier Design
Characterized BJTs and designed CE/CB amplifiers using LTSpice, including gain, bandwidth, impedance, and bias analysis.

??? "Click to show project details"

    #### Part 1 — 2N2222A Transistor
    - Extracted small-signal parameters from datasheets and simulation
    - Simulated transistor IV curves and calculated gm, β, rπ, and Early voltage
    - Designed and compared multiple biasing methods
    - Compared behavior of 2N2222A, 2N3904, and 2N4401 transistors

    #### Part 2 — CE Amplifier
    - Built common-emitter amplifiers using 1/3-rule biasing
    - Calculated and measured low/high-frequency poles
    - Measured gain, bandwidth, linear range, and input/output impedance
    - Compared performance between transistor models

    #### Part 3 — CB Amplifier
    - Built and analyzed a common-base amplifier
    - Verified predicted pole locations and impedance behavior
    - Measured Bode response and amplifier characteristics

---

### Project 3 — Advanced Amplifier Systems
Designed and simulated advanced transistor amplifier systems including cascode, differential, cascaded, and AM modulation circuits.

??? "Click to show project details"

    #### Part 1 — Cascode Amplifier
    - Designed cascode amplifier using specified voltage bias splits
    - Calculated coupling and bypass capacitors
    - Measured gain, bandwidth, linearity, and impedance

    #### Part 2 — Cascaded Amplifiers
    - Designed CB → CC cascaded amplifier system
    - Calculated resistor/capacitor values for bandwidth requirements
    - Verified gain and impedance specifications in LTSpice

    #### Part 3 — Differential Operational Amplifier
    - Built differential pair amplifier with symmetric inputs
    - Measured differential gain, CMRR, and impedance
    - Tested non-inverting amplifier configuration and linearity

    #### Part 4 — AM Modulator
    - Built transistor AM modulation circuit
    - Observed carrier modulation and distortion effects
    - Evaluated impact of input amplitude on signal quality

---

### Project 4 — Filters, Oscillators, and Feedback
Designed active filters, oscillators, and feedback amplifiers while analyzing gain, stability, impedance, and oscillation behavior.

??? "Click to show project details"

    #### Active filter
    - Designed second-order Butterworth low-pass filter
    - Selected component values for 10 kHz cutoff
    - Observed oscillation onset with increased gain

    #### Phase-shift oscillator
    - Built three-stage RC oscillator
    - Tuned resistor values for sustained oscillation
    - Verified oscillation frequency scaling with R and C

    #### Feedback amplifier
    - Performed DC biasing and small-signal analysis
    - Measured open-loop and closed-loop responses
    - Evaluated feedback factor, impedance, and desensitivity
