# Circuit Projects

## Multivibrator PCB Design with Altium

## LTSpice Simulation (2024 Jan - Apr)

CMOS / Transistor Simulations, AC & DC Small‑Signal Analysis, and Evaluation of Transient Behavior

This section documents my work from ELEC 301 Mini Project 1, where I designed, analyzed, and simulated multi‑pole circuits using LTSpice. The project involved reconstructing transfer functions, validating OC/SC time‑constant approximations, and comparing analytical results with LTSpice simulations.

### Project 1

3‑pole system  
Designed RC network to match a given 3‑pole transfer function.
Used OC/SC method to find capacitor values and verified with LTSpice AC + Laplace block.

4‑pole system  
Ran AC sweep on provided circuit to identify low‑ and high‑frequency poles.
Varied C3 to observe how the low‑frequency pole shifts.
Compared simulated results with OC/SC approximations.

Transconductance amplifier  
Found low‑frequency poles from large capacitors.
Used Miller effect to estimate high‑frequency poles.
Verified midband gain and pole locations with LTSpice.

### Project 2

Part 1 — 2N2222A Transistor

    Extracted small‑signal parameters from datasheet (hfe, rπ, ro).

    Simulated IB–VBE and IC–VCE curves in LTSpice.

    Calculated gm, β, rπ, and Early voltage from plots.

    Verified all calculated values fall within datasheet ranges.

    Designed bias network using measured parameters (no 1/3 rule).

    Repeated bias design using 1/3 rule and with standard resistor values.

    Compared DC operating points — all methods give similar results.

    Compared 2N2222A, 2N4401, 2N3904 — small differences in bias currents and voltages.

Part 2 — CE Amplifier

    Built CE amplifier using 1/3‑rule bias network.

    Calculated low‑frequency and high‑frequency poles (LP/HP).

    Measured Bode plot in LTSpice and compared with calculations.

    Repeated full analysis for both 2N3904 and 2N4401.

    Found midband linear range by sweeping input amplitude.

    Measured input impedance at midband (~20 kHz).

    Measured output impedance at midband.

    2N3904 showed better bandwidth and closer match to calculations.

Part 3 — 2N2222A CB Amplifier

    Built common‑base amplifier and repeated pole calculations.

    Measured Bode plot and compared with predicted LP/HP values.

    Measured input impedance (very low, as expected for CB).

    Measured output impedance (high, dominated by RC + RL).

### Project 3

Part 1 Cascode Amplifier (2N3904)

    Biased cascode using given voltage splits (¼, ½, ¾ of VCC).

    Calculated RB network and RE using MP2 transistor parameters.

    Solved for coupling/bypass capacitors using given low‑frequency spec.

    Verified DC operating point in LTSpice.

    Measured low‑ and high‑frequency 3 dB points → close to calculated values.

    Found midband gain and linear range (non‑linear above ~0.15 V).

    Measured input/output impedance at midband.

Part 2 Cascaded Amplifiers (2N2222A)

    Designed CB → CC cascade using 1/3‑rule biasing.

    Calculated RB, RC, RE, and coupling capacitors for 1 kHz low‑frequency cutoff.

    Standardized resistor/capacitor values and built circuit in LTSpice.

    Measured midband input/output impedance → adjusted RE to meet spec.

    Measured midband gain and 3 dB frequencies.

    Verified design meets required Rin, Rout, and bandwidth.

Part 3 Differential Operational Amplifier (2N3904)

    Built differential pair with symmetric AC inputs.

    Measured differential gain (~80 dB at 1 kHz).

    Measured differential input impedance (tens of MΩ) and output impedance (~kΩ).

    Applied small resistor mismatch to test common‑mode gain.

    Calculated CMRR from differential vs common‑mode response.

    Built non‑inverting amplifier configuration and measured gain + linearity.

    Measured Rin and Rout at 1 kHz.

Part 4 AM Modulator (CM2N2222A)

    Applied small‑signal sine input and observed differential output.

    Swept input amplitude (10–100 mVp) → distortion decreases around ~70 mVp.

    Tested square‑wave modulation → higher amplitude reduces distortion.

    Observed AM behavior: low‑frequency input modulates high‑frequency carrier.

    Noted importance of choosing proper input amplitude to minimize distortion.

    Noted importance of choosing proper input amplitude to minimize distortion.

### Project 4

Active filter

    Designed a second‑order Butterworth low‑pass filter.

    Selected R and C values to meet a 10 kHz cutoff.

    Increased gain to observe onset of oscillation and pole movement.

Phase‑shift oscillator

    Built a three‑stage RC phase‑shift oscillator.

    Adjusted resistor values to achieve sustained oscillation.

    Measured oscillation frequency and verified scaling with R and C.

Feedback amplifier

    Performed DC biasing and extracted small‑signal parameters.

    Measured open‑loop and closed‑loop frequency response.

    Evaluated input and output impedance under feedback.

    Calculated feedback factor and desensitivity, confirming expected behavior.
