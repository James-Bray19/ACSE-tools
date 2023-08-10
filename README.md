# ACSE Tools - Control Loop Diagram and Analysis

This MATLAB code snippet provides a control loop diagram and analysis for a given control system. The system is depicted in a control loop diagram, and key performance metrics and behaviors are analyzed and presented graphically. The code utilizes MATLAB's Control System Toolbox to analyze and visualize the control system's behavior.

## Control Loop Diagram

The control loop diagram consists of the following components:

![image](https://github.com/James-Bray19/ACSE-tools/assets/47334864/cbe20517-4a04-4507-ae2b-d0f4ab23d049)

- **R**: Desired output setpoint.
- **E**: Error signal (difference between desired output and actual output).
- **U**: Plant input (control signal to the plant).
- **Y**: Actual output.

## System Inputs

The following inputs are provided to the control system:

- **Desired Output (R)**: The desired output setpoint.
- **Controllers**: Proportional (Kp), Integral (Ki), and Derivative (Kd) controller gains.
- **Plant Function (G)**: Transfer function representing the plant dynamics.

## Analysis and Plots

The code generates three plots to analyze the control system's behaviour:

1. **Error (E) Analysis**:
   - Step response of the error signal (E).
   - Pole-zero map of the error transfer function.
   - Bode plot of the error transfer function.

2. **Plant Input (U) Analysis**:
   - Step response of the plant input (U).
   - Pole-zero map of the plant input transfer function.
   - Bode plot of the plant input transfer function.

3. **Output (Y) Analysis**:
   - Step response of the output (Y).
   - Pole-zero map of the output transfer function.
   - Bode plot of the output transfer function.

For each analysis, the following key behaviours and metrics are computed and displayed in a table:

- Steady-state gain.
- Steady-state gain with respect to the reference (R).
- Percent overshoot.
- Rise time.
- Settling time.
- Peak time.
- Time constant (τ).
- Damping ratio (ζ).
- Natural frequency (ωn).
- Damped natural frequency (ωd).
- Resonant frequency (wr).
- Cutoff frequency (wc).

## Usage

1. Set the desired output (R), disturbance (V), and controller gains (Ki, Kp, Kd).
2. Define the plant transfer function (G).
3. Run the code in a MATLAB environment.
