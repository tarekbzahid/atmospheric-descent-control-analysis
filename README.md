# Hybrid RCS & Aerodynamic Control Simulation for Reusable Launch Vehicles

This repository provides a simulation framework for analyzing and comparing different control strategies—Reaction Control System (RCS)-only vs. Hybrid (RCS + Aerodynamic control surfaces)—for the descent phase of reusable launch vehicles (RLVs). The code models six-degrees-of-freedom (6-DOF) dynamics and evaluates performance metrics such as fuel consumption, torque dynamics, and lateral deviation. By offering a testbed for studying altitude-based transitions from vacuum to denser atmospheric conditions, this environment supports research into more efficient, robust, and adaptive control algorithms.

## Key Features
- **6-DOF Dynamics**: Accurate modeling of rocket position, velocity, attitude, and angular rates under varying atmospheric conditions.
- **Multiple Control Modes**: 
  - **RCS Mode**: Ideal at high altitudes with low-density atmosphere, relying solely on thrusters.
  - **Hybrid Mode**: Blends aerodynamic surfaces (e.g., grid fins) with RCS thrusters for enhanced efficiency and stability at transitional altitudes.
- **PID Control**: A tunable PID controller adjusts control efforts dynamically to correct trajectory deviations.
- **Metrics Tracking**: The simulation logs fuel usage, torque, and lateral deviation for comparative analysis.
- **Open-Source and Extensible**: Designed in Python with open-source libraries, enabling easy modifications, integrations, and inclusion of advanced techniques like machine learning-based optimization.

## Requirements
- **Python 3.8+** recommended
- **Dependencies**:
  - `numpy`
  - `scipy`
  - `matplotlib`
  - (Optional) Additional control or aerospace-focused libraries if needed
