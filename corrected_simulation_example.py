"""
Corrected Simulation Examples
==============================

This file contains corrected implementations of the key issues identified
in the performance analysis. These corrections address physical accuracy,
performance, and code quality problems.

Use these as reference for updating sim.ipynb.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Dict


# ============================================================================
# CORRECTED CONSTANTS
# ============================================================================

# Simulation parameters
TIME_STEPS = 8500  # Increased to complete full descent
DT = 0.1  # Time step duration in seconds
ALTITUDE_START = 40000  # Initial altitude in meters
TARGET_ALTITUDE = 0  # Target altitude

# Physical constants
AIR_DENSITY_SEA_LEVEL = 1.225  # kg/m³
SCALE_HEIGHT = 8500  # Atmospheric scale height in meters
GRAVITY = 9.81  # m/s²

# Vehicle parameters
VEHICLE_MASS = 25000  # kg (example: ~25 tons dry mass)
MOMENT_OF_INERTIA = np.array([50000, 50000, 10000])  # kg⋅m² [Ixx, Iyy, Izz]

# RCS parameters
RCS_THRUST_MAGNITUDE = 1000  # Newtons per thruster
RCS_MOMENT_ARM = 5.0  # meters (distance from CoM to thruster)
SPECIFIC_IMPULSE_RCS = 250  # seconds (typical for hypergolic propellants)

# Grid fin parameters
FIN_AREA = 2.0  # m² per fin (4 fins total)
TOTAL_FIN_AREA = 4 * FIN_AREA
FIN_MOMENT_ARM = 8.0  # meters from CoM
GRID_FIN_ANGLE_OF_ATTACK = 10  # degrees
LIFT_COEFFICIENT = 0.5  # Typical for grid fins
DRAG_COEFFICIENT = 1.2

# Control mode thresholds
RCS_TO_HYBRID_ALTITUDE = 30000  # meters
HYBRID_TO_GRIDFINS_ALTITUDE = 10000  # meters
HYSTERESIS_BAND = 500  # meters

# Perturbation parameters
PERTURBATION_INTERVAL = 100  # steps
PERTURBATION_MAGNITUDE = 0.05  # rad/s


# ============================================================================
# CORRECTED ATMOSPHERIC MODEL
# ============================================================================

def atmospheric_density(altitude: float) -> float:
    """
    Calculate air density using barometric formula (exponential decay).

    Correction: Previous version used linear scaling which overestimated
    density by up to 447% at 20km altitude.

    Args:
        altitude: Height above sea level in meters

    Returns:
        Air density in kg/m³
    """
    return AIR_DENSITY_SEA_LEVEL * np.exp(-altitude / SCALE_HEIGHT)


# ============================================================================
# CORRECTED 6-DOF STATE REPRESENTATION
# ============================================================================

class Vehicle6DOFState:
    """
    Complete 6-DOF state vector for reusable launch vehicle.

    Correction: Previous implementation only tracked altitude and angular
    velocity. This implements full translational and rotational dynamics.
    """

    def __init__(self):
        # Translational state (3 DOF)
        self.position = np.array([0.0, 0.0, ALTITUDE_START])  # [x, y, z] in meters
        self.velocity = np.array([0.0, 0.0, -50.0])  # [vx, vy, vz] in m/s

        # Rotational state (3 DOF)
        self.attitude = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # [ωx, ωy, ωz] in rad/s

        # Fuel state
        self.fuel_mass = 5000  # kg
        self.fuel_consumed = 0  # kg

    @property
    def altitude(self) -> float:
        """Current altitude above ground."""
        return self.position[2]

    @property
    def lateral_displacement(self) -> float:
        """Lateral deviation from vertical descent."""
        return np.linalg.norm(self.position[:2])

    def update_dynamics(self, forces: np.ndarray, torques: np.ndarray, dt: float):
        """
        Integrate 6-DOF equations of motion.

        Correction: Previous version had no proper dynamics integration.
        This implements Newton-Euler equations.

        Args:
            forces: 3D force vector [Fx, Fy, Fz] in Newtons
            torques: 3D torque vector [τx, τy, τz] in N⋅m
            dt: Time step in seconds
        """
        # Translational dynamics: F = ma
        total_mass = VEHICLE_MASS + self.fuel_mass
        acceleration = forces / total_mass

        # Add gravity
        acceleration[2] -= GRAVITY

        # Update velocity and position
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # Rotational dynamics: τ = I⋅α
        angular_acceleration = torques / MOMENT_OF_INERTIA
        self.angular_velocity += angular_acceleration * dt
        self.attitude += self.angular_velocity * dt

        # Wrap angles to [-π, π]
        self.attitude = np.arctan2(np.sin(self.attitude), np.cos(self.attitude))


# ============================================================================
# CORRECTED CONTROL MODE SELECTOR WITH HYSTERESIS
# ============================================================================

class ControlModeSelector:
    """
    Select control mode with hysteresis to prevent rapid switching.

    Correction: Previous implementation had hard thresholds causing
    potential control discontinuities at mode boundaries.
    """

    def __init__(self):
        self.current_mode = "RCS"
        self.hysteresis = HYSTERESIS_BAND

    def select(self, altitude: float) -> str:
        """
        Select control mode based on altitude with hysteresis.

        Args:
            altitude: Current altitude in meters

        Returns:
            Control mode string: "RCS", "Hybrid", or "Grid Fins"
        """
        if self.current_mode == "RCS":
            if altitude < RCS_TO_HYBRID_ALTITUDE - self.hysteresis:
                self.current_mode = "Hybrid"

        elif self.current_mode == "Hybrid":
            if altitude > RCS_TO_HYBRID_ALTITUDE + self.hysteresis:
                self.current_mode = "RCS"
            elif altitude < HYBRID_TO_GRIDFINS_ALTITUDE - self.hysteresis:
                self.current_mode = "Grid Fins"

        elif self.current_mode == "Grid Fins":
            if altitude > HYBRID_TO_GRIDFINS_ALTITUDE + self.hysteresis:
                self.current_mode = "Hybrid"

        return self.current_mode


# ============================================================================
# CORRECTED AERODYNAMIC FORCES
# ============================================================================

def calculate_aerodynamic_forces(
    velocity: np.ndarray,
    attitude: np.ndarray,
    air_density: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate lift and drag forces from grid fins.

    Correction: Previous version returned scalar lift/drag and incorrectly
    used them as torque components. This properly calculates forces and
    converts to torques.

    Args:
        velocity: 3D velocity vector [vx, vy, vz] in m/s
        attitude: 3D attitude vector [roll, pitch, yaw] in radians
        air_density: Air density in kg/m³

    Returns:
        Tuple of (aerodynamic_forces, aerodynamic_torques)
    """
    # Calculate relative wind velocity magnitude
    velocity_magnitude = np.linalg.norm(velocity)

    if velocity_magnitude < 1e-3:
        return np.zeros(3), np.zeros(3)

    # Dynamic pressure
    q = 0.5 * air_density * velocity_magnitude**2

    # Calculate lift and drag magnitudes
    # Note: Using pitch angle as angle of attack (simplified)
    alpha = attitude[1]  # pitch angle

    lift_magnitude = q * TOTAL_FIN_AREA * LIFT_COEFFICIENT * np.sin(alpha)
    drag_magnitude = q * TOTAL_FIN_AREA * DRAG_COEFFICIENT

    # Convert to force vectors (in body frame)
    # Lift perpendicular to velocity, drag opposite to velocity
    velocity_direction = velocity / velocity_magnitude

    # Simplified: drag opposes motion, lift acts perpendicular
    drag_force = -drag_magnitude * velocity_direction

    # Lift in vertical plane (simplified)
    lift_force = np.array([0, 0, lift_magnitude])

    total_force = drag_force + lift_force

    # Calculate torques: τ = r × F
    # Grid fins act at moment arm from center of mass
    moment_arm_vector = np.array([0, 0, -FIN_MOMENT_ARM])  # Fins behind CoM
    torque = np.cross(moment_arm_vector, total_force)

    return total_force, torque


# ============================================================================
# CORRECTED RCS TORQUE CALCULATION
# ============================================================================

def calculate_rcs_torque(
    control_demand: np.ndarray,
    state: Vehicle6DOFState
) -> Tuple[np.ndarray, float]:
    """
    Calculate torque from RCS thrusters and fuel consumption.

    Correction: Previous version had hardcoded thruster positions and
    didn't properly track fuel consumption.

    Args:
        control_demand: Desired torque in each axis [τx, τy, τz]
        state: Current vehicle state

    Returns:
        Tuple of (actual_torque, fuel_consumed_kg)
    """
    # Thruster configuration: 4 thrusters in cross pattern
    thruster_positions = np.array([
        [RCS_MOMENT_ARM, 0, 0],   # +X thruster
        [-RCS_MOMENT_ARM, 0, 0],  # -X thruster
        [0, RCS_MOMENT_ARM, 0],   # +Y thruster
        [0, -RCS_MOMENT_ARM, 0]   # -Y thruster
    ])

    # Simplified thruster allocation (in reality, this is an optimization problem)
    # Each thruster fires in Z direction
    thrust_direction = np.array([0, 0, 1])

    # Calculate required thrust for each thruster to achieve desired torque
    # For simplicity, use paired thrusters
    torque = np.zeros(3)
    total_thrust = 0

    # Roll control: +X and -X thrusters
    if abs(control_demand[0]) > 1e-6:
        thrust_magnitude = min(abs(control_demand[0]) / RCS_MOMENT_ARM, RCS_THRUST_MAGNITUDE)
        sign = np.sign(control_demand[0])
        torque[0] = sign * thrust_magnitude * RCS_MOMENT_ARM
        total_thrust += 2 * thrust_magnitude  # Two thrusters firing

    # Pitch control: +Y and -Y thrusters
    if abs(control_demand[1]) > 1e-6:
        thrust_magnitude = min(abs(control_demand[1]) / RCS_MOMENT_ARM, RCS_THRUST_MAGNITUDE)
        sign = np.sign(control_demand[1])
        torque[1] = sign * thrust_magnitude * RCS_MOMENT_ARM
        total_thrust += 2 * thrust_magnitude

    # Calculate fuel consumption using rocket equation
    # Δm = (F⋅Δt) / (g₀⋅Isp)
    fuel_consumed = (total_thrust * DT) / (GRAVITY * SPECIFIC_IMPULSE_RCS)

    return torque, fuel_consumed


# ============================================================================
# CORRECTED PID CONTROLLER
# ============================================================================

class AttitudePIDController:
    """
    PID controller for attitude control (not altitude!).

    Correction: Previous version controlled altitude with torque, which
    is physically incorrect. Torque controls rotation, not translation.
    """

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)

    def compute(self, target_attitude: np.ndarray, current_attitude: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute control torque to achieve target attitude.

        Args:
            target_attitude: Desired [roll, pitch, yaw] in radians
            current_attitude: Current [roll, pitch, yaw] in radians
            dt: Time step in seconds

        Returns:
            Control torque vector [τx, τy, τz] in N⋅m
        """
        # Calculate attitude error (wrap to [-π, π])
        error = target_attitude - current_attitude
        error = np.arctan2(np.sin(error), np.cos(error))

        # PID terms
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        # Compute control output
        control = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        return control


# ============================================================================
# CORRECTED HYBRID CONTROL
# ============================================================================

def calculate_hybrid_control(
    rcs_torque: np.ndarray,
    aero_torque: np.ndarray,
    altitude: float
) -> np.ndarray:
    """
    Blend RCS and aerodynamic torques based on altitude.

    Correction: Previous implementation was correct, but this adds
    documentation and bounds checking.

    Args:
        rcs_torque: Torque from RCS thrusters [N⋅m]
        aero_torque: Torque from aerodynamic surfaces [N⋅m]
        altitude: Current altitude [m]

    Returns:
        Combined torque vector [N⋅m]
    """
    # Weight RCS linearly from 1.0 at 30km to 0.0 at 10km
    weight_rcs = np.clip(
        (altitude - HYBRID_TO_GRIDFINS_ALTITUDE) /
        (RCS_TO_HYBRID_ALTITUDE - HYBRID_TO_GRIDFINS_ALTITUDE),
        0.0, 1.0
    )
    weight_aero = 1.0 - weight_rcs

    return weight_rcs * rcs_torque + weight_aero * aero_torque


# ============================================================================
# OPTIMIZED DATA STRUCTURES
# ============================================================================

class SimulationHistory:
    """
    Pre-allocated arrays for simulation history.

    Correction: Previous version used Python lists which are inefficient
    for numerical data. This uses NumPy arrays with pre-allocation.
    """

    def __init__(self, time_steps: int):
        self.time_steps = time_steps
        self.current_step = 0

        # Pre-allocate all arrays
        self.time = np.zeros(time_steps)
        self.altitude = np.zeros(time_steps)
        self.lateral_displacement = np.zeros(time_steps)
        self.velocity = np.zeros((time_steps, 3))
        self.attitude = np.zeros((time_steps, 3))
        self.angular_velocity = np.zeros((time_steps, 3))
        self.fuel_consumed = np.zeros(time_steps)
        self.torque_magnitude = np.zeros(time_steps)
        self.control_mode = np.empty(time_steps, dtype='U10')  # String array

    def record(self, state: Vehicle6DOFState, torque: np.ndarray, mode: str):
        """Record current state to history."""
        if self.current_step >= self.time_steps:
            return

        i = self.current_step
        self.time[i] = i * DT
        self.altitude[i] = state.altitude
        self.lateral_displacement[i] = state.lateral_displacement
        self.velocity[i] = state.velocity
        self.attitude[i] = state.attitude
        self.angular_velocity[i] = state.angular_velocity
        self.fuel_consumed[i] = state.fuel_consumed
        self.torque_magnitude[i] = np.linalg.norm(torque)
        self.control_mode[i] = mode

        self.current_step += 1

    def trim(self):
        """Remove unused pre-allocated space."""
        n = self.current_step
        self.time = self.time[:n]
        self.altitude = self.altitude[:n]
        self.lateral_displacement = self.lateral_displacement[:n]
        self.velocity = self.velocity[:n]
        self.attitude = self.attitude[:n]
        self.angular_velocity = self.angular_velocity[:n]
        self.fuel_consumed = self.fuel_consumed[:n]
        self.torque_magnitude = self.torque_magnitude[:n]
        self.control_mode = self.control_mode[:n]


# ============================================================================
# CORRECTED MAIN SIMULATION
# ============================================================================

def run_corrected_simulation() -> SimulationHistory:
    """
    Run the corrected simulation with all fixes applied.

    Returns:
        SimulationHistory object containing all recorded data
    """
    # Initialize state and controllers
    state = Vehicle6DOFState()
    mode_selector = ControlModeSelector()
    attitude_controller = AttitudePIDController(kp=1000, ki=100, kd=500)
    history = SimulationHistory(TIME_STEPS)

    # Target attitude (vertical descent)
    target_attitude = np.array([0.0, 0.0, 0.0])

    # Main simulation loop
    for step in range(TIME_STEPS):
        # Select control mode
        mode = mode_selector.select(state.altitude)

        # Calculate control demand from PID
        control_demand = attitude_controller.compute(
            target_attitude, state.attitude, DT
        )

        # Calculate atmospheric density
        rho = atmospheric_density(state.altitude)

        # Calculate forces and torques based on control mode
        if mode == "RCS":
            # RCS only
            torque, fuel_consumed = calculate_rcs_torque(control_demand, state)
            forces = np.zeros(3)
            state.fuel_consumed += fuel_consumed
            state.fuel_mass -= fuel_consumed

        elif mode == "Grid Fins":
            # Aerodynamic only
            forces, torque = calculate_aerodynamic_forces(
                state.velocity, state.attitude, rho
            )

        else:  # Hybrid
            # Both RCS and aerodynamics
            rcs_torque, fuel_consumed = calculate_rcs_torque(control_demand, state)
            aero_forces, aero_torque = calculate_aerodynamic_forces(
                state.velocity, state.attitude, rho
            )
            torque = calculate_hybrid_control(rcs_torque, aero_torque, state.altitude)
            forces = aero_forces
            state.fuel_consumed += fuel_consumed
            state.fuel_mass -= fuel_consumed

        # Apply random perturbations periodically
        if step % PERTURBATION_INTERVAL == 0:
            perturbation = PERTURBATION_MAGNITUDE * np.random.randn(3)
            state.angular_velocity += perturbation

        # Update dynamics
        state.update_dynamics(forces, torque, DT)

        # Record state
        history.record(state, torque, mode)

        # Check termination condition
        if state.altitude <= 0:
            print(f"Simulation completed at step {step}")
            print(f"Final altitude: {state.altitude:.2f} m")
            print(f"Lateral displacement: {state.lateral_displacement:.2f} m")
            print(f"Total fuel consumed: {state.fuel_consumed:.2f} kg")
            break
    else:
        print(f"Warning: Simulation did not reach ground in {TIME_STEPS} steps")
        print(f"Final altitude: {state.altitude:.2f} m")

    # Trim unused pre-allocated space
    history.trim()

    return history


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    print("Running corrected simulation...")
    print("=" * 60)

    # Run simulation
    history = run_corrected_simulation()

    print("\n" + "=" * 60)
    print("Simulation statistics:")
    print(f"Duration: {history.time[-1]:.1f} seconds")
    print(f"Steps executed: {history.current_step}")
    print(f"Final lateral deviation: {history.lateral_displacement[-1]:.2f} m")
    print(f"Total fuel used: {history.fuel_consumed[-1]:.2f} kg")

    # Count control mode usage
    unique_modes, counts = np.unique(history.control_mode, return_counts=True)
    print("\nControl mode usage:")
    for mode, count in zip(unique_modes, counts):
        percentage = 100 * count / history.current_step
        print(f"  {mode}: {count} steps ({percentage:.1f}%)")

    # Create visualization
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Plot 1: Lateral deviation vs altitude
    axes[0].plot(history.altitude, history.lateral_displacement, 'b-', linewidth=1)
    axes[0].set_xlabel('Altitude (m)')
    axes[0].set_ylabel('Lateral Displacement (m)')
    axes[0].set_title('Lateral Deviation vs Altitude')
    axes[0].grid(True)
    axes[0].invert_xaxis()

    # Plot 2: Fuel consumption vs altitude
    axes[1].plot(history.altitude, history.fuel_consumed, 'r-', linewidth=1)
    axes[1].set_xlabel('Altitude (m)')
    axes[1].set_ylabel('Fuel Consumed (kg)')
    axes[1].set_title('Fuel Consumption vs Altitude')
    axes[1].grid(True)
    axes[1].invert_xaxis()

    # Plot 3: Torque magnitude vs altitude
    axes[2].plot(history.altitude, history.torque_magnitude, 'g-', linewidth=1)
    axes[2].set_xlabel('Altitude (m)')
    axes[2].set_ylabel('Torque Magnitude (N⋅m)')
    axes[2].set_title('Control Torque vs Altitude')
    axes[2].grid(True)
    axes[2].invert_xaxis()

    plt.tight_layout()
    plt.savefig('corrected_simulation_results.png', dpi=150)
    print("\nPlot saved as 'corrected_simulation_results.png'")
