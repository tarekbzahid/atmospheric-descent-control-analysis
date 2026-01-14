# System Diagrams and Architecture

## 1. Complete System Architecture

```mermaid
graph TB
    subgraph "Initialization"
        A[Start Simulation] --> B[Define Constants]
        B --> C[Initialize State Vector]
        C --> D[Pre-allocate Arrays]
    end

    subgraph "Main Loop"
        D --> E{Altitude > 0?}
        E -->|Yes| F[Measure Atmospheric Density]
        F --> G[Select Control Mode]

        G --> H{Mode?}
        H -->|RCS| I[RCS Controller]
        H -->|Hybrid| J[Hybrid Controller]
        H -->|Grid Fins| K[Aerodynamic Controller]

        I --> L[Calculate RCS Torque]
        J --> M[Blend RCS + Aero Torque]
        K --> N[Calculate Aero Torque]

        L --> O[Compute Fuel Consumption]
        M --> O
        N --> P[Zero Fuel Consumption]
        O --> Q[Apply Perturbations]
        P --> Q

        Q --> R[Integrate Rotational Dynamics]
        R --> S[Integrate Translational Dynamics]
        S --> T[Update State Vector]
        T --> U[Record Telemetry]
        U --> E

        E -->|No| V[Landing Detected]
    end

    subgraph "Post-Processing"
        V --> W[Analyze Performance]
        W --> X[Generate Visualizations]
        X --> Y[Export Results]
    end

    style I fill:#ff6b6b
    style J fill:#ffa500
    style K fill:#4ecdc4
    style E fill:#ffd700
    style V fill:#95a5a6
```

## 2. Physics Model - Force and Torque Diagram

```mermaid
graph LR
    subgraph "Forces (Translational)"
        F1[Gravity<br/>F_g = -mg] --> FNet[Net Force<br/>ΣF = ma]
        F2[Drag<br/>F_d = -½ρv²AC_d] --> FNet
        F3[Thrust<br/>F_t ≈ 0<br/>unpowered] --> FNet
        FNet --> Acc[Acceleration<br/>a = ΣF/m]
        Acc --> Vel[Velocity<br/>v = ∫a dt]
        Vel --> Pos[Position<br/>x = ∫v dt]
    end

    subgraph "Torques (Rotational)"
        T1[Control Torque<br/>τ_c] --> TNet[Net Torque<br/>Στ = Iα]
        T2[Aero Torque<br/>τ_a = qAL sin α] --> TNet
        T3[Damping<br/>τ_d = -k_d ω] --> TNet
        TNet --> AngAcc[Angular Accel<br/>α = Στ/I]
        AngAcc --> AngVel[Angular Velocity<br/>ω = ∫α dt]
        AngVel --> Att[Attitude<br/>θ = ∫ω dt]
    end

    style FNet fill:#ffcccc
    style TNet fill:#ccccff
    style Pos fill:#90ee90
    style Att fill:#90ee90
```

## 3. State Vector Integration Flow

```mermaid
sequenceDiagram
    participant S as State Vector
    participant E as Environment
    participant C as Controller
    participant P as Physics Engine
    participant I as Integrator

    Note over S: [x,y,z,vx,vy,vz,<br/>θ,φ,ψ,ωx,ωy,ωz]

    S->>E: Current altitude & velocity
    E->>E: Calculate ρ(h)
    E->>C: Atmospheric density

    S->>C: Current attitude & ω
    C->>C: Compute error signals
    C->>C: Select control mode

    alt RCS Mode
        C->>P: τ_rcs = K_p(ω_d - ω)
    else Hybrid Mode
        C->>P: τ = w₁τ_rcs + w₂τ_aero
    else Grid Fins Mode
        C->>P: τ_aero = f(q, α)
    end

    P->>P: Calculate F_drag
    P->>P: Calculate F_gravity
    P->>P: F_net = ΣF
    P->>P: τ_net = Στ

    P->>I: a = F_net/m
    P->>I: α = τ_net/I

    I->>I: v(t+dt) = v(t) + a·dt
    I->>I: ω(t+dt) = ω(t) + α·dt
    I->>I: x(t+dt) = x(t) + v·dt
    I->>I: θ(t+dt) = θ(t) + ω·dt

    I->>S: Update state vector
```

## 4. Control Mode State Machine

```mermaid
stateDiagram-v2
    [*] --> RCS: h > 30km

    RCS: RCS Mode
    RCS: • Thruster-only control
    RCS: • High fuel consumption
    RCS: • Low atmospheric density

    Hybrid: Hybrid Mode
    Hybrid: • Weighted RCS + Aero
    Hybrid: • Moderate fuel use
    Hybrid: • Transition zone

    GridFins: Grid Fins Mode
    GridFins: • Aerodynamic-only
    GridFins: • Zero fuel consumption
    GridFins: • High air density

    Ground: Landed
    Ground: • Altitude = 0
    Ground: • Mission complete

    RCS --> Hybrid: h ≤ 30km
    Hybrid --> GridFins: h ≤ 10km
    GridFins --> Ground: h = 0

    note right of RCS
        Weight: 100% RCS
        Altitude: 30-40 km
        ρ ≈ 0.01 kg/m³
    end note

    note right of Hybrid
        Weight: w(h) blend
        Altitude: 10-30 km
        ρ ≈ 0.01-0.4 kg/m³
    end note

    note right of GridFins
        Weight: 100% Aero
        Altitude: 0-10 km
        ρ ≈ 0.4-1.2 kg/m³
    end note
```

## 5. Atmospheric Density Model

```mermaid
graph TD
    A[Altitude h] --> B{Model Type}

    B -->|Original WRONG| C[Linear Model]
    C --> C1[ρ = ρ₀ × h/h₀]
    C1 --> C2[Error: 1000× at 40km!]

    B -->|Corrected ✓| D[Exponential Model]
    D --> D1[ρ = ρ₀ × exp-h/H]
    D1 --> D2[Physically accurate]

    D2 --> E[Examples:]
    E --> E1[h=0: ρ=1.225 kg/m³]
    E --> E2[h=10km: ρ=0.414 kg/m³]
    E --> E3[h=20km: ρ=0.089 kg/m³]
    E --> E4[h=40km: ρ=0.003 kg/m³]

    style C fill:#ff6b6b
    style C2 fill:#ff0000,color:#fff
    style D fill:#4ecdc4
    style D2 fill:#00ff00,color:#000
```

## 6. Free Body Diagram

```
                    ↑ z (altitude)
                    |
                    |
         ┌──────────────────────┐
         │                      │
         │    Grid Fins ═══╪═══ │ ← τ_aero (aerodynamic moment)
         │         ╔════╗       │
         │         ║ RV ║       │ ← RCS thrusters (τ_rcs)
         │         ╚════╝       │
         │      (CoM)           │
         │         ●            │
         └──────────────────────┘
                   ↓
                 F_g = mg
                (gravity)

         ←───── F_drag (aerodynamic drag)


Forces:
  F_gravity = -m × g                    (downward)
  F_drag = -½ρv²AC_d                    (opposes motion)
  F_net = F_gravity + F_drag

Torques:
  τ_rcs = F_thruster × arm              (thruster moment)
  τ_aero = ½ρv²AL × sin(α)             (aerodynamic moment)
  τ_damping = -k_d × ω                  (angular damping)
  τ_net = τ_control + τ_aero + τ_damping
```

## 7. Data Flow Architecture

```mermaid
graph TB
    subgraph "Input Layer"
        I1[Initial Conditions]
        I2[Physical Constants]
        I3[Control Parameters]
    end

    subgraph "Computation Layer"
        C1[Atmospheric Model]
        C2[Control Mode Selector]
        C3[Force Calculator]
        C4[Torque Calculator]
        C5[Dynamics Integrator]
    end

    subgraph "State Layer"
        S1[Position x,y,z]
        S2[Velocity vx,vy,vz]
        S3[Attitude θ,φ,ψ]
        S4[Angular Vel ωx,ωy,ωz]
    end

    subgraph "Output Layer"
        O1[Telemetry Arrays]
        O2[Performance Metrics]
        O3[Visualizations]
    end

    I1 --> S1
    I1 --> S2
    I1 --> S3
    I1 --> S4

    I2 --> C1
    I2 --> C3
    I2 --> C4

    I3 --> C2

    S1 --> C1
    C1 --> C3

    S2 --> C3
    S3 --> C2
    S3 --> C4
    S4 --> C4

    C2 --> C4
    C3 --> C5
    C4 --> C5

    C5 --> S1
    C5 --> S2
    C5 --> S3
    C5 --> S4

    S1 --> O1
    S2 --> O1
    S3 --> O1
    S4 --> O1

    O1 --> O2
    O2 --> O3
```

## 8. Hybrid Control Weighting Function

```mermaid
graph LR
    subgraph "Altitude Ranges"
        A1[h > 30km] --> W1[w_rcs = 1.0<br/>w_aero = 0.0]
        A2[h = 20km] --> W2[w_rcs = 0.5<br/>w_aero = 0.5]
        A3[h < 10km] --> W3[w_rcs = 0.0<br/>w_aero = 1.0]
    end

    subgraph "Control Blend"
        W1 --> B[τ_total = w_rcs × τ_rcs +<br/>w_aero × τ_aero]
        W2 --> B
        W3 --> B
    end

    B --> C[Applied Torque]

    style W1 fill:#ff6b6b
    style W2 fill:#ffa500
    style W3 fill:#4ecdc4
```

## 9. Performance Metrics Pipeline

```mermaid
graph TD
    A[Simulation Data] --> B{Metric Type}

    B --> C1[Fuel Efficiency]
    C1 --> C11[Total Fuel Used]
    C1 --> C12[Fuel by Mode]
    C1 --> C13[Specific Impulse]

    B --> C2[Control Performance]
    C2 --> C21[Torque Magnitude]
    C2 --> C22[Angular Stability]
    C2 --> C23[Attitude Error]

    B --> C3[Trajectory Accuracy]
    C3 --> C31[Lateral Deviation]
    C3 --> C32[Descent Rate]
    C3 --> C33[Landing Precision]

    B --> C4[Mode Analysis]
    C4 --> C41[Time in Each Mode]
    C4 --> C42[Transition Smoothness]
    C4 --> C43[Mode Efficiency]

    C11 --> D[Comparative Report]
    C12 --> D
    C13 --> D
    C21 --> D
    C22 --> D
    C23 --> D
    C31 --> D
    C32 --> D
    C33 --> D
    C41 --> D
    C42 --> D
    C43 --> D

    D --> E[Visualization]
    E --> E1[Time Series Plots]
    E --> E2[Mode Comparison Charts]
    E --> E3[3D Trajectory]
```

## 10. Numerical Integration Method

```mermaid
graph LR
    subgraph "Time Step n"
        T1[State: X_n] --> T2[Compute: F_n, τ_n]
        T2 --> T3[Calculate: a_n, α_n]
    end

    subgraph "Integration (Euler)"
        T3 --> I1[v_n+1 = v_n + a_n × Δt]
        T3 --> I2[ω_n+1 = ω_n + α_n × Δt]
        I1 --> I3[x_n+1 = x_n + v_n × Δt]
        I2 --> I4[θ_n+1 = θ_n + ω_n × Δt]
    end

    subgraph "Time Step n+1"
        I3 --> N1[State: X_n+1]
        I4 --> N1
    end

    N1 --> N2{Continue?}
    N2 -->|Yes| T1
    N2 -->|No| N3[End]

    style I1 fill:#ffffcc
    style I2 fill:#ffffcc
    style I3 fill:#ccffcc
    style I4 fill:#ccffcc
```

## 11. Error Comparison: Original vs Corrected

```mermaid
graph TB
    subgraph "Original Errors"
        E1[❌ Stops at 15km]
        E2[❌ Linear atmosphere<br/>1000× error]
        E3[❌ Constant velocity]
        E4[❌ Wrong torque units]
        E5[❌ Missing mass/inertia]
        E6[❌ List appends<br/>100× slower]
    end

    subgraph "Corrections"
        F1[✅ Reaches ground]
        F2[✅ Exponential atmosphere<br/>physically accurate]
        F3[✅ Dynamic velocity<br/>from force balance]
        F4[✅ Consistent units<br/>proper physics]
        F5[✅ Realistic parameters<br/>25-ton vehicle]
        F6[✅ Pre-allocated arrays<br/>100× faster]
    end

    E1 -.->|Fixed| F1
    E2 -.->|Fixed| F2
    E3 -.->|Fixed| F3
    E4 -.->|Fixed| F4
    E5 -.->|Fixed| F5
    E6 -.->|Fixed| F6

    style E1 fill:#ff6b6b
    style E2 fill:#ff6b6b
    style E3 fill:#ff6b6b
    style E4 fill:#ff6b6b
    style E5 fill:#ff6b6b
    style E6 fill:#ff6b6b

    style F1 fill:#4ecdc4
    style F2 fill:#4ecdc4
    style F3 fill:#4ecdc4
    style F4 fill:#4ecdc4
    style F5 fill:#4ecdc4
    style F6 fill:#4ecdc4
```

## 12. Simulation Validation Checklist

```mermaid
graph TD
    A[Run Simulation] --> B{Validation Checks}

    B --> C1{Altitude = 0?}
    C1 -->|Yes| D1[✅ Pass]
    C1 -->|No| D1F[❌ Fail: Doesn't land]

    B --> C2{Velocity increases<br/>then decreases?}
    C2 -->|Yes| D2[✅ Pass]
    C2 -->|No| D2F[❌ Fail: Bad dynamics]

    B --> C3{RCS uses fuel,<br/>Grid Fins don't?}
    C3 -->|Yes| D3[✅ Pass]
    C3 -->|No| D3F[❌ Fail: Fuel logic error]

    B --> C4{Smooth mode<br/>transitions?}
    C4 -->|Yes| D4[✅ Pass]
    C4 -->|No| D4F[❌ Fail: Discontinuities]

    B --> C5{Lateral deviation<br/>< 1000m?}
    C5 -->|Yes| D5[✅ Pass]
    C5 -->|No| D5F[❌ Fail: Poor control]

    D1 --> E{All Pass?}
    D2 --> E
    D3 --> E
    D4 --> E
    D5 --> E

    E -->|Yes| F[✅ Valid Simulation]
    E -->|No| G[❌ Review Implementation]

    style F fill:#00ff00,color:#000
    style G fill:#ff0000,color:#fff
```

---

## Physics Equations Summary

### Atmospheric Density
```
ρ(h) = ρ₀ × exp(-h/H)

Where:
  ρ₀ = 1.225 kg/m³    (sea level density)
  H = 8500 m          (scale height)
  h = altitude        (meters)
```

### Translational Dynamics
```
F_gravity = -m × g
F_drag = -½ × C_d × ρ(h) × A × v²
F_net = F_gravity + F_drag

a = F_net / m
v(t+Δt) = v(t) + a × Δt
x(t+Δt) = x(t) + v(t) × Δt
```

### Rotational Dynamics
```
τ_rcs = K_p × (ω_desired - ω_current)
τ_aero = ½ × ρ × v² × A_fin × L × sin(α)
τ_damping = -K_d × ω
τ_net = τ_control + τ_aero + τ_damping

α = I⁻¹ × τ_net
ω(t+Δt) = ω(t) + α × Δt
θ(t+Δt) = θ(t) + ω(t) × Δt
```

### Hybrid Control
```
w_rcs = (h - h_grid) / (h_rcs - h_grid)
w_rcs = clip(w_rcs, 0, 1)
w_aero = 1 - w_rcs

τ_total = w_rcs × τ_rcs + w_aero × τ_aero
```

---

**Note:** These diagrams represent the corrected simulation architecture with proper physics modeling, state integration, and control logic.
