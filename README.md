# Active Suspension Control Analysis for a Quarter Car Model

This repository contains MATLAB and Simulink implementations of several control strategies for an **Active Quarter Car Suspension System**. The system is modeled using physical parameters corresponding to the front suspension of a BMW vehicle. 

The primary objective of these control systems is to improve passenger ride comfort (minimizing vertical acceleration and body displacement) while maintaining road-holding capability (minimizing suspension travel and tire deflection) under road disturbances.

---

## 🚘 Vehicle Parameters (BMW Front Side)

The model is parameterized using the following values:
* **Sprung Mass (Car Body), $m_c$**: $408 \text{ kg}$
* **Unsprung Mass (Wheel Assembly), $m_{us}$**: $48.3 \text{ kg}$
* **Suspension Stiffness, $k_k$**: $340 \text{ kN/m}$
* **Tire Stiffness, $k_r$**: $30 \text{ kN/m}$
* **Suspension Damping, $b_r$**: $1450 \text{ Ns/m}$

---

## 🛠 Control Strategies Implemented

### 1. Linear Quadratic Regulator (LQR) with Genetic Algorithm
* **Folder**: `1. LQR/`
* **Theory**: Minimizes the quadratic cost function representing a trade-off between state errors and control inputs.
* **Optimization**: The weighting matrices $Q$ and $R$ are optimized via a **Genetic Algorithm (GA)**. The GA evaluates a fitness function that penalizes settling time, overshoot, steady-state error, and integrated square error (ISE) from simulations.
* **Simulation**: Features both Simulink (`CariLQR.slx`) and pure ODE solver (`GA.m`) implementations.

### 2. Hybrid LQR-PID (Backstepping Integral Augmentation)
* **Folder**: `3. LQR-PID/`
* **Theory**: Connects modern LQR control with industrial PID controllers. It uses a **Backstepping Integral Augmentation** technique on the state-space model to formulate an augmented system. 
* **Mapping**: The optimal state-feedback gain $K$ computed via LQR is transformed into equivalent PID parameters ($K_p$, $K_i$, $K_d$) using a mapping matrix $\gamma$ and its pseudo-inverse.
* **Optimization**: GA is used to optimize the 6-state augmented LQR weights before mapping to PID.

### 3. Pontryagin's Minimum Principle (PMP)
* **Folder**: `4. PMP/`
* **Theory**: An optimal control framework based on solving the state and co-state differential equations derived from the Hamiltonian system.
* **Status**: Model structure and weighting matrix setups are defined.

### 4. Internal Model Principle (IMP)
* **Folder**: `5. IMP/`
* **Theory**: To achieve zero steady-state tracking error for specific reference signals or disturbances, a dynamic compensator (internal model) containing the poles of the reference/disturbance is inserted into the loop.
* **Implementation**: The plant is augmented with a compensator designed for target poles (`[-1, 0]`). LQR weighting matrices for the combined 6-state model are optimized via GA, and simulated in `CariIMP.slx`.

### 5. Reinforcement Learning (RL)
* **Folder**: `6. RL/`
* **Status**: Simulink modeling (`FindRL.slx`) set up for Reinforcement Learning agent interaction.

---

## 📁 Repository Structure

```bash
AnalysisQuarterCarSuspension/
├── 0. Referensi/             # Reference materials, PDFs, and lecture notes
├── 1. LQR/                   # LQR control & Genetic Algorithm optimization
│   ├── CariLQR.slx           # Simulink simulation model
│   ├── FindLQR.m             # GA-LQR optimization script
│   └── GA.m                  # ODE45 validation and baseline comparison script
├── 2. PID/                   # Traditional PID control (placeholder directory)
│   └── FindPID.m
├── 3. LQR-PID/               # Hybrid LQR-PID translation via Backstepping
│   ├── CariLQR_PID.slx       # Simulink simulation model
│   ├── FindLQR_PID.m         # GA-LQR-PID optimization and translation script
│   └── cobacobaLQRPID.m      # Experimental translation script
├── 4. PMP/                   # Pontryagin's Minimum Principle setup
│   ├── CariPMP.slx           # Simulink simulation model
│   └── FindPMP.m             # LQR/PMP model definition script
├── 5. IMP/                   # Internal Model Principle (IMP) control
│   ├── CariIMP.slx           # Simulink simulation model
│   ├── FindIMP.m             # GA-IMP optimization script
│   └── cobacobaIMP.m         # Experimental IMP script
├── 6. RL/                    # Reinforcement Learning setup
│   └── FindRL.slx            # RL agent training/simulation model
├── Report.pdf                # Project summary report
├── .gitignore                # Excludes Simulink cache files (slprj, .slxc)
└── README.md                 # Project documentation
```

---

## 🚀 Step-by-Step Instructions

To run the optimizations and simulations, follow these steps:

### Prerequisites
* MATLAB (R2021a or newer recommended)
* Simulink
* Control System Toolbox
* Global Optimization Toolbox (for `ga` function)

### Running LQR Optimization
1. Open MATLAB and set your current directory to the `1. LQR/` folder.
2. Run the `FindLQR.m` script:
   ```matlab
   FindLQR
   ```
3. The Genetic Algorithm will initialize a population of size 20 and evolve up to 1000 generations to find the optimal diagonal elements of $Q$ and $R$.
4. Once completed, the script automatically triggers the Simulink model `CariLQR.slx`, runs a 30-second simulation, and plots the comparison between the **Baseline LQR** (with identity weights) and the **GA-Optimized LQR**.

### Running LQR-PID Mapping
1. Change your MATLAB directory to the `3. LQR-PID/` folder.
2. Run the `FindLQR_PID.m` script:
   ```matlab
   FindLQR_PID
   ```
3. The script will perform LQR optimization on the augmented state-space representation, map the feedback gains to PID parameters ($K_p$, $K_i$, $K_d$), and simulate the response in `CariLQR_PID.slx`.

### Running IMP Control
1. Change your MATLAB directory to the `5. IMP/` folder.
2. Run `FindIMP.m`:
   ```matlab
   FindIMP
   ```
3. This designs the compensator based on poles `[-1, 0]`, optimizes the augmented LQR weights, and runs the `CariIMP.slx` Simulink model.

---

## 📈 Expected Results

Upon running the optimization scripts, the scripts will generate:
1. **State Response Plots**: Comparisons of the state variables (displacements, velocities of sprung and unsprung masses) under base control and GA-optimized control.
2. **Output Response Plots**: Performance under road profile disturbances showing how effectively the controller dampens vibrations.
3. **Control Effort Plots**: The force input requested by the active actuator to check if it's within physical/practical limits.
4. **Cost History Plot**: Cost value vs. GA iteration/generation, demonstrating convergence.
5. **Eigenvalue Check**: Closed-loop eigenvalues will be printed in the MATLAB Command Window to verify asymptotic stability (all real parts must be negative).
