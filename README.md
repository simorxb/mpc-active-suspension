# Model Predictive Control - Active Suspension (Quarter-Car Model)

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/mpc-active-suspension)

## Summary
This project implements a Model Predictive Controller (MPC) for an active automotive suspension, modelled as a quarter-car system. The plant is described as a linear state-space model that augments the classical two-mass quarter-car with a fast first-order actuator lag, and the controller is designed and simulated in MATLAB using the Model Predictive Control Toolbox. A Manim Python script is also provided to generate the schematic of the system.

## Project Overview
Active suspension systems use a controllable force in parallel with the passive spring/damper to reject road disturbances and improve ride comfort and handling. The control challenge is to attenuate vertical body acceleration and limit suspension deflection while respecting the actuator's force budget, all in the presence of an unmeasured road profile.

This repository solves that problem with a linear MPC formulated on a quarter-car state-space model. The road profile $r$ enters as an unmeasured disturbance (UD), the active force command $f_s$ is the manipulated variable (MV), and the controlled outputs are body travel, suspension deflection, and body acceleration. A small actuator time constant is added so that the MPC plant has no direct feedthrough from the MV, which is required by the toolbox formulation, while keeping the closed-loop dynamics essentially unchanged.

The closed-loop response is then compared against the passive baseline ($f_s = 0$) on the same road bump, to quantify how much vibration is removed by the active controller for a given control-force effort.

### Key Features
- **Linear MPC** of a quarter-car active suspension, designed with the MATLAB MPC Toolbox.
- **Augmented state-space plant** including a first-order actuator lag to remove direct feedthrough from the MV.
- **Road profile as unmeasured disturbance**, with a smooth half-cosine bump used as the test scenario.
- **Comparison with the passive suspension** ($f_s = 0$) on the same input for a fair benchmark.
- **Schematic of the system** generated programmatically with Manim (`quarter_car_diagram.py`).

## Quarter-Car Modelling

### System Diagram
The quarter-car model lumps one corner of the vehicle into a body mass $m_b$ and a wheel mass $m_w$. The two masses are connected by a passive spring $k_s$ and a passive damper $b_s$ in parallel with a controllable force actuator $f_s$. The wheel mass is supported on the road by the tyre stiffness $k_t$, and the road height $r$ acts as a vertical input.

### Equations of Motion
Newton's second law applied to each mass gives:

$$
m_b \ddot{x}_b = -k_s (x_b - x_w) - b_s (\dot{x}_b - \dot{x}_w) + f_s
$$

$$
m_w \ddot{x}_w = k_s (x_b - x_w) + b_s (\dot{x}_b - \dot{x}_w) - k_t (x_w - r) - f_s
$$

Where:
- $x_b$, $\dot{x}_b$, $\ddot{x}_b$: body vertical displacement, velocity and acceleration.
- $x_w$, $\dot{x}_w$: wheel vertical displacement and velocity.
- $r$: road profile (vertical displacement input from the ground).
- $f_s$: active suspension force.
- $m_b$, $m_w$: body and wheel (sprung and unsprung) masses.
- $k_s$, $b_s$: passive spring stiffness and damping coefficient.
- $k_t$: tyre vertical stiffness.

### Augmented State-Space Model
The MATLAB MPC Toolbox does not allow direct feedthrough from the manipulated variable to any output. To handle this, the actuator is modelled as a first-order lag

$$
f_{s,\text{act}} = \frac{1}{\tau s + 1}\, f_{s,\text{cmd}}
$$

with $\tau$ small enough to leave closed-loop performance essentially unchanged. The actuator state is appended to the plant, giving the augmented state vector

$$
x = \begin{bmatrix} x_b & \dot{x}_b & x_w & \dot{x}_w & f_{s,\text{act}} \end{bmatrix}^T
$$

with inputs $u = [\,r,\; f_{s,\text{cmd}}\,]^T$ and outputs

$$
y = \begin{bmatrix} x_b \\\\ s_d \\\\ a_b \end{bmatrix} = \begin{bmatrix} x_b \\\\ x_b - x_w \\\\ \ddot{x}_b \end{bmatrix}
$$

Where:
- $s_d$: suspension deflection (body travel relative to wheel travel).
- $a_b$: body vertical acceleration, used as a comfort metric.

### Plant Parameters
- **Body mass** ($m_b$): 300 kg
- **Wheel mass** ($m_w$): 60 kg
- **Suspension stiffness** ($k_s$): 16,000 N/m
- **Suspension damping** ($b_s$): 1,000 N/(m/s)
- **Tyre stiffness** ($k_t$): 190,000 N/m
- **Actuator time constant** ($\tau$): 0.005 s

## MPC Design

### Control Objectives
The controller is designed to:
- Drive the suspension deflection $s_d$ and body acceleration $a_b$ to zero in the presence of the road disturbance.
- Reject the unmeasured road profile $r$ without measuring it directly.
- Respect the actuator force limits and avoid chatter on the manipulated variable.

### Signal Configuration
- Input 1: $r$ — unmeasured disturbance (UD).
- Input 2: $f_s$ — manipulated variable (MV).
- Outputs: $x_b$, $s_d$, $a_b$ — measured outputs.

### Controller Parameters
- **Sample time** ($T_s$): 0.01 s
- **Prediction horizon** ($p$): 40 steps
- **Control horizon** ($m$): 10 steps
- **MV bounds**: $f_s \in [-5, +5]$ kN
- **MV scale factor**: 2 (kN)
- **Output scale factors**: 0.05 m for $x_b$, 0.05 m for $s_d$, 1 m/s² for $a_b$
- **Output weights**: $[0,\; 3,\; 5]$ on $[x_b,\; s_d,\; a_b]$
- **MV rate weight**: 0.1

The output weights deliberately leave $x_b$ unweighted: the controller is asked to follow the road in body travel only as much as is needed to keep deflection and acceleration small, rather than trying to hold the body at an absolute reference.

## Simulation Scenario
The closed-loop is simulated for 3 s with a smooth half-cosine road bump as the unmeasured disturbance:

$$
r(t) = 0.025\,(1 - \cos(8\pi t)) \quad \text{for } 0 \le t \le 0.25\ \text{s}, \qquad r(t) = 0 \text{ otherwise}
$$

This produces a 0.05 m peak-to-peak bump entering the wheel through the tyre stiffness. The reference for all three outputs is held at zero. The same road profile is then applied to the open-loop plant with $f_s = 0$ to obtain the passive-suspension baseline used for comparison.

## Files
- **`init.m`** — Defines the physical parameters of the quarter-car ($m_b$, $m_w$, $k_s$, $b_s$, $k_t$).
- **`design_mpc.m`** — Builds the augmented state-space model, configures the MPC object, runs the closed-loop simulation against a road bump, simulates the passive baseline, and plots body travel, body acceleration, suspension deflection and control force.
- **`quarter_car_diagram.py`** — Manim script that renders the schematic of the quarter-car active-suspension model used in the project documentation.

### Typical run
1. Run `init` to load the plant parameters into the workspace.
2. Run `design_mpc` to build the controller, simulate the closed loop and the passive baseline, and plot the comparison.

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
