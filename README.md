# Analog PID Controller (Op-Amp Based)

This project implements a continuous-time Proportional-Integral-Derivative (PID) controller using analog circuitry based on operational amplifiers. The objective is to demonstrate classical feedback control concepts in hardware, evaluate performance through simulation and testing, and compare theoretical predictions to real measurements.

---

##  Overview

PID controllers are widely used in closed-loop control systems across industrial, automotive, robotics, and instrumentation applications due to their simplicity, robustness, and effective tuning characteristics. Although modern controllers are commonly digital, analog PID implementations remain relevant for ultra-low-latency, power-constrained, or noise-sensitive systems.

In this design:
- **Proportional**, **Integral**, and **Derivative** actions are realized using op-amp networks.
- Component selection defines gain and time constants of the loop.
- System performance is analyzed through transient response characteristics.

---

##  System Architecture

The analog controller is structured as:

Reference → PID Controller → Plant/System → Feedback


Each block is hardware-implemented using:
- Operational amplifier LM324
- Passive components like resistors & capacitors
- Feedback networks

---

##   Control Theory

The continuous-time PID controller is defined as:

u(t) = Kp * e(t) + Ki ∫ e(t) dt + Kd * (de(t)/dt)


Where:
- `e(t)` = error between reference and system output
- `Kp` = proportional gain
- `Ki` = integral gain
- `Kd` = derivative gain

Design equations and component selections map:
- `Kp` → resistor ratios
- `Ki` → R-C integration network
- `Kd` → R-C differentiation network

---

##  Implementation Details

**Components Used:**
- Operational amplifier: LM324
- Supply voltage: +5V 
- Resistive network for gain control
- Capacitive network for I/D actions

**Design Stages:**
- PID loop gain design
- Stability considerations
- Bandwidth vs noise tradeoffs
- Compensation and tuning

---

##  Simulation

Simulation was performed using:
- KiCAD Spice

Simulation artifacts include:
- Step response
- Frequency response / Bode plot
- Noise response
- Tuning variations


##  Hardware Testing 

Hardware validation verifies:
- Closed-loop stability
- Overshoot
- Rise time
- Settling time
- Steady-state error

Testing was performed using:
- Oscilloscope
- Function generator
- DC supply


## Tuning Methods

Explored tuning techniques:
- Manual tuning
- Gain scaling
- Zero placement
- Time constant selection

## Skills Demonstrated

- Analog circuit design
- Operational amplifier applications
- Classical control theory
- Feedback systems
- Simulation & hardware testing
- Data analysis & tuning

---
