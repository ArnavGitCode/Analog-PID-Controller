# Analog PID Controller (Op-Amp Based)

This project presents a fully analog implementation of a classical Proportional–Integral–Derivative (PID) controller using operational amplifiers, passive components, and a first-order anti-aliasing filter. The objective was to physically realize closed-loop feedback control using purely continuous-time electronics, characterize the transient and steady-state behavior, tune the controller parameters, and study the interplay between theory and hardware non-idealities.

While PID control is ubiquitous in digital microcontroller- and DSP-based environments, its analog implementation remains a powerful demonstration of classical control practice, signal conditioning, and real-world op-amp limitations. This project bridges formal control theory with practical circuit construction, highlighting the design trade-offs engineers face when building continuous controllers from scratch.
---

##  Overview

PID controllers are widely used in closed-loop control systems across industrial, automotive, robotics, and instrumentation applications due to their simplicity, robustness, and effective tuning characteristics. Although modern controllers are commonly digital, analog PID implementations remain relevant for ultra-low-latency, power-constrained, or noise-sensitive systems like motor drivers and power electronics.

In this design:
- **Proportional**, **Integral**, and **Derivative** actions are realized using op-amp networks.
- Component selection defines gain and time constants of the loop.
- System performance is analyzed through transient response characteristics.

---

##  System Architecture

The analog controller is structured as:

Reference input → Error Summer -> PID Controller → Summing Amplifier -> Plant/System → Feedback


Each block is hardware-implemented using:
- Operational amplifier LM324
- Passive components like resistors & capacitors
- Feedback networks

**Blocks Implemented**

Error Computing Block: subtracts feedback from reference to generate control error.
PID Network: implemented using three op-amp stages realizing P, I, and D actions in continuous time.
Analog Buffer/Driver: isolates the PID network from plant loading.
First Order RC Anti-Aliasing Filter: conditions the signal, attenuating high-frequency noise & derivative spikes.
Feedback Path: returns plant output for steady-state error correction.

**Op-Amp Selection**

Initially, the classic µA741 and LM324 were evaluated:
741: limited slew rate, bandwidth, and needs dual supply rails.
LM324: rail-to-rail-ish input range, single supply, slower but more practical for embedded analog.
The slew rate differences affected derivative action significantly — the D block can easily push op-amps into slew-limited behavior even for modest square-wave test signals.

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

  Each term has distinct dynamical effects:

P (Proportional): scales instantaneous error; increases system stiffness and bandwidth.
I (Integral): accumulates persistent error; eliminates steady-state error.
D (Derivative): predicts future error; improves damping, reduces overshoot, accelerates settling.

Analog PID networks typically realize:

Proportional via resistor ratios,
Integral via (1/sRC) ratios, 
Derivative via sRC lead networks 

In hardware, derivative terms must be filtered to avoid amplifying noise at high frequencies — hence the inclusion of RC poles, effectively forming a realistic PD compensator rather than an ideal unbounded differentiator.

**Proportional**

**Increasing K𝑝**:

Decreases rise time
Increases closed-loop gain
Increases risk of oscillation (reduced phase margin)

Measured using step/square test inputs. Excessive gain produced quasi-underdamped oscillatory ring.

**Increasing Ki**:

Eliminates steady-state error
Slows transient response if excessive
Creates low-frequency pole → risk of sustained oscillations

Integral action was verified by observing zero steady-state error with constant (DC) stimuli.

**Increasing Kd**

Increases damping
Reduces overshoot
Decreases settling time
Amplifies high-frequency noise if unfiltered

Derivative spikes were clearly visible on oscilloscope when fed a square input; the anti-aliasing filter suppressed high-frequency artifacts.
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

## Anti-Aliasing + RC filter reasoning 

Derivative action interacts strongly with input discontinuities. A square input causes theoretical impulse-like outputs via dV/dt. In hardware:

Op-amp slew limits these impulses,
RC filtering reduces noise & harmonics,
Prevents saturation of the PID output stage,
Creates realistic bounded control effort.

This mirrors industrial PID practice where derivative filtering is mandatory.

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

## Hardware Challenges Faced 

Several non-idealities emerged during construction:
**Slew Rate Limitations**
The derivative block demands high dV/dt. The 741 struggled for square inputs, visibly rounding transitions.
**Saturation and rail inputs**
Analog control effort saturates visibly near rails, unlike ideal simulations.
**Component Tolerances**
Discrete R/C tolerances shift time constants, requiring retuning.

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

## Future directions 

Possible extensions include:
Adding plant emulation (DC motor / thermal system),
Cascaded control (PI + PD stages),
Digital PID for performance comparison

## Conclusion 

This project demonstrated that classical PID controllers can be implemented in purely analog hardware using standard components, while revealing the deep connection between control theory and circuit-level constraints. Through hands-on experiments, the effects of Kp, Ki, and Kd were directly visualized, tested, and tuned — illustrating both the elegance and the practical challenges of continuous-time control.
  

---
