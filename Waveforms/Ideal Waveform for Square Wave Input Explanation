The plot shows several periods of a square-wave input (blue) compared against the response of a first-order RC plant (orange). The square wave switches between logic levels with effectively infinite slope (dv/dt -> infinity), meaning it carries energy across a wide harmonic spectrum. In contrast, the RC network exhibits the classical first-order exponential rise and decay behavior, where the output approaches the new command with time constant t=RC. This creates rounded transitions and a lagging response that never matches the instantaneous nature of the reference.
Over multiple cycles, the difference becomes more pronounced: during each high interval, the RC output charges toward steady-state, and during each low interval it discharges back toward zero, both governed by the same time constant. As the input frequency increases, the plant spends proportionally less time near steady-state because the system is dominated by transient behavior. This illustrates bandwidth limitation in the time domain — the RC plant only passes low frequency components of the square wave and rejects higher harmonics.

**How this differs from real hardware waveforms**
Breadboard Parasitics
Stray capacitances, inductances, and resistances distort the ideal exponential, especially near transitions.

Op-Amp Slew Rate and GBW Limit
If the plant is driven through an op-amp buffer, slew-rate limiting and finite gain-bandwidth introduce additional lag, rounding, or small overshoot.

Derivative & Measurement Interference
If the PID derivative stage is active in the loop, it amplifies high-frequency noise and edge transitions, adding visible spikes not present in the ideal curve.

Discrete Test Equipment Limits
Trigger stability, sample rate, and bandwidth limitations affect how sharp edges and transitions appear in real measurements.


