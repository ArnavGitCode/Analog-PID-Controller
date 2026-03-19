**Objective**

The objective of this experiment is to study the response of the RC plant to a triangular wave input. Unlike a sinusoidal signal, a triangular waveform contains a wide range of harmonic components, making it useful for evaluating the plant’s frequency-dependent attenuation, smoothing behavior, and phase characteristics.

**Experimental Setup**

A low-frequency triangular waveform was generated using a function generator and applied to the plant input. The output was measured across the RC network using a digital storage oscilloscope.

Input signal (CH1 – Yellow): Triangular wave from the function generator
Output signal (CH2 – Blue): Voltage across the RC plant
Operating frequency: Approximately 2.6 Hz
Measurement mode: Time-domain observation

**Observations**

Observations

Amplitude Attenuation
The output waveform exhibits a noticeable reduction in peak-to-peak amplitude relative to the input. This attenuation is consistent with the low-pass behavior of the RC plant, which suppresses higher-frequency harmonic components present in the triangular input.

Waveform Smoothing
The output signal appears smoother and more rounded compared to the input. This effect arises because the RC network attenuates the higher-order harmonics that define the sharp corners of a triangular waveform, causing the output to approach a sinusoidal shape.

Phase Lag
A time delay between the input and output waveforms is observed, indicating a phase lag introduced by the RC time constant. This lag is a function of frequency and becomes more pronounced as harmonic frequency components increase.

Linear Operation
No clipping, distortion, or saturation is observed in the output signal. This confirms that the plant remains within its linear operating region under triangular excitation.

**Conclusion**

The response of the RC plant to a triangular wave input demonstrates predictable attenuation, phase lag, and harmonic suppression, consistent with first-order system theory. The smoothing of the waveform confirms the low-pass nature of the plant and reinforces the suitability of linear control techniques for subsequent PID implementation.
