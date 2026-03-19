**Objective**

The objective of this experiment is to characterize the dynamic behavior of the RC plant under sinusoidal excitation. Sinusoidal inputs are particularly useful for analyzing frequency response, phase lag, and amplitude attenuation, which are fundamental parameters in control system design and PID tuning.

**Experimental Setup**

A low-frequency sinusoidal signal was generated using a function generator and applied as the plant input. The output was measured across the RC network using a digital storage oscilloscope.

Input signal (CH1 – Yellow): Sinusoidal waveform from the function generator
Output signal (CH2 – Blue): Voltage across the RC plant
Operating frequency: Approximately 2.6 Hz
Measurement mode: Time-domain observation

**Observations**

Amplitude Attenuation
The output signal exhibits a reduced peak-to-peak amplitude compared to the input. This attenuation is characteristic of a first-order RC system operating as a low-pass filter, where the magnitude response decreases with increasing input frequency.

Phase Lag
A clear phase delay is observed between the input and output waveforms. The output sinusoid lags the input in time, indicating the presence of a frequency-dependent phase shift introduced by the RC time constant.

Waveform Linearity
The output waveform retains a clean sinusoidal shape without noticeable distortion, clipping, or saturation. This confirms that the plant is operating in the linear region, validating the use of linear control models for analysis and controller design.

**Conclusion**

The sinusoidal input test confirms that the RC plant exhibits stable, linear, first-order dynamics. The predictable attenuation and phase lag observed in the output make the system well-suited for PID control experimentation. These results serve as a reference point for subsequent controller tuning and closed-loop performance evaluation.
