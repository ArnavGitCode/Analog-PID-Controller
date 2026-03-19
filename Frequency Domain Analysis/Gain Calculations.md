We are going to calculate the 2 types of gain in this part of the documentation: The voltage gain and the controller gain.
The voltage gain will be computed for all the 3 kinds of input: Sine wave input, Triangle wave input and the square wave input.
The input frequency is 2.5-2.6 Hz 
The gain calculated in this analysis represents the open-loop voltage magnitude of the RC plant. Since the plant is passive and contains no active elements, its gain remains less than unity across all operating frequencies. The observed attenuation increases with frequency, consistent with the low-pass nature of the RC network. This behavior necessitates the use of an active PID controller to provide the required gain and phase compensation in closed-loop operation.

##**LET US CALCULATE THE VOLTAGE GAINS FOR ALL 3 INPUTS**##

**Let us calculate the gain for sine wave input**
The gain is given by:
|G| = Vout/Vin or |G(jw)|=20log10(gain)
Vout = CH2 (blue channel) = 38.4 mVpp
Vin = CH1 (yellow channel) = 152 mVpp
|G| = 38.4/152 = 0.253 
The gain in dB is given by |G(jw)| = 20log10(0.253)
Gain (dB) = -11.9 dB

**Let us calculate the gain for triangle wave input**
The gain is given by:
|G| = Vout/Vin or |G(jw)|=20log10(gain)
Vout = CH2 (blue channel) = 32.8 mVpp
Vin = CH1 (yellow channel) = 132 mVpp
|G| = 32.8/132 = 0.248 
The gain in dB is given by |G(jw)| = 20log10(0.248)
Gain (dB) = -12.1 dB

**Let us calculate the gain for square wave input**
The gain is given by:
|G| = Vout/Vin or |G(jw)|=20log10(gain)
Vout = CH2 (blue channel) = 110 mVpp
Vin = CH1 (yellow channel) = 156 mVpp
|G| = 110/156 = 0.705 
The gain in dB is given by |G(jw)| = 20log10(0.705)
Gain (dB) = -3 dB

The gain in all 3 cases is less than 1 since the PID is composed of RC networks
R and C are only capable of dissipating and storing energy, respectively hence the voltage gain in all 3 cases is less than 1. 

##**LET US CALCULATE THE CONTROLLER GAIN**##
C(s) = Kp + (Ki/s) + (Kd*s)
We know that,
L(jw) = C(jw) * G(jw)
We shall take L(jw) = 1, which means that this is a unity feedback loop.
*Let us calculate the controller gain for the sine wave input*
1 = C(jw) * 0.253
C(jw) = 1/0.253
C(jw) = 4.03
*Let us calculate the controller gain for the triangle wave input*
1 = C(jw) * 0.248
C(jw) = 1/0.248
C(jw) = 3.95
*Let us calculate the controller gain for the square wave input*
1 = C(jw) * 0.705
C(jw) = 1/0.705
C(jw) = 1.42

The gain measured using sinusoidal and triangular inputs is similar because both waveforms are dominated by a single fundamental frequency component at the test frequency. Since the RC plant behaves as a linear time-invariant low-pass system, its magnitude response at a given frequency is independent of waveform shape, provided the signal energy is concentrated at that frequency. As a result, the measured attenuation for sine and triangle wave inputs closely reflects the true frequency-domain magnitude 
∣G(jω)∣ of the plant.
In contrast, a square wave contains significant DC and low-frequency harmonic components in addition to its fundamental frequency. The RC plant exhibits unity gain at DC and low frequencies, allowing these components to pass with minimal attenuation. Consequently, the time-domain peak-to-peak output of the square wave is influenced by low-frequency content rather than a single spectral component, leading to a higher apparent gain. For this reason, square-wave measurements reflect transient and averaging behavior rather than the plant’s true frequency response and are not suitable for Bode magnitude estimation.

We have successfully calculated the gains (both the voltage and the controller gains) and documented the results. 
