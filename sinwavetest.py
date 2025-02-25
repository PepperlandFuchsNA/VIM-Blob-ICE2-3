import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

# Parameters
sampling_rate = 10000  # Sampling rate in Hz
duration = 1.0         # Duration in seconds
frequency = 500        # Frequency of the sine wave in Hz

# Generate time axis
t = np.linspace(0, duration, int(sampling_rate * duration), endpoint=False)

# Generate sine wave
y = np.sin(2 * np.pi * frequency * t)

# Plot the sine wave
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(t, y)
plt.title('500 Hz Sine Wave')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')

# Perform FFT
yf = fft(y)
xf = fftfreq(len(t), 1 / sampling_rate)

# Plot the FFT
plt.subplot(2, 1, 2)
plt.plot(xf, np.abs(yf))
plt.title('FFT of 500 Hz Sine Wave')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.xlim(0, 1000)  # Limit x-axis to 1000 Hz for better visualization

plt.tight_layout()
plt.show()