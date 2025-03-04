#this code is used to calculate the FFT of the g values
#the g values are stored in a csv file or excel file

import numpy as np
import pandas as pd
from scipy.fft import fft, fftfreq
import os

import matplotlib.pyplot as plt
duration = 0.5         # Duration in seconds

sampling_rate = 2000  # Sampling rate in Hz
t = np.linspace(0, duration, int(sampling_rate * duration), endpoint=False) #added time vector for consitency

# Load the CSV file
file_path = 'g_values_fast_fan_2.xlsx'
file_extension = os.path.splitext(file_path)[1]

if file_extension == '.csv':
    data = pd.read_csv(file_path)
elif file_extension in ['.xls', '.xlsx']:
    data = pd.read_excel(file_path)
else:
    raise ValueError("Unsupported file format")

# Extract the first column (assuming the column name is 'actual_g')
actual_g = data.iloc[:5000, 0]
# Find the mean of the points and shift to zero
mean_value = actual_g.mean()
actual_g = actual_g - mean_value

# Plot the actual_g values
plt.figure()
plt.plot(actual_g)
plt.title('Actual G Values for stopped fan')
plt.xlabel('Sample')
plt.ylabel('Actual G')
plt.grid(True)
plt.show()

# Convert actual_g to a NumPy array and compute the FFT
fft_values = fft(actual_g.to_numpy()) #changed to numpy array
xf = fftfreq(len(actual_g), 1 / sampling_rate) #compute frequency based on actual_g length
# Plot the FFT
plt.figure()
idx = xf >= 0
plt.plot(xf[idx], np.abs(fft_values)[idx]) #plotting only the positive frequencies

plt.title('FFT of Actual G Values for stopped fan')
plt.xlabel('Frequency')
plt.ylabel('Magnitude')
plt.grid(True)
plt.show()