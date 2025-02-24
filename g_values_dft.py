#this code is used to calculate the FFT of the g values
#the g values are stored in a csv file or excel file

import numpy as np
import pandas as pd
from scipy.fft import fft
import os

import matplotlib.pyplot as plt

# Load the CSV file
file_path = 'g_values_slow_fan.xlsx'
file_extension = os.path.splitext(file_path)[1]

if file_extension == '.csv':
    data = pd.read_csv(file_path)
elif file_extension in ['.xls', '.xlsx']:
    data = pd.read_excel(file_path)
else:
    raise ValueError("Unsupported file format")

# Extract the first column (assuming the column name is 'actual_g')
actual_g = data.iloc[:1000, 0]
# Find the mean of the points and shift to zero
mean_value = actual_g.mean()
actual_g = actual_g - mean_value
# Plot the actual_g values
plt.figure()
plt.plot(actual_g)
plt.title('Actual G Values')
plt.xlabel('Sample')
plt.ylabel('Actual G')
plt.grid(True)
plt.show()

# Convert actual_g to a NumPy array and compute the FFT
fft_values = fft(actual_g.to_numpy())

# Plot the FFT
plt.figure()
plt.plot(np.abs(fft_values))
plt.title('FFT of Actual G Values')
plt.xlabel('Frequency')
plt.ylabel('Magnitude')
plt.grid(True)
plt.show()