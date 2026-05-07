# This code calculates the FFT of g-values from a CSV or Excel file.
# It supports:
#   1. New CSV format: sample_index,g_value
#   2. Old CSV format: g_value,counter
#   3. Excel files with similar columns

import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.fft import rfft, rfftfreq


# ------------------------------------------------------------
# User settings
# ------------------------------------------------------------

file_path = "g_values.csv"

# Use the actual VIM sampling rate.
# If raw_data_sampling_rate = 0 in the VIM code, use 64000.
# If raw_data_sampling_rate = 5 in the VIM code, use 2000.
sampling_rate = 64000  # Hz

plot_title_name = "fast fan"


# ------------------------------------------------------------
# Load CSV or Excel file
# ------------------------------------------------------------

def load_table(path):
    """
    Load a CSV or Excel file.

    If the CSV was written without a header, pandas may accidentally treat
    the first data row as the header. This function detects that case and
    reloads the file with header=None.
    """
    path = Path(path)
    file_extension = path.suffix.lower()

    if file_extension == ".csv":
        data = pd.read_csv(path)

        # Detect old no-header CSV files.
        # Example old row:
        #   -0.12345,7
        # pandas may treat "-0.12345" and "7" as column names.
        if columns_look_numeric(data.columns):
            data = pd.read_csv(path, header=None)

    elif file_extension in [".xls", ".xlsx"]:
        data = pd.read_excel(path)

        if columns_look_numeric(data.columns):
            data = pd.read_excel(path, header=None)

    else:
        raise ValueError(f"Unsupported file format: {file_extension}")

    return data


def columns_look_numeric(columns):
    """
    Return True if every column name looks like a number.

    This usually means the file had no header and pandas interpreted
    the first data row as column names.
    """
    for column in columns:
        try:
            float(column)
        except ValueError:
            return False
        except TypeError:
            return False

    return True


def extract_g_values(data):
    """
    Extract the g-value column from the loaded table.

    Preferred format:
        sample_index,g_value

    Also supports:
        actual_g
        Actual G
        old no-header format: g_value,counter
    """
    # Normalize column names so "G Value", "g_value", "Actual G" are easy to match.
    normalized_columns = {
        str(column).strip().lower().replace(" ", "_"): column
        for column in data.columns
    }

    possible_g_columns = [
        "g_value",
        "actual_g",
        "actual_g_value",
        "g",
    ]

    for name in possible_g_columns:
        if name in normalized_columns:
            column = normalized_columns[name]
            return pd.to_numeric(data[column], errors="coerce").dropna()

    # If there is only one column, assume it is the g-value column.
    if len(data.columns) == 1:
        return pd.to_numeric(data.iloc[:, 0], errors="coerce").dropna()

    # If there are two columns with no useful header:
    #   old original code: column 0 = g_value, column 1 = counter
    #   newer code:        column 0 = sample_index, column 1 = g_value
    col0 = pd.to_numeric(data.iloc[:, 0], errors="coerce")
    col1 = pd.to_numeric(data.iloc[:, 1], errors="coerce")

    # Heuristic: if column 0 looks like sample index 0,1,2,3...
    # then use column 1 as g-values.
    sample_index = np.arange(len(col0))
    if np.allclose(col0.dropna().to_numpy(), sample_index[:len(col0.dropna())]):
        return col1.dropna()

    # Otherwise assume old original format: g_value,counter.
    return col0.dropna()


# ------------------------------------------------------------
# FFT calculation
# ------------------------------------------------------------

def calculate_fft(g_values, fs):
    """
    Calculate one-sided FFT for real-valued vibration data.

    Steps:
      1. Convert to NumPy array.
      2. Remove the mean so DC does not dominate the FFT.
      3. Apply a Hann window to reduce spectral leakage.
      4. Use rfft because the signal is real-valued.
      5. Scale magnitude so amplitudes are easier to compare.
    """
    signal = np.asarray(g_values, dtype=float)

    if len(signal) < 2:
        raise ValueError("Need at least two g-value samples to calculate FFT.")

    # Remove DC offset.
    signal = signal - np.mean(signal)

    # Window reduces leakage when the signal does not contain an exact integer
    # number of cycles inside the capture.
    window = np.hanning(len(signal))
    windowed_signal = signal * window

    fft_values = rfft(windowed_signal)
    frequencies = rfftfreq(len(signal), d=1.0 / fs)

    # Amplitude scaling.
    # sum(window) compensates for the Hann window.
    magnitude = (2.0 / np.sum(window)) * np.abs(fft_values)

    return signal, frequencies, magnitude


def print_basic_results(signal, frequencies, magnitude, fs):
    """
    Print useful summary information.
    """
    sample_count = len(signal)
    duration = sample_count / fs
    frequency_resolution = fs / sample_count

    print(f"Samples: {sample_count}")
    print(f"Sampling rate: {fs} Hz")
    print(f"Duration: {duration:.6f} seconds")
    print(f"Frequency resolution: {frequency_resolution:.3f} Hz")

    # Ignore DC bin at index 0 when finding dominant vibration frequency.
    if len(magnitude) > 1:
        peak_index = np.argmax(magnitude[1:]) + 1
        print(f"Dominant frequency: {frequencies[peak_index]:.3f} Hz")
        print(f"Dominant magnitude: {magnitude[peak_index]:.6f}")


# ------------------------------------------------------------
# Plotting
# ------------------------------------------------------------

def plot_time_signal(signal, fs, title_name):
    """
    Plot g-values versus time.
    """
    time_axis = np.arange(len(signal)) / fs

    plt.figure()
    plt.plot(time_axis, signal)
    plt.title(f"Actual G Values for {title_name}")
    plt.xlabel("Time [s]")
    plt.ylabel("Actual G")
    plt.grid(True)
    plt.show()


def plot_fft(frequencies, magnitude, title_name):
    """
    Plot one-sided FFT magnitude.
    """
    plt.figure()
    plt.plot(frequencies, magnitude)
    plt.title(f"FFT of Actual G Values for {title_name}")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Magnitude")
    plt.grid(True)
    plt.show()




# ------------------------------------------------------------
# Main program
# ------------------------------------------------------------

def main():
    data = load_table(file_path)
    g_values = extract_g_values(data)

    signal, frequencies, magnitude = calculate_fft(g_values, sampling_rate)

    print_basic_results(signal, frequencies, magnitude, sampling_rate)

    plot_time_signal(signal, sampling_rate, plot_title_name)
    plot_fft(frequencies, magnitude, plot_title_name)


if __name__ == "__main__":
    main()