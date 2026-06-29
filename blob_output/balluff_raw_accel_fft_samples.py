#!/usr/bin/env python3
"""
Calculate and plot a one-sided FFT from a Balluff BCM raw-acceleration CSV.

Configuration can be placed in a .env file beside this script.

The input can be limited to a selected section of the recording:
    FFT_START_SAMPLE=0
    FFT_SAMPLE_COUNT=32768

FFT_SAMPLE_COUNT=0 means use all samples from FFT_START_SAMPLE onward.

Command-line options override the env file.

Install:
    pip install numpy pandas matplotlib scipy

Run using the env file:
    python balluff_raw_accel_fft_samples.py

Override the sample range:
    python balluff_raw_accel_fft_samples.py recording.csv \
        --start-sample 10000 --sample-count 32768
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks


def default_env_file_path() -> Path:
    return Path(__file__).with_suffix(".env")


def load_env_file(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}

    if not path.exists():
        return values

    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        if "=" not in line:
            raise ValueError(
                f"Invalid env line {line_number} in {path}: {raw_line!r}"
            )

        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")

    return values


def env_text(values: Dict[str, str], key: str, default: str = "") -> str:
    value = values.get(key, "")
    return value if value != "" else default


def env_float(values: Dict[str, str], key: str, default: float) -> float:
    value = values.get(key, "")
    return float(value) if value != "" else default


def env_int(values: Dict[str, str], key: str, default: int) -> int:
    value = values.get(key, "")
    return int(value) if value != "" else default


def env_bool(values: Dict[str, str], key: str, default: bool) -> bool:
    value = values.get(key, "")

    if value == "":
        return default

    normalized = value.strip().lower()

    if normalized in {"1", "true", "yes", "y", "on"}:
        return True

    if normalized in {"0", "false", "no", "n", "off"}:
        return False

    raise ValueError(f"Invalid boolean value for {key}: {value!r}")


def inspect_balluff_csv(
    path: Path,
) -> tuple[dict[str, str], int, list[str]]:
    """Read only the metadata/header section of a Balluff CSV."""
    metadata: dict[str, str] = {}
    header_row: Optional[int] = None
    column_names: list[str] = []

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.reader(file)

        for row_number, row in enumerate(reader):
            if not row:
                continue

            first_value = row[0].strip()

            if first_value == "sample_index":
                header_row = row_number
                column_names = [value.strip() for value in row]
                break

            if len(row) >= 2:
                metadata[first_value] = row[1].strip()

    if header_row is None:
        raise ValueError(
            "Could not find the sample_index header. "
            "This does not appear to be a Balluff raw-acceleration CSV."
        )

    if "sampling_rate_ms" not in metadata:
        raise ValueError("CSV metadata does not contain sampling_rate_ms.")

    return metadata, header_row, column_names


def load_balluff_sample_range(
    path: Path,
    header_row: int,
    column_names: list[str],
    start_sample: int,
    sample_count: int,
) -> pd.DataFrame:
    """
    Load only the requested sample rows instead of reading the entire CSV.

    sample_count=0 means read all rows from start_sample onward.
    """
    rows_before_samples = header_row + 1
    skiprows = rows_before_samples + start_sample
    nrows = None if sample_count == 0 else sample_count

    data = pd.read_csv(
        path,
        header=None,
        names=column_names,
        skiprows=skiprows,
        nrows=nrows,
    )

    required_columns = {
        "sample_index",
        "sample_time_from_payload_start_ms",
        "acceleration_g",
    }
    missing = required_columns.difference(data.columns)

    if missing:
        raise ValueError(f"CSV is missing required columns: {sorted(missing)}")

    if data.empty:
        raise ValueError(
            f"No sample data was loaded. start_sample={start_sample} "
            f"may be beyond the end of the recording."
        )

    return data


def calculate_fft(
    signal_g: np.ndarray,
    sampling_rate_hz: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(signal_g) < 2:
        raise ValueError("At least two samples are required.")

    detrended_g = signal_g - np.mean(signal_g)
    window = np.hanning(len(detrended_g))
    window_sum = np.sum(window)

    if window_sum <= 0:
        raise ValueError("Invalid FFT window.")

    fft_values = rfft(detrended_g * window)
    frequencies_hz = rfftfreq(
        len(detrended_g),
        d=1.0 / sampling_rate_hz,
    )

    # One-sided peak-amplitude spectrum corrected for Hann-window coherent gain.
    amplitude_g = (2.0 / window_sum) * np.abs(fft_values)
    amplitude_g[0] *= 0.5

    if len(detrended_g) % 2 == 0:
        amplitude_g[-1] *= 0.5

    return detrended_g, frequencies_hz, amplitude_g


def find_top_peaks(
    frequencies_hz: np.ndarray,
    amplitude_g: np.ndarray,
    frequency_resolution_hz: float,
    maximum_frequency_hz: float,
    count: int = 20,
) -> pd.DataFrame:
    mask = (
        (frequencies_hz >= 1.0)
        & (frequencies_hz <= maximum_frequency_hz)
    )
    indices = np.flatnonzero(mask)

    if len(indices) == 0:
        return pd.DataFrame(
            columns=["frequency_hz", "amplitude_g_peak"]
        )

    minimum_spacing_hz = 2.0
    distance_bins = max(
        1,
        int(round(minimum_spacing_hz / frequency_resolution_hz)),
    )

    local_amplitude = amplitude_g[mask]
    prominence = max(
        float(np.median(local_amplitude)) * 3.0,
        0.0,
    )

    local_peaks, _ = find_peaks(
        local_amplitude,
        distance=distance_bins,
        prominence=prominence,
    )
    peak_indices = indices[local_peaks]

    if len(peak_indices) == 0:
        peak_indices = indices

    selected = peak_indices[
        np.argsort(amplitude_g[peak_indices])[-count:][::-1]
    ]

    return pd.DataFrame({
        "frequency_hz": frequencies_hz[selected],
        "amplitude_g_peak": amplitude_g[selected],
    })


def save_plots(
    time_s: np.ndarray,
    original_signal_g: np.ndarray,
    frequencies_hz: np.ndarray,
    amplitude_g: np.ndarray,
    output_dir: Path,
    output_stem: str,
    plot_name: str,
    maximum_frequency_hz: float,
    show: bool,
) -> None:
    time_path = output_dir / f"{output_stem}_time_signal.png"

    plt.figure(figsize=(11, 5))
    plt.plot(time_s, original_signal_g)
    plt.title(f"{plot_name}: raw acceleration")
    plt.xlabel("Time from recording start [s]")
    plt.ylabel("Acceleration [g]")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(time_path, dpi=180)

    if show:
        plt.show()

    plt.close()

    full_path = output_dir / f"{output_stem}_fft_full.png"

    plt.figure(figsize=(11, 5))
    plt.plot(frequencies_hz, amplitude_g)
    plt.title(f"{plot_name}: one-sided FFT")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Peak amplitude [g]")
    plt.xlim(0, frequencies_hz[-1])
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(full_path, dpi=180)

    if show:
        plt.show()

    plt.close()

    zoom_path = (
        output_dir
        / f"{output_stem}_fft_0_to_{maximum_frequency_hz:g}_hz.png"
    )
    mask = frequencies_hz <= maximum_frequency_hz

    plt.figure(figsize=(11, 5))
    plt.plot(frequencies_hz[mask], amplitude_g[mask])
    plt.title(f"{plot_name}: FFT, 0–{maximum_frequency_hz:g} Hz")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Peak amplitude [g]")
    plt.xlim(0, maximum_frequency_hz)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(zoom_path, dpi=180)

    if show:
        plt.show()

    plt.close()


def parse_args() -> argparse.Namespace:
    argv_probe = argparse.ArgumentParser(add_help=False)
    argv_probe.add_argument(
        "--env-file",
        type=Path,
        default=default_env_file_path(),
    )
    argv_probe.add_argument(
        "--no-env-file",
        action="store_true",
    )
    env_args, _ = argv_probe.parse_known_args()

    env_values: Dict[str, str] = {}

    if not env_args.no_env_file:
        env_values = load_env_file(env_args.env_file)

    merged_env = dict(env_values)
    merged_env.update(os.environ)

    default_csv_text = env_text(merged_env, "FFT_CSV_FILE", "")
    default_csv = Path(default_csv_text) if default_csv_text else None

    default_output_dir = Path(
        env_text(merged_env, "FFT_OUTPUT_DIR", "fft_output")
    )
    default_max_frequency = env_float(
        merged_env,
        "FFT_MAX_FREQUENCY_HZ",
        2000.0,
    )
    default_plot_name = env_text(
        merged_env,
        "FFT_PLOT_NAME",
        "",
    )
    default_start_sample = env_int(
        merged_env,
        "FFT_START_SAMPLE",
        0,
    )
    default_sample_count = env_int(
        merged_env,
        "FFT_SAMPLE_COUNT",
        0,
    )
    default_show = env_bool(
        merged_env,
        "FFT_SHOW",
        False,
    )

    parser = argparse.ArgumentParser(
        description=(
            "Calculate and plot an FFT from a selected sample range "
            "in a Balluff BCM raw-acceleration CSV."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "csv_file",
        type=Path,
        nargs="?",
        default=default_csv,
        help=(
            "Input Balluff CSV. Optional when FFT_CSV_FILE is "
            "defined in the env file."
        ),
    )
    parser.add_argument(
        "--env-file",
        type=Path,
        default=env_args.env_file,
        help="Path to the configuration env file.",
    )
    parser.add_argument(
        "--no-env-file",
        action="store_true",
        default=env_args.no_env_file,
        help="Do not load an env file.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=default_output_dir,
    )
    parser.add_argument(
        "--max-frequency",
        type=float,
        default=default_max_frequency,
        help=(
            "Maximum frequency shown in the zoomed plot "
            "and dominant-peaks table."
        ),
    )
    parser.add_argument(
        "--plot-name",
        default=default_plot_name,
        help=(
            "Display name used in plot titles. Defaults to "
            "the input CSV filename."
        ),
    )
    parser.add_argument(
        "--start-sample",
        type=int,
        default=default_start_sample,
        help="Zero-based sample at which processing begins.",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=default_sample_count,
        help=(
            "Number of samples to process. Use 0 to process every "
            "sample from --start-sample onward."
        ),
    )

    show_group = parser.add_mutually_exclusive_group()
    show_group.add_argument(
        "--show",
        dest="show",
        action="store_true",
        help="Display plots as well as saving them.",
    )
    show_group.add_argument(
        "--no-show",
        dest="show",
        action="store_false",
        help="Save plots without opening plot windows.",
    )
    parser.set_defaults(show=default_show)

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.csv_file is None:
        raise ValueError(
            "No CSV file was provided. Set FFT_CSV_FILE in the env "
            "file or pass a CSV filename on the command line."
        )

    if args.start_sample < 0:
        raise ValueError("--start-sample must be zero or greater.")

    if args.sample_count < 0:
        raise ValueError("--sample-count must be zero or greater.")

    if args.sample_count == 1:
        raise ValueError(
            "--sample-count must be 0 for all samples or at least 2."
        )

    if args.max_frequency <= 0:
        raise ValueError("--max-frequency must be positive.")

    csv_file = args.csv_file.expanduser()

    if not csv_file.exists():
        raise FileNotFoundError(
            f"Input CSV file not found: {csv_file.resolve()}"
        )

    metadata, header_row, column_names = inspect_balluff_csv(csv_file)
    data = load_balluff_sample_range(
        csv_file,
        header_row,
        column_names,
        args.start_sample,
        args.sample_count,
    )

    sampling_rate_ms = float(metadata["sampling_rate_ms"])

    if sampling_rate_ms <= 0:
        raise ValueError("sampling_rate_ms must be positive.")

    sampling_rate_hz = 1000.0 / sampling_rate_ms
    signal_g = pd.to_numeric(
        data["acceleration_g"],
        errors="coerce",
    ).dropna().to_numpy(dtype=float)

    if len(signal_g) < 2:
        raise ValueError(
            "The selected sample range contains fewer than two valid samples."
        )

    _, frequencies_hz, amplitude_g = calculate_fft(
        signal_g,
        sampling_rate_hz,
    )

    actual_sample_count = len(signal_g)
    duration_s = actual_sample_count / sampling_rate_hz
    resolution_hz = sampling_rate_hz / actual_sample_count
    nyquist_hz = sampling_rate_hz / 2.0
    maximum_frequency_hz = min(
        args.max_frequency,
        nyquist_hz,
    )

    output_dir = args.output_dir.expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    base_stem = csv_file.stem
    if args.start_sample == 0 and args.sample_count == 0:
        output_stem = base_stem
    else:
        output_stem = (
            f"{base_stem}_s{args.start_sample}_n{actual_sample_count}"
        )

    plot_name = args.plot_name.strip() or base_stem
    if args.start_sample != 0 or args.sample_count != 0:
        plot_name = (
            f"{plot_name} "
            f"(samples {args.start_sample}–"
            f"{args.start_sample + actual_sample_count - 1})"
        )

    spectrum_path = (
        output_dir / f"{output_stem}_fft_spectrum.csv"
    )
    pd.DataFrame({
        "frequency_hz": frequencies_hz,
        "amplitude_g_peak": amplitude_g,
    }).to_csv(spectrum_path, index=False)

    peaks = find_top_peaks(
        frequencies_hz,
        amplitude_g,
        resolution_hz,
        maximum_frequency_hz,
    )
    peaks_path = (
        output_dir / f"{output_stem}_top_fft_peaks.csv"
    )
    peaks.to_csv(peaks_path, index=False)

    # Use the original recording timeline for the selected range.
    time_s = (
        args.start_sample + np.arange(actual_sample_count)
    ) / sampling_rate_hz

    save_plots(
        time_s,
        signal_g,
        frequencies_hz,
        amplitude_g,
        output_dir,
        output_stem,
        plot_name,
        maximum_frequency_hz,
        args.show,
    )

    available_samples_text = metadata.get("sample_count", "unknown")

    print(f"Environment file      : {args.env_file}")
    print(f"Input CSV             : {csv_file.resolve()}")
    print(f"CSV sample count      : {available_samples_text}")
    print(f"Start sample          : {args.start_sample}")
    print(f"Requested sample count: {args.sample_count} (0 means all)")
    print(f"Processed samples     : {actual_sample_count}")
    print(f"Sampling rate         : {sampling_rate_hz:.6f} Hz")
    print(f"Selected duration     : {duration_s:.6f} s")
    print(f"Frequency resolution  : {resolution_hz:.6f} Hz")
    print(f"Nyquist frequency     : {nyquist_hz:.6f} Hz")
    print(f"Mean removed          : {np.mean(signal_g):.9f} g")
    print()

    print("Strongest separated peaks in the selected range:")

    if peaks.empty:
        print("  No peaks found.")
    else:
        print(
            peaks.to_string(
                index=False,
                formatters={
                    "frequency_hz": "{:.3f}".format,
                    "amplitude_g_peak": "{:.9f}".format,
                },
            )
        )

    print()
    print(f"Outputs saved in: {output_dir.resolve()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
