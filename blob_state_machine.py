# blob_state_machine_simple.py
"""
Simple Balluff BCM BLOB transfer runner.

What this script does:
  1. Sends a startup clear/recovery sequence.
  2. Waits for operator input.
  3. Records one dataset.
  4. Transfers all required BLOBs, for example rawX/rawY/rawZ.
  5. Saves each BLOB to a timestamped CSV.
  6. Waits for operator input before the next round.

This version is intentionally simple and avoids a large state-machine framework.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional
import time

import Balluff_blob_functions as BF


# ============================================================
# User settings
# ============================================================

DPTG_VALUE = 2          # 2 = data provider is triggered by ISDU
RADPTM_VALUE = 0        # 0 = raw acceleration starts after trigger
DCAS_VALUE = 6          # 6 = X, Y and Z axes

# DCT options:
#   0 = raw acceleration only
#   1 = amplitude/envelope spectrum only
#   2 = raw acceleration + amplitude/envelope spectrum
DCT_VALUE = 0

# Leave as None to auto-select from DCAS_VALUE and DCT_VALUE.
# With DCAS_VALUE=6 and DCT_VALUE=0, this becomes rawX/rawY/rawZ.
SELECTED_BLOB_TYPES: Optional[list[str]] = None

TIMEOUT_S = 120.0
STATUS_POLL_S = 0.2
PACKET_POLL_S = 0.05
OUTPUT_DIR = "blob_csv"

# Keep False unless your IO-Link master write response polling is stable.
WAIT_FOR_WRITE_RESPONSES = False

WORD_BYTE_ORDER = "big"
OBJECT_ENDIAN = "big"
FREQUENCY_RESOLUTION_HZ = None


# ============================================================
# BLOB selection
# ============================================================

def axes_from_dcas(dcas_value: int) -> list[str]:
    dcas_to_axes = {
        0: ["X"],
        1: ["Y"],
        2: ["Z"],
        3: ["X", "Y"],
        4: ["X", "Z"],
        5: ["Y", "Z"],
        6: ["X", "Y", "Z"],
    }

    if dcas_value not in dcas_to_axes:
        raise ValueError(f"Unsupported DCAS value: {dcas_value}")

    return dcas_to_axes[dcas_value]


def blob_types_from_config(dcas_value: int, dct_value: int) -> list[str]:
    axes = axes_from_dcas(dcas_value)

    if dct_value == 0:
        return [f"raw{axis}" for axis in axes]

    if dct_value == 1:
        return [name for axis in axes for name in (f"AmpSpec{axis}", f"EnvSpec{axis}")]

    if dct_value == 2:
        raw = [f"raw{axis}" for axis in axes]
        spectra = [name for axis in axes for name in (f"AmpSpec{axis}", f"EnvSpec{axis}")]
        return raw + spectra

    raise ValueError(f"Unsupported DCT value: {dct_value}")


def validate_blob_types(blob_types: Iterable[str]) -> list[str]:
    blob_types = list(blob_types)

    if not blob_types:
        raise ValueError("No BLOB types selected.")

    for blob_type in blob_types:
        if blob_type not in BF.BLOB_TYPE_TO_ID:
            raise ValueError(f"Unsupported BLOB type: {blob_type}")

    return blob_types


# ============================================================
# Recovery / clear
# ============================================================

def send_blob_clear_sequence(
    wait_for_response: bool = WAIT_FOR_WRITE_RESPONSES,
    restart_raw_data_feature: bool = False,
) -> None:
    """
    Best-effort clear/recovery sequence.

    Finish -> Abort -> Finish handles the common interrupted states:
      - stopped during packet transfer
      - stopped after marker 0x40 but before BLOB_Finish
    """
    print("\nSending BLOB clear/recovery sequence...")

    commands = [
        ("BLOB_Finish", BF.Write_Blob_Finish),
        ("BLOB_Abort", BF.Write_Blob_Abort),
        ("BLOB_Finish", BF.Write_Blob_Finish),
    ]

    for name, function in commands:
        try:
            print(function(wait_for_response=wait_for_response))
        except Exception as exc:
            # Do not stop the program if the device rejects a clear command while idle.
            print(f"Warning: {name} failed or was ignored: {exc}")
        time.sleep(0.1)

    if restart_raw_data_feature:
        try:
            print(BF.Restart_Raw_Data_Feature(wait_for_response=wait_for_response))
            time.sleep(0.2)
        except Exception as exc:
            print(f"Warning: raw data feature restart failed: {exc}")

    print("BLOB clear/recovery sequence complete.")


# ============================================================
# Status helpers
# ============================================================

def read_selected_statuses(blob_types: Iterable[str]) -> dict[str, int]:
    status_map = BF.Read_Blob_Status_Map()
    return {blob_type: status_map[blob_type] for blob_type in blob_types}


def wait_for_status(
    blob_types: Iterable[str],
    allowed_statuses: int | set[int],
    timeout_s: float = TIMEOUT_S,
    poll_s: float = STATUS_POLL_S,
) -> dict[str, int]:
    """
    Wait until all selected BLOBs are in the requested status.

    Useful status values:
      1 = waiting for trigger
      2 = data provider preparing data
      3 = data ready for BLOB transfer
    """
    if isinstance(allowed_statuses, int):
        allowed_statuses = {allowed_statuses}

    start_time = time.monotonic()
    last_statuses: Optional[dict[str, int]] = None

    while time.monotonic() - start_time <= timeout_s:
        statuses = read_selected_statuses(blob_types)
        last_statuses = statuses

        readable = {
            name: f"{value} ({BF.BLOB_STATUS_TEXT.get(value, 'Unknown')})"
            for name, value in statuses.items()
        }
        print(f"Selected BLOB statuses: {readable}")

        if all(value in allowed_statuses for value in statuses.values()):
            return statuses

        time.sleep(poll_s)

    raise TimeoutError(
        f"Timeout waiting for statuses {sorted(allowed_statuses)}. "
        f"Last statuses: {last_statuses}"
    )


# ============================================================
# Record once
# ============================================================

def record_one_dataset(blob_types: list[str]) -> None:
    """Configure the sensor and trigger one complete data collection."""
    print("\nConfiguring BLOB data provider...")
    print(
        BF.Sensor_Blob_Configuration(
            DPTG_value=DPTG_VALUE,
            RADPTM_value=RADPTM_VALUE,
            DCAS_value=DCAS_VALUE,
            DCT_value=DCT_VALUE,
            wait_for_response=WAIT_FOR_WRITE_RESPONSES,
        )
    )

    print("\nWaiting for data provider to be ready for trigger, status 1...")
    wait_for_status(blob_types, allowed_statuses=1)

    print("\nStarting data collection...")
    print(BF.Trigger_Start_Collection(wait_for_response=WAIT_FOR_WRITE_RESPONSES))

    print("\nWaiting for sensor to start preparing data, status 2 or 3...")
    wait_for_status(blob_types, allowed_statuses={2, 3})

    print("\nWaiting for data ready, status 3...")
    wait_for_status(blob_types, allowed_statuses=3)

    print("\nDataset is ready for BLOB transfer.")


# ============================================================
# Transfer one BLOB
# ============================================================

def read_blob_channel() -> tuple[Optional[int], bytes]:
    response = BF.Read_Blob_Data()
    data = BF.response_to_bytes(response)

    if not data:
        return None, b""

    return data[0], data


def transfer_one_blob(blob_type: str) -> bytes:
    """
    Transfer one BLOB and return its payload bytes.

    Marker handling:
      0x10 = BLOB start accepted
      0x2n = data packet, n is a modulo-16 packet counter
      0x30 = final data packet
      0x40 = ready to finish transfer
    """
    print(f"\nStarting BLOB transfer: {blob_type}")
    print(BF.Get_Blob_Data_cmd(Type=blob_type, wait_for_response=WAIT_FOR_WRITE_RESPONSES))

    payload = bytearray()
    expected_counter = 0
    packet_count = 0
    final_packet_saved = False
    start_time = time.monotonic()

    while time.monotonic() - start_time <= TIMEOUT_S:
        marker, data = read_blob_channel()

        if marker is None:
            time.sleep(PACKET_POLL_S)
            continue

        print(f"{blob_type}: marker 0x{marker:02X}, bytes {len(data)}")

        if marker == 0x10:
            time.sleep(PACKET_POLL_S)
            continue

        if (marker & 0xF0) == 0x20:
            counter = marker & 0x0F

            if counter != expected_counter:
                print(
                    f"Warning: {blob_type} packet counter mismatch. "
                    f"Expected {expected_counter}, got {counter}."
                )

            expected_counter = (counter + 1) % 16
            payload.extend(data[1:])
            packet_count += 1
            time.sleep(PACKET_POLL_S)
            continue

        if marker == 0x30:
            if not final_packet_saved:
                payload.extend(data[1:])
                packet_count += 1
                final_packet_saved = True
                print(f"{blob_type}: final data packet received.")
            else:
                print(f"{blob_type}: repeated final packet marker ignored.")

            time.sleep(PACKET_POLL_S)
            continue

        if marker == 0x40:
            print(f"{blob_type}: transfer complete, sending BLOB_Finish.")
            print(BF.Write_Blob_Finish(wait_for_response=WAIT_FOR_WRITE_RESPONSES))
            print(f"{blob_type}: packets={packet_count}, payload_bytes={len(payload)}")
            return bytes(payload)

        print(f"{blob_type}: unexpected marker 0x{marker:02X}; continuing.")
        time.sleep(PACKET_POLL_S)

    raise TimeoutError(f"Timeout while transferring {blob_type}.")


# ============================================================
# CSV save
# ============================================================

def make_csv_path(blob_type: str, output_dir: str) -> str:
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return str(Path(output_dir) / f"{blob_type}_{timestamp}.csv")


def save_payload_to_csv(blob_type: str, payload: bytes, output_dir: str) -> str:
    csv_path = make_csv_path(blob_type, output_dir)
    blob_id = BF.BLOB_TYPE_TO_ID[blob_type]

    BF.parse_balluff_blob(
        data=payload,
        blob_id=blob_id,
        word_byte_order=WORD_BYTE_ORDER,
        object_endian=OBJECT_ENDIAN,
        csv_path=csv_path,
        frequency_resolution_hz=FREQUENCY_RESOLUTION_HZ,
        print_values=True,
    )

    print(f"{blob_type}: CSV saved to {csv_path}")
    return csv_path


# ============================================================
# One full round
# ============================================================

def run_one_round(round_number: int, blob_types: list[str]) -> dict[str, str]:
    """
    One round:
      record once -> transfer all selected BLOBs -> save CSVs
    """
    round_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    round_output_dir = str(Path(OUTPUT_DIR) / f"round_{round_number:03d}_{round_timestamp}")

    print("\n====================================================")
    print(f"Round {round_number} started")
    print(f"Selected BLOB types: {blob_types}")
    print(f"Output folder: {round_output_dir}")

    record_one_dataset(blob_types)

    csv_paths: dict[str, str] = {}

    for blob_type in blob_types:
        payload = transfer_one_blob(blob_type)
        csv_paths[blob_type] = save_payload_to_csv(blob_type, payload, round_output_dir)
        time.sleep(0.2)

    print(f"\nRound {round_number} complete.")
    return csv_paths


# ============================================================
# Main loop
# ============================================================

def main() -> None:
    if SELECTED_BLOB_TYPES is None:
        blob_types = blob_types_from_config(DCAS_VALUE, DCT_VALUE)
    else:
        blob_types = SELECTED_BLOB_TYPES

    blob_types = validate_blob_types(blob_types)

    print("\nSimple Balluff BLOB record/transfer program")
    print(f"Selected BLOB types: {blob_types}")
    print("Press Enter to record and transfer one complete dataset.")
    print("Type q and press Enter to quit.")

    # Important recovery step for a previous aborted run.
    send_blob_clear_sequence(restart_raw_data_feature=True)

    round_number = 1

    while True:
        choice = input("\nPress Enter to start next round, or q to quit: ").strip().lower()

        if choice in {"q", "quit", "exit", "stop"}:
            print("Operator stopped the program.")
            break

        try:
            csv_paths = run_one_round(round_number, blob_types)

            print("\nCSV files created:")
            for blob_type, path in csv_paths.items():
                print(f"  {blob_type}: {path}")

            round_number += 1

        except KeyboardInterrupt:
            print("\nProgram interrupted during transfer.")
            send_blob_clear_sequence(restart_raw_data_feature=False)
            raise

        except Exception as exc:
            print(f"\nRound failed: {exc}")
            send_blob_clear_sequence(restart_raw_data_feature=False)
            print("Fix the issue, then press Enter to try another round or q to quit.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting after keyboard interrupt.")
    finally:
        # Best effort cleanup on normal exit or Ctrl+C.
        send_blob_clear_sequence(restart_raw_data_feature=False)
