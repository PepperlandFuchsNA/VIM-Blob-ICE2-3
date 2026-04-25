# blob_state_machine.py
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
from typing import Iterable, Optional
import logging
import time

import Balluff_blob_functions as BF

logger = logging.getLogger(__name__)


class BlobState(Enum):
    CONFIGURE_SENSOR = auto()
    WAIT_STATUS_1 = auto()
    START_COLLECTION = auto()
    WAIT_STATUS_2_OR_3 = auto()
    WAIT_STATUS_3 = auto()
    SEND_BLOB_START = auto()
    READ_BLOB_PACKETS = auto()
    WAIT_BLOB_END_40 = auto()
    SEND_BLOB_FINISH = auto()
    DONE = auto()
    ERROR = auto()


@dataclass
class BlobStateMachineResult:
    success: bool
    blob_type: str
    payload: bytes
    packet_count: int
    final_state: BlobState
    error: Optional[str] = None
    csv_path: Optional[str] = None


@dataclass
class MultiBlobStateMachineResult:
    success: bool
    results: dict[str, BlobStateMachineResult]
    final_state: BlobState
    error: Optional[str] = None


# ============================================================
# Blob selection helpers
# ============================================================

def get_axes_from_dcas(DCAS_value: int) -> list[str]:
    """
    DCAS:
      0 = X
      1 = Y
      2 = Z
      3 = X and Y
      4 = X and Z
      5 = Y and Z
      6 = X, Y and Z
    """
    dcas_to_axes = {
        0: ["X"],
        1: ["Y"],
        2: ["Z"],
        3: ["X", "Y"],
        4: ["X", "Z"],
        5: ["Y", "Z"],
        6: ["X", "Y", "Z"],
    }

    if DCAS_value not in dcas_to_axes:
        raise ValueError(f"Unsupported DCAS_value: {DCAS_value}")

    return dcas_to_axes[DCAS_value]


def get_blob_types_from_config(DCAS_value: int = 6, DCT_value: int = 0) -> list[str]:
    """
    Automatically decides which BLOBs to transfer based on configuration.

    DCT:
      0 = Collect raw acceleration data only
      1 = Collect amplitude and envelope spectrum data only
      2 = Collect raw acceleration data and amplitude/envelope spectrum data
    """
    axes = get_axes_from_dcas(int(DCAS_value))
    blob_types: list[str] = []

    if DCT_value == 0:
        for axis in axes:
            blob_types.append(f"raw{axis}")

    elif DCT_value == 1:
        for axis in axes:
            blob_types.append(f"AmpSpec{axis}")
            blob_types.append(f"EnvSpec{axis}")

    elif DCT_value == 2:
        for axis in axes:
            blob_types.append(f"raw{axis}")

        for axis in axes:
            blob_types.append(f"AmpSpec{axis}")
            blob_types.append(f"EnvSpec{axis}")

    else:
        raise ValueError(f"Unsupported DCT_value: {DCT_value}")

    return blob_types


def validate_blob_types(blob_types: Iterable[str]) -> list[str]:
    output = list(blob_types)

    if not output:
        raise ValueError("blob_types cannot be empty")

    for blob_type in output:
        if blob_type not in BF.BLOB_TYPE_TO_ID:
            raise ValueError(f"Unsupported BLOB type: {blob_type}")

    return output


# ============================================================
# Status helpers
# ============================================================

def selected_statuses(blob_types: Iterable[str]) -> dict[str, int]:
    blob_types = validate_blob_types(blob_types)
    status_map = BF.Read_Blob_Status_Map()
    return {blob_type: status_map[blob_type] for blob_type in blob_types}


def print_selected_statuses(blob_types: Iterable[str]) -> dict[str, int]:
    statuses = selected_statuses(blob_types)
    print(BF.Format_Blob_Status_Map(statuses))
    return statuses


def wait_for_selected_blob_statuses(
    blob_types: Iterable[str],
    allowed_statuses: int | set[int] | tuple[int, ...] | list[int],
    timeout_s: float = 90.0,
    poll_s: float = 0.2,
) -> dict[str, int]:
    """
    Poll index 8607 until all selected BLOB types are in allowed_statuses.

    Status meanings:
      0 = Data collection disabled
      1 = Waiting for trigger
      2 = Data provider preparing data
      3 = Data ready for BLOB transfer
    """
    blob_types = validate_blob_types(blob_types)

    if isinstance(allowed_statuses, int):
        allowed = {allowed_statuses}
    else:
        allowed = {int(status) for status in allowed_statuses}

    start_time = time.monotonic()
    last_statuses: Optional[dict[str, int]] = None

    while time.monotonic() - start_time < timeout_s:
        statuses = selected_statuses(blob_types)
        last_statuses = statuses

        pretty = {
            blob_type: f"{status} ({BF.BLOB_STATUS_TEXT.get(status, 'Unknown')})"
            for blob_type, status in statuses.items()
        }
        print(f"Selected BLOB statuses: {pretty}")

        if all(status in allowed for status in statuses.values()):
            return statuses

        time.sleep(poll_s)

    raise TimeoutError(
        f"Timeout waiting for selected BLOB statuses to be in {sorted(allowed)}. "
        f"Last statuses: {last_statuses}"
    )


# ============================================================
# BLOB packet helpers
# ============================================================

def read_blob_marker() -> tuple[Optional[int], bytes]:
    """
    Read BLOB_CH index 50 and return:
      marker, data_bytes

    marker is data byte 0.
    """
    response = BF.Read_Blob_Data()
    data = BF.response_to_bytes(response)

    if not data:
        return None, b""

    return data[0], data


def make_timestamped_csv_path(blob_type: str, output_dir: str = "blob_csv") -> str:
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = f"{blob_type}_{timestamp}.csv"

    return str(Path(output_dir) / filename)


def save_blob_to_csv(
    blob_type: str,
    payload: bytes,
    output_dir: str = "blob_csv",
    word_byte_order: str = "big",
    object_endian: str = "big",
    frequency_resolution_hz: Optional[float] = None,
    print_values: bool = True,
):
    """
    Parses one BLOB payload and saves it to a timestamped CSV.
    """
    if blob_type not in BF.BLOB_TYPE_TO_ID:
        raise ValueError(f"Unsupported blob_type for CSV parsing: {blob_type}")

    blob_id = BF.BLOB_TYPE_TO_ID[blob_type]
    csv_path = make_timestamped_csv_path(blob_type, output_dir=output_dir)

    parsed = BF.parse_balluff_blob(
        data=payload,
        blob_id=blob_id,
        word_byte_order=word_byte_order,
        object_endian=object_endian,
        csv_path=csv_path,
        frequency_resolution_hz=frequency_resolution_hz,
        print_values=print_values,
    )

    return csv_path, parsed


# ============================================================
# Recovery helpers
# ============================================================

def cleanup_active_blob_transfer(
    strategy: str = "abort",
    wait_for_write_responses: bool = False,
    quiet: bool = True,
) -> bool:
    """
    Best-effort recovery for a stuck or interrupted BLOB transfer.

    strategy:
      "abort"  -> send BLOB_Abort 0xF0
      "finish" -> send BLOB_Finish 0xF2
      "both"   -> send abort, then finish

    Returns True if all attempted cleanup commands were accepted by the Modbus layer.
    """
    strategy = strategy.lower().strip()
    if strategy not in {"abort", "finish", "both"}:
        raise ValueError("cleanup strategy must be 'abort', 'finish', or 'both'")

    commands = []
    if strategy in {"abort", "both"}:
        commands.append(("abort", BF.Write_Blob_Abort))
    if strategy in {"finish", "both"}:
        commands.append(("finish", BF.Write_Blob_Finish))

    ok = True

    for name, func in commands:
        try:
            message = func(wait_for_response=wait_for_write_responses)
            if not quiet:
                print(message)
            logger.info("BLOB cleanup command sent: %s", name)
        except Exception as exc:
            ok = False
            logger.warning("BLOB cleanup command failed: %s: %s", name, exc)
            if not quiet:
                print(f"Cleanup command {name} failed: {exc}")

    return ok


# ============================================================
# Acquisition and transfer state machines
# ============================================================

def run_acquisition_until_ready(
    blob_types: Iterable[str],
    timeout_s: float = 90.0,
    poll_s: float = 0.2,
    DPTG_value: int = 2,
    RADPTM_value: int = 0,
    DCAS_value: int = 6,
    DCT_value: int = 0,
    wait_for_write_responses: bool = False,
    restart_before_config: bool = True,
    cleanup_before_acquisition: bool = True,
) -> BlobState:
    """
    Runs the acquisition part of the Balluff BLOB flow once.

    This does:
      CONFIGURE_SENSOR
      WAIT_STATUS_1
      START_COLLECTION
      WAIT_STATUS_2_OR_3
      WAIT_STATUS_3

    After this, the selected sensor data is ready and multiple BLOBs can be transferred.
    """
    blob_types = validate_blob_types(blob_types)
    state = BlobState.CONFIGURE_SENSOR
    start_time = time.monotonic()

    print("\nStarting BLOB acquisition state machine...")
    print(f"Selected BLOB types: {blob_types}")
    print(f"DPTG_value: {DPTG_value}")
    print(f"RADPTM_value: {RADPTM_value}")
    print(f"DCAS_value: {DCAS_value}")
    print(f"DCT_value: {DCT_value}")

    if cleanup_before_acquisition:
        print("Running best-effort BLOB cleanup before acquisition...")
        cleanup_active_blob_transfer(
            strategy="abort",
            wait_for_write_responses=wait_for_write_responses,
            quiet=False,
        )

    if restart_before_config:
        try:
            print(BF.Restart_Raw_Data_Feature(wait_for_response=wait_for_write_responses))
            time.sleep(0.2)
        except Exception as exc:
            logger.warning("Raw data feature restart failed before config: %s", exc)
            print(f"Warning: raw data feature restart failed before config: {exc}")

    while state not in (BlobState.WAIT_STATUS_3, BlobState.ERROR):
        elapsed = time.monotonic() - start_time

        if elapsed > timeout_s:
            raise TimeoutError(
                f"Acquisition timeout after {timeout_s} seconds. "
                f"Last state: {state.name}"
            )

        print(f"\nCurrent acquisition state: {state.name}")

        if state == BlobState.CONFIGURE_SENSOR:
            result = BF.Sensor_Blob_Configuration(
                DPTG_value=DPTG_value,
                RADPTM_value=RADPTM_value,
                DCAS_value=DCAS_value,
                DCT_value=DCT_value,
                wait_for_response=wait_for_write_responses,
            )
            print(result)
            state = BlobState.WAIT_STATUS_1

        elif state == BlobState.WAIT_STATUS_1:
            wait_for_selected_blob_statuses(
                blob_types=blob_types,
                allowed_statuses=1,
                timeout_s=timeout_s,
                poll_s=poll_s,
            )
            state = BlobState.START_COLLECTION

        elif state == BlobState.START_COLLECTION:
            result = BF.Trigger_Start_Collection(
                wait_for_response=wait_for_write_responses,
            )
            print(result)
            state = BlobState.WAIT_STATUS_2_OR_3

        elif state == BlobState.WAIT_STATUS_2_OR_3:
            # Some captures can complete so quickly that polling misses status 2.
            wait_for_selected_blob_statuses(
                blob_types=blob_types,
                allowed_statuses={2, 3},
                timeout_s=timeout_s,
                poll_s=poll_s,
            )
            state = BlobState.WAIT_STATUS_3

    print(f"\nCurrent acquisition state: {state.name}")

    wait_for_selected_blob_statuses(
        blob_types=blob_types,
        allowed_statuses=3,
        timeout_s=timeout_s,
        poll_s=poll_s,
    )

    print("\nSensor data is ready for BLOB transfer.")
    return BlobState.WAIT_STATUS_3


def collect_single_blob_transfer(
    blob_type: str,
    timeout_s: float = 60.0,
    packet_poll_s: float = 0.05,
    wait_for_write_responses: bool = False,
    include_0x30_payload: bool = True,
    cleanup_on_error: bool = True,
    cleanup_strategy_on_error: str = "abort",
) -> BlobStateMachineResult:
    """
    Transfers one BLOB from already-ready sensor data.

    This assumes the selected data provider status is already 3.

    Flow:
      SEND_BLOB_START
      READ_BLOB_PACKETS until 0x30
      WAIT_BLOB_END_40
      SEND_BLOB_FINISH

    Marker handling:
      0x10 = BLOB start accepted
      0x2n = data packet with 4-bit packet counter n
      0x30 = last data packet marker
      0x40 = BLOB transfer ready to finish
    """
    if blob_type not in BF.BLOB_TYPE_TO_ID:
        raise ValueError(f"Unsupported BLOB type: {blob_type}")

    state = BlobState.SEND_BLOB_START
    payload = bytearray()
    expected_counter = 0
    packet_count = 0
    start_time = time.monotonic()

    print(f"\nStarting transfer for BLOB type: {blob_type}")

    try:
        while state not in (BlobState.DONE, BlobState.ERROR):
            elapsed = time.monotonic() - start_time

            if elapsed > timeout_s:
                raise TimeoutError(
                    f"BLOB transfer timeout after {timeout_s} seconds. "
                    f"BLOB type: {blob_type}. Last state: {state.name}"
                )

            print(f"\nCurrent transfer state for {blob_type}: {state.name}")

            if state == BlobState.SEND_BLOB_START:
                result = BF.Get_Blob_Data_cmd(
                    Type=blob_type,
                    wait_for_response=wait_for_write_responses,
                )
                print(result)
                state = BlobState.READ_BLOB_PACKETS
                time.sleep(packet_poll_s)

            elif state == BlobState.READ_BLOB_PACKETS:
                marker, data = read_blob_marker()

                if marker is None:
                    time.sleep(packet_poll_s)
                    continue

                print(
                    f"{blob_type} marker = 0x{marker:02X}, "
                    f"packet length = {len(data)} bytes"
                )

                if marker == 0x10:
                    # Start accepted. Keep reading; no payload in this packet.
                    time.sleep(packet_poll_s)
                    continue

                if (marker & 0xF0) == 0x20:
                    counter = marker & 0x0F

                    if counter != expected_counter:
                        print(
                            f"Warning for {blob_type}: packet counter mismatch. "
                            f"Expected {expected_counter}, got {counter}."
                        )

                    expected_counter = (counter + 1) % 16
                    packet_count += 1
                    payload.extend(data[1:])
                    time.sleep(packet_poll_s)
                    continue

                if marker == 0x30:
                    print(f"{blob_type}: received final data marker 0x30.")

                    if include_0x30_payload and len(data) > 1:
                        payload.extend(data[1:])
                        packet_count += 1

                    state = BlobState.WAIT_BLOB_END_40
                    time.sleep(packet_poll_s)
                    continue

                if marker == 0x40:
                    print(f"{blob_type}: received 0x40. Ready to send BLOB_Finish.")
                    state = BlobState.SEND_BLOB_FINISH
                    continue

                print(f"{blob_type}: unexpected marker while reading packets: 0x{marker:02X}")
                time.sleep(packet_poll_s)

            elif state == BlobState.WAIT_BLOB_END_40:
                marker, data = read_blob_marker()

                if marker is None:
                    time.sleep(packet_poll_s)
                    continue

                print(
                    f"{blob_type} end-wait marker = 0x{marker:02X}, "
                    f"packet length = {len(data)} bytes"
                )

                if marker == 0x40:
                    state = BlobState.SEND_BLOB_FINISH
                    continue

                # Defensive handling: if the device still has late data packets,
                # do not discard them.
                if (marker & 0xF0) == 0x20:
                    counter = marker & 0x0F

                    if counter != expected_counter:
                        print(
                            f"Warning for {blob_type}: late packet counter mismatch. "
                            f"Expected {expected_counter}, got {counter}."
                        )

                    expected_counter = (counter + 1) % 16
                    packet_count += 1
                    payload.extend(data[1:])
                    time.sleep(packet_poll_s)
                    continue

                if marker == 0x30:
                    print(f"{blob_type}: repeated 0x30 while waiting for 0x40.")
                    if include_0x30_payload and len(data) > 1:
                        payload.extend(data[1:])
                        packet_count += 1
                    time.sleep(packet_poll_s)
                    continue

                print(f"{blob_type}: unexpected marker while waiting for 0x40: 0x{marker:02X}")
                time.sleep(packet_poll_s)

            elif state == BlobState.SEND_BLOB_FINISH:
                result = BF.Write_Blob_Finish(
                    wait_for_response=wait_for_write_responses,
                )
                print(result)
                state = BlobState.DONE

        print(f"\n{blob_type} transfer completed.")
        print(f"{blob_type} packet count: {packet_count}")
        print(f"{blob_type} payload bytes collected: {len(payload)}")

        return BlobStateMachineResult(
            success=True,
            blob_type=blob_type,
            payload=bytes(payload),
            packet_count=packet_count,
            final_state=state,
            error=None,
            csv_path=None,
        )

    except Exception as e:
        error_message = str(e)
        print(f"\n{blob_type} transfer failed: {error_message}")

        if cleanup_on_error:
            print(f"{blob_type}: running best-effort transfer cleanup using {cleanup_strategy_on_error}.")
            cleanup_active_blob_transfer(
                strategy=cleanup_strategy_on_error,
                wait_for_write_responses=wait_for_write_responses,
                quiet=False,
            )

        return BlobStateMachineResult(
            success=False,
            blob_type=blob_type,
            payload=bytes(payload),
            packet_count=packet_count,
            final_state=BlobState.ERROR,
            error=error_message,
            csv_path=None,
        )


def collect_ready_blobs_multi(
    blob_types: Iterable[str],
    timeout_s: float = 120.0,
    packet_poll_s: float = 0.05,
    save_csv: bool = True,
    output_dir: str = "blob_csv",
    word_byte_order: str = "big",
    object_endian: str = "big",
    frequency_resolution_hz: Optional[float] = None,
    wait_for_write_responses: bool = False,
    verify_ready_status: bool = True,
    continue_on_blob_error: bool = False,
    cleanup_on_error: bool = True,
) -> MultiBlobStateMachineResult:
    """
    Transfer multiple BLOBs from an already-collected dataset.

    Use this when you already triggered once and only want to fetch rawY/rawZ/etc.
    without running the full acquisition/configuration flow again.
    """
    blob_types = validate_blob_types(blob_types)
    results: dict[str, BlobStateMachineResult] = {}

    try:
        if verify_ready_status:
            wait_for_selected_blob_statuses(
                blob_types=blob_types,
                allowed_statuses=3,
                timeout_s=timeout_s,
                poll_s=0.2,
            )

        for blob_type in blob_types:
            transfer_result = collect_single_blob_transfer(
                blob_type=blob_type,
                timeout_s=timeout_s,
                packet_poll_s=packet_poll_s,
                wait_for_write_responses=wait_for_write_responses,
                cleanup_on_error=cleanup_on_error,
            )

            if not transfer_result.success:
                results[blob_type] = transfer_result

                if continue_on_blob_error:
                    print(f"{blob_type} failed, continuing because continue_on_blob_error=True.")
                    continue

                return MultiBlobStateMachineResult(
                    success=False,
                    results=results,
                    final_state=BlobState.ERROR,
                    error=transfer_result.error,
                )

            if save_csv:
                csv_path, _parsed = save_blob_to_csv(
                    blob_type=blob_type,
                    payload=transfer_result.payload,
                    output_dir=output_dir,
                    word_byte_order=word_byte_order,
                    object_endian=object_endian,
                    frequency_resolution_hz=frequency_resolution_hz,
                )

                transfer_result.csv_path = csv_path
                print(f"\n{blob_type} CSV saved to: {csv_path}")

            results[blob_type] = transfer_result

            # Small delay between BLOB transfers.
            time.sleep(0.2)

        failed = {name: result.error for name, result in results.items() if not result.success}
        if failed:
            return MultiBlobStateMachineResult(
                success=False,
                results=results,
                final_state=BlobState.ERROR,
                error=f"One or more BLOB transfers failed: {failed}",
            )

        return MultiBlobStateMachineResult(
            success=True,
            results=results,
            final_state=BlobState.DONE,
            error=None,
        )

    except Exception as e:
        error_message = str(e)
        print("\nMulti-BLOB ready-data transfer failed.")
        print(f"Error: {error_message}")

        return MultiBlobStateMachineResult(
            success=False,
            results=results,
            final_state=BlobState.ERROR,
            error=error_message,
        )


def run_blob_state_machine_multi(
    blob_types: Optional[Iterable[str]] = None,
    timeout_s: float = 120.0,
    poll_s: float = 0.2,
    packet_poll_s: float = 0.05,
    DPTG_value: int = 2,
    RADPTM_value: int = 0,
    DCAS_value: int = 6,
    DCT_value: int = 0,
    save_csv: bool = True,
    output_dir: str = "blob_csv",
    word_byte_order: str = "big",
    object_endian: str = "big",
    frequency_resolution_hz: Optional[float] = None,
    wait_for_write_responses: bool = False,
    restart_before_config: bool = True,
    cleanup_before_acquisition: bool = True,
    continue_on_blob_error: bool = False,
) -> MultiBlobStateMachineResult:
    """
    Flexible multi-BLOB state machine.

    It configures and triggers the sensor once, then transfers all requested BLOBs
    from that same captured dataset.

    If blob_types is None, BLOB types are chosen automatically from DCAS_value and DCT_value.

    Examples:
      DCAS_value=6, DCT_value=0:
        rawX, rawY, rawZ

      DCAS_value=6, DCT_value=1:
        AmpSpecX, EnvSpecX, AmpSpecY, EnvSpecY, AmpSpecZ, EnvSpecZ

      DCAS_value=6, DCT_value=2:
        rawX, rawY, rawZ,
        AmpSpecX, EnvSpecX,
        AmpSpecY, EnvSpecY,
        AmpSpecZ, EnvSpecZ
    """
    if blob_types is None:
        blob_types = get_blob_types_from_config(
            DCAS_value=DCAS_value,
            DCT_value=DCT_value,
        )

    blob_types = validate_blob_types(blob_types)

    print("\nStarting flexible multi-BLOB state machine...")
    print(f"Selected BLOB types: {blob_types}")

    try:
        run_acquisition_until_ready(
            blob_types=blob_types,
            timeout_s=timeout_s,
            poll_s=poll_s,
            DPTG_value=DPTG_value,
            RADPTM_value=RADPTM_value,
            DCAS_value=DCAS_value,
            DCT_value=DCT_value,
            wait_for_write_responses=wait_for_write_responses,
            restart_before_config=restart_before_config,
            cleanup_before_acquisition=cleanup_before_acquisition,
        )

        result = collect_ready_blobs_multi(
            blob_types=blob_types,
            timeout_s=timeout_s,
            packet_poll_s=packet_poll_s,
            save_csv=save_csv,
            output_dir=output_dir,
            word_byte_order=word_byte_order,
            object_endian=object_endian,
            frequency_resolution_hz=frequency_resolution_hz,
            wait_for_write_responses=wait_for_write_responses,
            verify_ready_status=False,  # acquisition already confirmed status 3
            continue_on_blob_error=continue_on_blob_error,
            cleanup_on_error=True,
        )

        if result.success:
            print("\nFlexible multi-BLOB state machine completed successfully.")

        return result

    except Exception as e:
        error_message = str(e)

        print("\nFlexible multi-BLOB state machine failed.")
        print(f"Error: {error_message}")

        return MultiBlobStateMachineResult(
            success=False,
            results={},
            final_state=BlobState.ERROR,
            error=error_message,
        )


def run_blob_state_machine(
    blob_type: str = "rawX",
    timeout_s: float = 90.0,
    poll_s: float = 0.2,
    packet_poll_s: float = 0.05,
    DPTG_value: int = 2,
    RADPTM_value: int = 0,
    DCAS_value: int = 6,
    DCT_value: int = 0,
    save_csv: bool = False,
    output_dir: str = "blob_csv",
    restart_before_config: bool = True,
    cleanup_before_acquisition: bool = True,
) -> BlobStateMachineResult:
    """
    Backward-compatible single-BLOB wrapper.

    Example:
      result = run_blob_state_machine(blob_type="rawX")
    """
    multi_result = run_blob_state_machine_multi(
        blob_types=[blob_type],
        timeout_s=timeout_s,
        poll_s=poll_s,
        packet_poll_s=packet_poll_s,
        DPTG_value=DPTG_value,
        RADPTM_value=RADPTM_value,
        DCAS_value=DCAS_value,
        DCT_value=DCT_value,
        save_csv=save_csv,
        output_dir=output_dir,
        restart_before_config=restart_before_config,
        cleanup_before_acquisition=cleanup_before_acquisition,
    )

    if not multi_result.success:
        return BlobStateMachineResult(
            success=False,
            blob_type=blob_type,
            payload=b"",
            packet_count=0,
            final_state=multi_result.final_state,
            error=multi_result.error,
            csv_path=None,
        )

    return multi_result.results[blob_type]


if __name__ == "__main__":

    # -----------------------------------------
    # Main configuration
    # -----------------------------------------

    DPTG_value = 2       # 2 = Data provider is triggered by ISDU
    RADPTM_value = 0     # 0 = Raw acceleration starts after trigger
    DCAS_value = 6       # 6 = X, Y, Z axes

    # DCT_value options:
    #   0 = raw acceleration data only
    #   1 = amplitude and envelope spectrum data only
    #   2 = raw acceleration + amplitude/envelope spectrum data
    DCT_value = 2

    # Leave this as None to auto-select BLOBs from DCAS_value and DCT_value.
    # Or manually set it, for example:
    # selected_blob_types = ["rawX", "rawY", "rawZ"]
    # selected_blob_types = ["AmpSpecX", "EnvSpecX"]
    selected_blob_types = None

    result = run_blob_state_machine_multi(
        blob_types=selected_blob_types,
        timeout_s=120.0,
        poll_s=0.2,
        packet_poll_s=0.05,

        DPTG_value=DPTG_value,
        RADPTM_value=RADPTM_value,
        DCAS_value=DCAS_value,
        DCT_value=DCT_value,

        save_csv=True,
        output_dir="blob_csv",

        word_byte_order="big",
        object_endian="big",

        # Set this if you know the spectrum frequency resolution.
        # Example: frequency_resolution_hz=1.5625
        frequency_resolution_hz=None,

        # Keep False unless your ICE write-response polling is stable.
        wait_for_write_responses=False,
    )

    if result.success:
        print("\nAll requested BLOB transfers completed.")

        for blob_type, transfer in result.results.items():
            print("\n-----------------------------------")
            print(f"BLOB type: {blob_type}")
            print(f"BLOB ID: {BF.BLOB_TYPE_TO_ID[blob_type]}")
            print(f"Payload bytes: {len(transfer.payload)}")
            print(f"Packets: {transfer.packet_count}")
            print(f"CSV path: {transfer.csv_path}")
            print("-----------------------------------")

    else:
        print("\nState machine did not complete.")
        print(f"Final state: {result.final_state.name}")
        print(f"Error: {result.error}")

        if result.results:
            print("\nPartial successful transfers:")

            for blob_type, transfer in result.results.items():
                print(f"{blob_type}: {len(transfer.payload)} bytes, CSV: {transfer.csv_path}")
