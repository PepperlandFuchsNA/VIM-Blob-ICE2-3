# blob_state_machine.py
from __future__ import annotations

from dataclasses import dataclass, field
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
class BlobTransferDiagnostics:
    """Engineering diagnostics for one BLOB transfer."""
    expected_payload_length: Optional[int] = None
    actual_payload_length: int = 0
    saw_info_packet: bool = False
    saw_final_packet: bool = False
    saw_crc_packet: bool = False
    info_packet_hex: Optional[str] = None
    crc_packet_hex: Optional[str] = None
    crc_bytes: Optional[bytes] = None
    markers_seen: list[str] = field(default_factory=list)
    counter_mismatches: list[str] = field(default_factory=list)
    validation_warnings: list[str] = field(default_factory=list)

    @property
    def has_counter_mismatch(self) -> bool:
        return bool(self.counter_mismatches)

    @property
    def has_validation_warnings(self) -> bool:
        return bool(self.validation_warnings)


@dataclass
class BlobStateMachineResult:
    success: bool
    blob_type: str
    payload: bytes
    packet_count: int
    final_state: BlobState
    error: Optional[str] = None
    csv_path: Optional[str] = None
    diagnostics: BlobTransferDiagnostics = field(default_factory=BlobTransferDiagnostics)


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


def _packet_hex(data: bytes, limit: int = 32) -> str:
    """Return compact packet hex for logs without dumping huge payloads."""
    if len(data) <= limit:
        return data.hex(" ").upper()
    return f"{data[:limit].hex(' ').upper()} ... ({len(data)} bytes)"


def _extract_expected_length_from_info_packet(
    data: bytes,
    payload_length_offset: Optional[int] = None,
    payload_length_size: int = 4,
    byteorder: str = "big",
) -> Optional[int]:
    """
    Extract expected BLOB payload length from a 0x10 info packet.

    The Balluff workflow describes 0x10 as the info packet containing the total
    length. The exact byte offset can vary by documentation/firmware, so an
    explicit offset is supported. Without one, this uses a conservative
    auto-detect fallback and only accepts plausible values.
    """
    if not data or data[0] != 0x10:
        return None

    if payload_length_size not in (2, 4):
        raise ValueError("payload_length_size must be 2 or 4")

    if payload_length_offset is not None:
        start = int(payload_length_offset)
        end = start + int(payload_length_size)
        if start < 1 or end > len(data):
            raise ValueError(
                f"Invalid BLOB info length field offset/size: "
                f"offset={payload_length_offset}, size={payload_length_size}, "
                f"packet_length={len(data)}"
            )
        return int.from_bytes(data[start:end], byteorder=byteorder, signed=False)

    candidates: list[int] = []
    for size in (4, 2):
        if len(data) < 1 + size:
            continue
        offsets = list(range(1, min(len(data) - size + 1, 9)))
        tail_offset = len(data) - size
        if tail_offset >= 1 and tail_offset not in offsets:
            offsets.append(tail_offset)
        for offset in offsets:
            value = int.from_bytes(data[offset:offset + size], byteorder=byteorder, signed=False)
            if 4 <= value <= 10_000_000:
                candidates.append(value)

    if not candidates:
        return None

    # Prefer the largest plausible value because metadata fields are often small.
    return max(candidates)


def _trim_payload_to_expected_length(payload: bytearray, expected_length: Optional[int]) -> None:
    if expected_length is not None and len(payload) > expected_length:
        del payload[expected_length:]


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
    force_reconfigure: bool = False,
    reuse_ready_data: bool = True,
) -> BlobState:
    """
    Runs the acquisition part of the Balluff BLOB flow.

    Status-aware behavior:
      status 0 -> configure, wait for 1, trigger, wait for 3
      status 1 -> already configured, trigger, wait for 3
      status 2 -> already collecting/preparing, wait for 3
      status 3 -> data is already ready; either reuse it or reconfigure for a new capture

    After this returns WAIT_STATUS_3, one or more BLOB transfers can be performed
    by selecting BLOB IDs through index 50.
    """
    blob_types = validate_blob_types(blob_types)
    start_time = time.monotonic()

    print("\nStarting BLOB acquisition state machine...")
    print(f"Selected BLOB types: {blob_types}")
    print(f"DPTG_value: {DPTG_value}")
    print(f"RADPTM_value: {RADPTM_value}")
    print(f"DCAS_value: {DCAS_value}")
    print(f"DCT_value: {DCT_value}")
    print(f"force_reconfigure: {force_reconfigure}")
    print(f"reuse_ready_data: {reuse_ready_data}")

    def remaining_timeout() -> float:
        remaining = timeout_s - (time.monotonic() - start_time)
        if remaining <= 0:
            raise TimeoutError(
                f"Acquisition timeout after {timeout_s} seconds."
            )
        return remaining

    def print_status_snapshot(label: str) -> dict[str, int]:
        statuses = selected_statuses(blob_types)
        pretty = {
            blob_type: f"{status} ({BF.BLOB_STATUS_TEXT.get(status, 'Unknown')})"
            for blob_type, status in statuses.items()
        }
        print(f"{label} selected BLOB statuses: {pretty}")
        return statuses

    def run_cleanup_if_enabled() -> None:
        if cleanup_before_acquisition:
            print("Running best-effort BLOB transfer cleanup before acquisition...")
            cleanup_active_blob_transfer(
                strategy="abort",
                wait_for_write_responses=wait_for_write_responses,
                quiet=False,
            )

    def run_restart_if_enabled() -> None:
        if restart_before_config:
            try:
                print(BF.Restart_Raw_Data_Feature(wait_for_response=wait_for_write_responses))
                time.sleep(0.2)
            except Exception as exc:
                logger.warning("Raw data feature restart failed before config: %s", exc)
                print(f"Warning: raw data feature restart failed before config: {exc}")

    def configure_sensor() -> None:
        print(f"\nCurrent acquisition state: {BlobState.CONFIGURE_SENSOR.name}")
        result = BF.Sensor_Blob_Configuration(
            DPTG_value=DPTG_value,
            RADPTM_value=RADPTM_value,
            DCAS_value=DCAS_value,
            DCT_value=DCT_value,
            wait_for_response=wait_for_write_responses,
        )
        print(result)

        print(f"\nCurrent acquisition state: {BlobState.WAIT_STATUS_1.name}")
        wait_for_selected_blob_statuses(
            blob_types=blob_types,
            allowed_statuses=1,
            timeout_s=remaining_timeout(),
            poll_s=poll_s,
        )

    def start_collection_and_wait_ready() -> BlobState:
        print(f"\nCurrent acquisition state: {BlobState.START_COLLECTION.name}")
        result = BF.Trigger_Start_Collection(
            wait_for_response=wait_for_write_responses,
        )
        print(result)

        print(f"\nCurrent acquisition state: {BlobState.WAIT_STATUS_2_OR_3.name}")
        # Some captures complete so quickly that polling misses status 2.
        wait_for_selected_blob_statuses(
            blob_types=blob_types,
            allowed_statuses={2, 3},
            timeout_s=remaining_timeout(),
            poll_s=poll_s,
        )

        print(f"\nCurrent acquisition state: {BlobState.WAIT_STATUS_3.name}")
        wait_for_selected_blob_statuses(
            blob_types=blob_types,
            allowed_statuses=3,
            timeout_s=remaining_timeout(),
            poll_s=poll_s,
        )

        print("\nSensor data is ready for BLOB transfer.")
        return BlobState.WAIT_STATUS_3

    initial_statuses = print_status_snapshot("Initial")
    initial_values = set(initial_statuses.values())

    # Status 3 means a dataset is already captured and available.
    # Do not restart/reconfigure unless the caller explicitly wants a new capture.
    if initial_values == {3} and reuse_ready_data and not force_reconfigure:
        print(
            "\nSelected BLOB data is already ready for transfer. "
            "Skipping configuration and trigger."
        )
        return BlobState.WAIT_STATUS_3

    # Status 2 means a capture is already in progress.
    if initial_values == {2} and not force_reconfigure:
        print("\nSelected BLOB data is already collecting/preparing. Waiting for status 3.")
        print(f"\nCurrent acquisition state: {BlobState.WAIT_STATUS_3.name}")
        wait_for_selected_blob_statuses(
            blob_types=blob_types,
            allowed_statuses=3,
            timeout_s=remaining_timeout(),
            poll_s=poll_s,
        )
        print("\nSensor data is ready for BLOB transfer.")
        return BlobState.WAIT_STATUS_3

    # Status 1 means the sensor is already configured and waiting for trigger.
    if initial_values == {1} and not force_reconfigure:
        print("\nSensor is already configured and waiting for trigger.")
        run_cleanup_if_enabled()
        return start_collection_and_wait_ready()

    # Status 0, mixed states, forced reconfigure, or ready-data-with-new-capture request.
    if force_reconfigure:
        print("\nforce_reconfigure=True. Reconfiguring sensor before acquisition.")
    elif initial_values == {0}:
        print("\nBLOB function is inactive for selected data. Configuring sensor.")
    elif initial_values == {3} and not reuse_ready_data:
        print("\nData is already ready, but reuse_ready_data=False. Starting a new configured capture.")
    else:
        print(
            "\nSelected BLOB statuses are mixed or not directly reusable. "
            "Reconfiguring sensor to get a clean acquisition state."
        )

    run_cleanup_if_enabled()
    run_restart_if_enabled()
    configure_sensor()
    return start_collection_and_wait_ready()

def collect_single_blob_transfer(
    blob_type: str,
    timeout_s: float = 60.0,
    packet_poll_s: float = 0.05,
    wait_for_write_responses: bool = False,
    include_0x30_payload: bool = True,
    cleanup_on_error: bool = True,
    cleanup_strategy_on_error: str = "abort",
    strict_transfer: bool = True,
    validate_packet_counter: bool = True,
    validate_expected_length: bool = True,
    blob_info_payload_length_offset: Optional[int] = None,
    blob_info_payload_length_size: int = 4,
    expected_payload_length: Optional[int] = None,
    trim_to_expected_length: bool = True,
) -> BlobStateMachineResult:
    """
    Transfers one BLOB from already-ready sensor data.

    Production-oriented behavior:
      - records 0x10 info packet diagnostics
      - tracks every marker seen
      - validates 0x2n packet counter sequence in strict mode
      - prevents repeated 0x30 from appending duplicate final payload
      - records 0x40 CRC/end packet bytes for traceability
      - validates payload length when the expected length is known
    """
    if blob_type not in BF.BLOB_TYPE_TO_ID:
        raise ValueError(f"Unsupported BLOB type: {blob_type}")

    state = BlobState.SEND_BLOB_START
    payload = bytearray()
    expected_counter = 0
    packet_count = 0
    start_time = time.monotonic()
    final_packet_consumed = False

    diagnostics = BlobTransferDiagnostics(expected_payload_length=expected_payload_length)

    print(f"\nStarting transfer for BLOB type: {blob_type}")

    def record_marker(marker: int, data: bytes) -> None:
        diagnostics.markers_seen.append(f"0x{marker:02X}")
        logger.debug(
            "%s marker=0x%02X length=%s packet=%s",
            blob_type,
            marker,
            len(data),
            _packet_hex(data),
        )

    def add_counter_mismatch(message: str) -> None:
        diagnostics.counter_mismatches.append(message)
        print(f"Warning for {blob_type}: {message}")
        logger.warning("%s: %s", blob_type, message)
        if strict_transfer and validate_packet_counter:
            raise RuntimeError(message)

    def add_warning(message: str) -> None:
        diagnostics.validation_warnings.append(message)
        print(f"Warning for {blob_type}: {message}")
        logger.warning("%s: %s", blob_type, message)

    def length_is_trusted() -> bool:
        # A length supplied by the caller or parsed from a documented explicit
        # offset is trusted. Auto-detected length is recorded for diagnostics
        # only, because the exact 0x10 layout may differ by sensor firmware.
        return expected_payload_length is not None or blob_info_payload_length_offset is not None

    def append_payload(packet_payload: bytes) -> None:
        payload.extend(packet_payload)
        if (
            trim_to_expected_length
            and diagnostics.expected_payload_length is not None
            and length_is_trusted()
        ):
            _trim_payload_to_expected_length(payload, diagnostics.expected_payload_length)

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

                record_marker(marker, data)
                print(
                    f"{blob_type} marker = 0x{marker:02X}, "
                    f"packet length = {len(data)} bytes"
                )

                if marker == 0x10:
                    diagnostics.saw_info_packet = True
                    diagnostics.info_packet_hex = _packet_hex(data)

                    if diagnostics.expected_payload_length is None:
                        try:
                            diagnostics.expected_payload_length = _extract_expected_length_from_info_packet(
                                data=data,
                                payload_length_offset=blob_info_payload_length_offset,
                                payload_length_size=blob_info_payload_length_size,
                            )
                        except Exception as exc:
                            add_warning(f"Could not parse 0x10 BLOB info length: {exc}")

                    if diagnostics.expected_payload_length is not None:
                        print(
                            f"{blob_type}: expected payload length from BLOB info = "
                            f"{diagnostics.expected_payload_length} bytes"
                        )
                    else:
                        add_warning(
                            "0x10 info packet received, but expected payload length "
                            "could not be determined. Length validation will be skipped."
                        )

                    time.sleep(packet_poll_s)
                    continue

                if (marker & 0xF0) == 0x20:
                    counter = marker & 0x0F

                    if counter != expected_counter:
                        add_counter_mismatch(
                            f"packet counter mismatch. Expected {expected_counter}, got {counter}."
                        )

                    expected_counter = (counter + 1) % 16
                    packet_count += 1
                    append_payload(data[1:])
                    time.sleep(packet_poll_s)
                    continue

                if marker == 0x30:
                    diagnostics.saw_final_packet = True
                    print(f"{blob_type}: received final data marker 0x30.")

                    if not final_packet_consumed:
                        if include_0x30_payload and len(data) > 1:
                            packet_count += 1
                            append_payload(data[1:])
                        final_packet_consumed = True
                    else:
                        add_warning(
                            "Repeated 0x30 received; final payload was already consumed, "
                            "so duplicate bytes were ignored."
                        )

                    state = BlobState.WAIT_BLOB_END_40
                    time.sleep(packet_poll_s)
                    continue

                if marker == 0x40:
                    diagnostics.saw_crc_packet = True
                    diagnostics.crc_packet_hex = _packet_hex(data)
                    diagnostics.crc_bytes = data[1:] if len(data) > 1 else b""
                    print(f"{blob_type}: received 0x40. Ready to send BLOB_Finish.")
                    state = BlobState.SEND_BLOB_FINISH
                    continue

                message = f"unexpected marker while reading packets: 0x{marker:02X}"
                if strict_transfer:
                    raise RuntimeError(message)
                add_warning(message)
                time.sleep(packet_poll_s)

            elif state == BlobState.WAIT_BLOB_END_40:
                marker, data = read_blob_marker()

                if marker is None:
                    time.sleep(packet_poll_s)
                    continue

                record_marker(marker, data)
                print(
                    f"{blob_type} end-wait marker = 0x{marker:02X}, "
                    f"packet length = {len(data)} bytes"
                )

                if marker == 0x40:
                    diagnostics.saw_crc_packet = True
                    diagnostics.crc_packet_hex = _packet_hex(data)
                    diagnostics.crc_bytes = data[1:] if len(data) > 1 else b""
                    state = BlobState.SEND_BLOB_FINISH
                    continue

                if (marker & 0xF0) == 0x20:
                    counter = marker & 0x0F

                    if counter != expected_counter:
                        add_counter_mismatch(
                            f"late packet counter mismatch. Expected {expected_counter}, got {counter}."
                        )

                    expected_counter = (counter + 1) % 16
                    packet_count += 1
                    append_payload(data[1:])
                    time.sleep(packet_poll_s)
                    continue

                if marker == 0x30:
                    diagnostics.saw_final_packet = True
                    if final_packet_consumed:
                        add_warning(
                            "Repeated 0x30 while waiting for 0x40; duplicate final payload ignored."
                        )
                    else:
                        print(f"{blob_type}: late 0x30 received while waiting for 0x40.")
                        if include_0x30_payload and len(data) > 1:
                            packet_count += 1
                            append_payload(data[1:])
                        final_packet_consumed = True
                    time.sleep(packet_poll_s)
                    continue

                message = f"unexpected marker while waiting for 0x40: 0x{marker:02X}"
                if strict_transfer:
                    raise RuntimeError(message)
                add_warning(message)
                time.sleep(packet_poll_s)

            elif state == BlobState.SEND_BLOB_FINISH:
                if (
                    validate_expected_length
                    and diagnostics.expected_payload_length is not None
                    and length_is_trusted()
                ):
                    if len(payload) != diagnostics.expected_payload_length:
                        raise RuntimeError(
                            f"Payload length mismatch for {blob_type}. "
                            f"Expected {diagnostics.expected_payload_length} bytes, got {len(payload)} bytes."
                        )

                if strict_transfer and not diagnostics.saw_final_packet:
                    raise RuntimeError(f"Final data marker 0x30 was not seen for {blob_type}.")

                if strict_transfer and not diagnostics.saw_crc_packet:
                    raise RuntimeError(f"CRC/end marker 0x40 was not seen for {blob_type}.")

                result = BF.Write_Blob_Finish(
                    wait_for_response=wait_for_write_responses,
                )
                print(result)
                state = BlobState.DONE

        diagnostics.actual_payload_length = len(payload)

        print(f"\n{blob_type} transfer completed.")
        print(f"{blob_type} packet count: {packet_count}")
        print(f"{blob_type} payload bytes collected: {len(payload)}")
        print(f"{blob_type} markers seen: {diagnostics.markers_seen}")
        if diagnostics.crc_packet_hex:
            print(f"{blob_type} CRC/end packet: {diagnostics.crc_packet_hex}")

        return BlobStateMachineResult(
            success=True,
            blob_type=blob_type,
            payload=bytes(payload),
            packet_count=packet_count,
            final_state=state,
            error=None,
            csv_path=None,
            diagnostics=diagnostics,
        )

    except Exception as e:
        diagnostics.actual_payload_length = len(payload)
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
            diagnostics=diagnostics,
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
    strict_transfer: bool = True,
    validate_packet_counter: bool = True,
    validate_expected_length: bool = True,
    blob_info_payload_length_offset: Optional[int] = None,
    blob_info_payload_length_size: int = 4,
    trim_to_expected_length: bool = True,
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
                strict_transfer=strict_transfer,
                validate_packet_counter=validate_packet_counter,
                validate_expected_length=validate_expected_length,
                blob_info_payload_length_offset=blob_info_payload_length_offset,
                blob_info_payload_length_size=blob_info_payload_length_size,
                trim_to_expected_length=trim_to_expected_length,
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
    force_reconfigure: bool = False,
    reuse_ready_data: bool = True,
    continue_on_blob_error: bool = False,
    cleanup_on_error: bool = True,
    strict_transfer: bool = True,
    validate_packet_counter: bool = True,
    validate_expected_length: bool = True,
    blob_info_payload_length_offset: Optional[int] = None,
    blob_info_payload_length_size: int = 4,
    trim_to_expected_length: bool = True,
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
            force_reconfigure=force_reconfigure,
            reuse_ready_data=reuse_ready_data,
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
            cleanup_on_error=cleanup_on_error,
            strict_transfer=strict_transfer,
            validate_packet_counter=validate_packet_counter,
            validate_expected_length=validate_expected_length,
            blob_info_payload_length_offset=blob_info_payload_length_offset,
            blob_info_payload_length_size=blob_info_payload_length_size,
            trim_to_expected_length=trim_to_expected_length,
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
    force_reconfigure: bool = False,
    reuse_ready_data: bool = True,
    cleanup_on_error: bool = True,
    strict_transfer: bool = True,
    validate_packet_counter: bool = True,
    validate_expected_length: bool = True,
    blob_info_payload_length_offset: Optional[int] = None,
    blob_info_payload_length_size: int = 4,
    trim_to_expected_length: bool = True,
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
        force_reconfigure=force_reconfigure,
        reuse_ready_data=reuse_ready_data,
        cleanup_on_error=cleanup_on_error,
        strict_transfer=strict_transfer,
        validate_packet_counter=validate_packet_counter,
        validate_expected_length=validate_expected_length,
        blob_info_payload_length_offset=blob_info_payload_length_offset,
        blob_info_payload_length_size=blob_info_payload_length_size,
        trim_to_expected_length=trim_to_expected_length,
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
