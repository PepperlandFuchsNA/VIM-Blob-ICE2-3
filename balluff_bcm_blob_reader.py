#!/usr/bin/env python3
"""
balluff_bcm_blob_reader.py

Read raw acceleration BLOBs from a Balluff BCM R16E-004-CI02-01,5-S4
(BCM0003) through an ICE2/3 IO-Link master using Modbus TCP.

Outputs:
    g_values_x.csv
    g_values_y.csv
    g_values_z.csv

Install:
    pip install pyModbusTCP

Run:
    python balluff_bcm_blob_reader.py

Optional:
    edit balluff_bcm_blob_reader.env next to this file, or pass CLI args.

Notes:
    - ICE2/3 Modbus mapping is kept the same as your VIM code:
        ISDU write request register = IO-Link port * 1000 + 300
        ISDU read response register = IO-Link port * 1000 + 100
    - Balluff BCM raw data provider status is read from index 8607.
    - BLOB transfer still uses BLOB_ID index 49 and BLOB_CH index 50.
    - Raw acceleration BLOB_IDs:
        X = -4096
        Y = -4097
        Z = -4098
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from pyModbusTCP.client import ModbusClient


# ---------------------------------------------------------------------------
# IO-Link / Balluff indices
# ---------------------------------------------------------------------------

# Standard IO-Link BLOB profile indices.
BLOB_ID_INDEX = 49          # 0x0031
BLOB_CH_INDEX = 50          # 0x0032

# Balluff BCM raw acceleration and spectrum provider indices.
RAW_CONFIG_INDEX = 8603     # 0x219B
ISDU_TRIGGER_INDEX = 8605   # 0x219D
RESET_FEATURE_INDEX = 8606  # 0x219E
STATUS_INDEX = 8607         # 0x219F
EST_TRANSFER_TIME_INDEX = 8608  # 0x21A0

# RAW_CONFIG_INDEX subindices.
SUB_TRIGGER_SOURCE = 1
SUB_TRIGGER_MODE = 2
SUB_AXIS_SELECTION = 3
SUB_TARGET = 4

# STATUS_INDEX subindices for raw acceleration data.
STATUS_RAW_X = 1
STATUS_RAW_Y = 2
STATUS_RAW_Z = 3

# Raw acceleration BLOB_IDs for the BCM.
BLOB_ID_RAW_X = -4096
BLOB_ID_RAW_Y = -4097
BLOB_ID_RAW_Z = -4098

# BLOB_CH functions.
BLOB_INFO = 0x1
BLOB_SEGMENT = 0x2
BLOB_LAST = 0x3
BLOB_CRC = 0x4
BLOB_COMMAND = 0xF

# BLOB_CH command subfunctions.
CMD_ABORT = 0x0
CMD_START = 0x1
CMD_FINISH = 0x2

# IO-Link BLOB CRC-32 polynomial, reversed representation.
CRC32_POLY_REVERSED = 0xEB31D82E
U32_MASK = 0xFFFFFFFF

# Balluff raw acceleration scale: INT16, LSB = 0.488 mg.
MG_PER_COUNT = 0.488
G_PER_COUNT = MG_PER_COUNT / 1000.0


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AxisInfo:
    name: str
    status_subindex: int
    blob_id: int
    csv_name: str
    raw_bin_name: str


AXES: Tuple[AxisInfo, ...] = (
    AxisInfo("x", STATUS_RAW_X, BLOB_ID_RAW_X, "g_values_x.csv", "blob_payload_x.bin"),
    AxisInfo("y", STATUS_RAW_Y, BLOB_ID_RAW_Y, "g_values_y.csv", "blob_payload_y.bin"),
    AxisInfo("z", STATUS_RAW_Z, BLOB_ID_RAW_Z, "g_values_z.csv", "blob_payload_z.bin"),
)


@dataclass
class RawAccelerationPayload:
    timestamp_ms: int
    trigger_source: int
    trigger_mode: int
    recording_time_code: int
    recording_time_s: float
    sampling_interval_ms: float
    raw_samples: List[int]
    acceleration_mg: List[float]
    acceleration_g: List[float]


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------


class ModbusError(RuntimeError):
    """Raised for Modbus-level communication failures."""


class BlobTransferError(RuntimeError):
    """Raised for BLOB protocol, length, flow-control, or CRC failures."""


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def words_to_bytes(words: Iterable[int]) -> bytes:
    """Convert 16-bit Modbus words to bytes, MSB first."""
    out = bytearray()
    for word in words:
        word &= 0xFFFF
        out.append((word >> 8) & 0xFF)
        out.append(word & 0xFF)
    return bytes(out)


def bytes_to_words(data: bytes) -> List[int]:
    """Convert bytes to 16-bit Modbus words, MSB first. Pads final odd byte."""
    padded = data + (b"\x00" if len(data) % 2 else b"")
    return [
        (padded[i] << 8) | padded[i + 1]
        for i in range(0, len(padded), 2)
    ]


def i16_from_register(value: int) -> int:
    """Interpret a 16-bit unsigned value as signed INT16."""
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def u16_from_i16(value: int) -> int:
    """Encode a signed INT16 value into an unsigned 16-bit value."""
    return value & 0xFFFF


def iolink_blob_crc32_update(data: bytes, previous_crc32: int = 1) -> int:
    """
    IO-Link BLOB CRC-32 update.

    Use previous_crc32=1 for the first block. For streaming reads, feed the
    returned CRC into the next call.
    """
    crc = (~previous_crc32) & U32_MASK

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ CRC32_POLY_REVERSED) & U32_MASK
            else:
                crc = (crc >> 1) & U32_MASK

    return (~crc) & U32_MASK


def status_text(value: int) -> str:
    return {
        0: "disabled",
        1: "waiting for trigger",
        2: "preparing data",
        3: "ready for BLOB transfer",
    }.get(value, f"unknown status {value}")


# ---------------------------------------------------------------------------
# .env support
# ---------------------------------------------------------------------------


def load_env_file(path: Path) -> Dict[str, str]:
    """Load KEY=VALUE pairs from a small .env file."""
    values: Dict[str, str] = {}

    if not path.exists():
        return values

    for line_number, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"Invalid env line {line_number} in {path}: {raw_line!r}")

        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")

    return values


def merged_environment(env_file: Optional[str]) -> Tuple[Dict[str, str], Path]:
    default_env_path = Path(__file__).with_suffix(".env")
    env_path = Path(env_file) if env_file else default_env_path

    env = load_env_file(env_path)
    env.update(os.environ)
    return env, env_path


def env_str(env: Dict[str, str], key: str, default: str) -> str:
    value = env.get(key, "")
    return value if value != "" else default


def env_int(env: Dict[str, str], key: str, default: int) -> int:
    value = env.get(key, "")
    return int(value) if value != "" else default


def env_float(env: Dict[str, str], key: str, default: float) -> float:
    value = env.get(key, "")
    return float(value) if value != "" else default


def env_bool(env: Dict[str, str], key: str, default: bool) -> bool:
    value = env.get(key, "")
    if value == "":
        return default

    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "no", "n", "off"}:
        return False

    raise ValueError(f"Invalid boolean for {key}: {value!r}")


# ---------------------------------------------------------------------------
# Balluff BCM over ICE2/3 Modbus TCP
# ---------------------------------------------------------------------------


class Ice2BalluffBcmBlobReader:
    """
    ICE2/3 Modbus TCP helper for Balluff BCM raw acceleration BLOB reads.

    Register mapping is kept from the original VIM script:
      ISDU write request register = IO-Link port * 1000 + 300
      ISDU read response register = IO-Link port * 1000 + 100
    """

    def __init__(
        self,
        host: str,
        tcp_port: int = 502,
        unit_id: int = 1,
        iol_port: int = 4,
        max_isdu_len: int = 201,
        isdu_delay_s: float = 0.2,
        timeout_s: float = 5.0,
        debug: bool = False,
    ) -> None:
        self.host = host
        self.tcp_port = tcp_port
        self.unit_id = unit_id
        self.iol_port = iol_port
        self.max_isdu_len = max_isdu_len
        self.isdu_delay_s = isdu_delay_s
        self.debug = debug

        self.write_register = iol_port * 1000 + 300
        self.read_register = iol_port * 1000 + 100

        self.client = ModbusClient(
            host=host,
            port=tcp_port,
            unit_id=unit_id,
            auto_open=True,
            auto_close=False,
            timeout=timeout_s,
        )

    def close(self) -> None:
        self.client.close()

    def _debug(self, message: str) -> None:
        if self.debug:
            print(f"[debug] {message}")

    def _read_holding(self, address: int, count: int, context: str) -> List[int]:
        response = self.client.read_holding_registers(address, count)

        if response is None:
            raise ModbusError(f"{context}: no Modbus response from register {address}")

        if len(response) < count:
            raise ModbusError(
                f"{context}: short Modbus response from register {address}: "
                f"expected {count} registers, got {len(response)}"
            )

        return response

    def _write_multiple(self, address: int, values: List[int], context: str) -> None:
        success = self.client.write_multiple_registers(address, values)
        if not success:
            raise ModbusError(
                f"{context}: failed to write Modbus registers at {address}: {values}"
            )

    def isdu_write_bytes(self, index: int, payload: bytes, subindex: int = 0) -> None:
        """
        Write bytes to an IO-Link ISDU through the ICE2/3 Modbus mapping.

        Request format used by the original code:
            [2, index, subindex, payload_length, data_words...]
        """
        request = [2, index, subindex, len(payload), *bytes_to_words(payload)]
        self._debug(f"ISDU write index={index}, sub={subindex}, payload={payload.hex(' ')}, request={request}")
        self._write_multiple(self.write_register, request, f"ISDU write index {index} subindex {subindex}")

    def isdu_read_bytes(
        self,
        index: int,
        length: int,
        subindex: int = 0,
        delay_s: Optional[float] = None,
    ) -> Tuple[bytes, List[int]]:
        """
        Read bytes from an IO-Link ISDU through the ICE2/3 Modbus mapping.

        Request format used by the original code:
            [1, index, subindex, requested_length]

        Response format used by the original code:
            [status, index, subindex, actual_length, data_words...]
        """
        request = [1, index, subindex, length]
        self._debug(f"ISDU read request index={index}, sub={subindex}, length={length}, request={request}")
        self._write_multiple(self.write_register, request, f"ISDU read request index {index} subindex {subindex}")

        time.sleep(self.isdu_delay_s if delay_s is None else delay_s)

        registers_to_read = 4 + ((length + 1) // 2)
        response = self._read_holding(
            self.read_register,
            registers_to_read,
            f"ISDU read response index {index} subindex {subindex}",
        )

        status = response[0]
        response_index = response[1]
        response_subindex = response[2]
        response_len = response[3]

        self._debug(f"ISDU read response={response}")

        if status != 0:
            raise ModbusError(
                f"ISDU read index {index} subindex {subindex} returned status {status}; "
                f"full response: {response}"
            )

        if response_index != index:
            raise ModbusError(
                f"ISDU read index mismatch: requested {index}, response {response_index}"
            )

        if response_subindex != subindex:
            raise ModbusError(
                f"ISDU read subindex mismatch for index {index}: "
                f"requested {subindex}, response {response_subindex}"
            )

        if response_len > length:
            raise ModbusError(
                f"ISDU read index {index} returned length {response_len}, "
                f"but only requested {length}"
            )

        data = words_to_bytes(response[4:])[:response_len]
        return data, response

    def write_u8(self, index: int, subindex: int, value: int) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError(f"UINT8 out of range: {value}")
        self.isdu_write_bytes(index, bytes([value]), subindex=subindex)

    def read_u8(self, index: int, subindex: int = 0) -> int:
        data, _ = self.isdu_read_bytes(index, 1, subindex=subindex)
        if len(data) < 1:
            raise ModbusError(f"ISDU index {index} subindex {subindex} returned no data")
        return data[0]

    def read_u16(self, index: int, subindex: int = 0) -> int:
        data, _ = self.isdu_read_bytes(index, 2, subindex=subindex)
        if len(data) < 2:
            raise ModbusError(f"ISDU index {index} subindex {subindex} returned less than 2 bytes")
        return int.from_bytes(data[:2], byteorder="big", signed=False)

    def read_blob_id(self) -> int:
        data, _ = self.isdu_read_bytes(BLOB_ID_INDEX, 2)
        if len(data) < 2:
            raise ModbusError("BLOB_ID read returned less than 2 bytes")
        return int.from_bytes(data[:2], byteorder="big", signed=True)

    def blob_start(self, blob_id: int) -> None:
        blob_id_u16 = u16_from_i16(blob_id)
        payload = bytes([
            (BLOB_COMMAND << 4) | CMD_START,
            (blob_id_u16 >> 8) & 0xFF,
            blob_id_u16 & 0xFF,
        ])
        print(f"Sending BLOB_Start for BLOB_ID {blob_id} ({payload.hex(' ')})")
        self.isdu_write_bytes(BLOB_CH_INDEX, payload)

    def blob_abort(self) -> None:
        payload = bytes([(BLOB_COMMAND << 4) | CMD_ABORT])
        self.isdu_write_bytes(BLOB_CH_INDEX, payload)

    def blob_finish(self) -> None:
        payload = bytes([(BLOB_COMMAND << 4) | CMD_FINISH])
        self.isdu_write_bytes(BLOB_CH_INDEX, payload)

    def read_blob_ch_once(self) -> Tuple[int, int, bytes]:
        """
        Read exactly one BLOB_CH response.

        Important: a BLOB_CH read advances the BLOB state machine, so do not
        call this twice for the same expected segment.
        """
        data, _ = self.isdu_read_bytes(BLOB_CH_INDEX, self.max_isdu_len)

        if not data:
            raise BlobTransferError("Empty BLOB_CH response")

        header = data[0]
        function = (header >> 4) & 0x0F
        subfunction = header & 0x0F
        body = data[1:]

        self._debug(
            f"BLOB_CH header=0x{header:02X}, function=0x{function:X}, "
            f"subfunction=0x{subfunction:X}, body_len={len(body)}"
        )

        return function, subfunction, body

    def configure_raw_data_provider(
        self,
        trigger_source: int = 2,
        trigger_mode: int = 0,
        axis_selection: int = 6,
        target: int = 0,
    ) -> None:
        """
        Configure Balluff raw acceleration data collection.

        Defaults follow your Balluff flowchart:
            8603:1 = 2  -> trigger by ISDU
            8603:2 = 0  -> record after trigger
            8603:3 = 6  -> collect X, Y, and Z axes
            8603:4 = 0  -> collect raw acceleration data only
        """
        print("Configuring Balluff raw acceleration provider")
        print(f"  trigger source : {trigger_source}")
        print(f"  trigger mode   : {trigger_mode}")
        print(f"  axis selection : {axis_selection}")
        print(f"  target         : {target}")

        self.write_u8(RAW_CONFIG_INDEX, SUB_TRIGGER_SOURCE, trigger_source)
        self.write_u8(RAW_CONFIG_INDEX, SUB_TRIGGER_MODE, trigger_mode)
        self.write_u8(RAW_CONFIG_INDEX, SUB_AXIS_SELECTION, axis_selection)
        self.write_u8(RAW_CONFIG_INDEX, SUB_TARGET, target)

    def reset_feature(self) -> None:
        print("Resetting raw acceleration provider feature")
        self.write_u8(RESET_FEATURE_INDEX, 0, 1)

    def send_isdu_trigger(self) -> None:
        print("Sending ISDU data trigger")
        self.write_u8(ISDU_TRIGGER_INDEX, 0, 1)

    def read_axis_status(self, axis: AxisInfo) -> int:
        return self.read_u8(STATUS_INDEX, axis.status_subindex)

    def read_all_raw_statuses(self) -> Dict[str, int]:
        return {axis.name: self.read_axis_status(axis) for axis in AXES}

    def print_all_raw_statuses(self) -> None:
        statuses = self.read_all_raw_statuses()
        text = ", ".join(
            f"{name.upper()}={value} ({status_text(value)})"
            for name, value in statuses.items()
        )
        print(f"Raw acceleration status: {text}")

    def wait_for_axis_status(
        self,
        axis: AxisInfo,
        target_status: int,
        timeout_s: float,
        poll_s: float,
        allow_already_past_ready: bool = True,
    ) -> int:
        """Wait until one axis status equals target_status."""
        start = time.monotonic()
        last_status: Optional[int] = None

        while True:
            value = self.read_axis_status(axis)

            if value != last_status:
                print(f"{axis.name.upper()} status = {value} ({status_text(value)})")
                last_status = value

            if value == target_status:
                return value

            # For the short status 2 state, the device can move from 1 to 3
            # before the PC polls it. Treat 3 as acceptable if we were waiting
            # only to observe preparing-data status.
            if allow_already_past_ready and target_status == 2 and value == 3:
                return value

            if time.monotonic() - start > timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for {axis.name.upper()} status {target_status}; "
                    f"last status was {value} ({status_text(value)})"
                )

            time.sleep(poll_s)

    def wait_for_all_axes_ready_for_trigger(self, timeout_s: float, poll_s: float) -> None:
        for axis in AXES:
            self.wait_for_axis_status(axis, 1, timeout_s, poll_s)

    def wait_for_all_axes_preparing_or_ready(self, timeout_s: float, poll_s: float) -> None:
        for axis in AXES:
            self.wait_for_axis_status(axis, 2, timeout_s, poll_s)

    def wait_for_all_axes_ready_for_blob(self, timeout_s: float, poll_s: float) -> None:
        for axis in AXES:
            self.wait_for_axis_status(axis, 3, timeout_s, poll_s)

    def read_blob_verified(self, blob_timeout_s: float = 600.0) -> Tuple[bytes, int, int]:
        """
        Read active BLOB, verify length, flow counter, and IO-Link BLOB CRC.

        Returns:
            payload bytes, local CRC, device CRC
        """
        start_time = time.monotonic()

        def check_timeout() -> None:
            if time.monotonic() - start_time > blob_timeout_s:
                raise TimeoutError(f"Timed out reading BLOB after {blob_timeout_s} seconds")

        try:
            function, subfunction, body = self.read_blob_ch_once()
            if function != BLOB_INFO or subfunction != 0:
                raise BlobTransferError(
                    f"Expected BLOB_Info_Read 0x10, got function=0x{function:X}, sub=0x{subfunction:X}"
                )
            if len(body) < 4:
                raise BlobTransferError(f"BLOB_Info_Read too short: {body.hex(' ')}")

            expected_blob_len = int.from_bytes(body[:4], byteorder="big", signed=False)
            print(f"Expected BLOB payload size: {expected_blob_len} bytes")

            payload = bytearray()
            local_crc = 1
            expected_flow = 0
            segment_count = 0

            while True:
                check_timeout()
                function, subfunction, body = self.read_blob_ch_once()

                if function == BLOB_SEGMENT:
                    if subfunction != expected_flow:
                        raise BlobTransferError(
                            f"BLOB flow counter error: expected {expected_flow}, got {subfunction}"
                        )

                    payload.extend(body)
                    local_crc = iolink_blob_crc32_update(body, local_crc)
                    segment_count += 1

                    if segment_count == 1 or segment_count % 25 == 0:
                        print(
                            f"Segment {segment_count}: flow={subfunction}, "
                            f"received={len(payload)}/{expected_blob_len}, "
                            f"crc=0x{local_crc:08X}"
                        )

                    expected_flow = (expected_flow + 1) & 0x0F

                    if len(payload) > expected_blob_len:
                        raise BlobTransferError(
                            f"Received too many BLOB bytes: {len(payload)} > {expected_blob_len}"
                        )

                elif function == BLOB_LAST:
                    if subfunction != 0:
                        raise BlobTransferError(f"BLOB_Last subfunction should be 0, got {subfunction}")

                    remaining = expected_blob_len - len(payload)
                    if remaining < 0:
                        raise BlobTransferError("BLOB_Last received after payload already exceeded expected length")

                    last_data = body[:remaining]
                    payload.extend(last_data)
                    local_crc = iolink_blob_crc32_update(last_data, local_crc)
                    print(
                        f"BLOB_Last: bytes={len(last_data)}, "
                        f"received={len(payload)}/{expected_blob_len}, "
                        f"crc=0x{local_crc:08X}"
                    )
                    break

                else:
                    raise BlobTransferError(
                        f"Expected BLOB_Segment 0x2n or BLOB_Last 0x30, "
                        f"got function=0x{function:X}, sub=0x{subfunction:X}"
                    )

            if len(payload) != expected_blob_len:
                raise BlobTransferError(
                    f"Incomplete BLOB: received {len(payload)} bytes, expected {expected_blob_len}"
                )

            check_timeout()
            function, subfunction, body = self.read_blob_ch_once()
            if function != BLOB_CRC or subfunction != 0:
                raise BlobTransferError(
                    f"Expected BLOB_CRC 0x40, got function=0x{function:X}, sub=0x{subfunction:X}"
                )
            if len(body) < 4:
                raise BlobTransferError(f"BLOB_CRC response too short: {body.hex(' ')}")

            device_crc = int.from_bytes(body[:4], byteorder="big", signed=False)

            print(f"Local CRC : 0x{local_crc:08X}")
            print(f"Device CRC: 0x{device_crc:08X}")

            if local_crc != device_crc:
                raise BlobTransferError(
                    f"BLOB CRC mismatch: local=0x{local_crc:08X}, device=0x{device_crc:08X}"
                )

            self.blob_finish()
            print("BLOB CRC check passed and BLOB_Finish sent")

            return bytes(payload), local_crc, device_crc

        except Exception:
            try:
                self.blob_abort()
                print("BLOB_Abort sent after transfer failure")
            except Exception as abort_error:
                print(f"Could not send BLOB_Abort: {abort_error}", file=sys.stderr)
            raise


# ---------------------------------------------------------------------------
# Balluff payload parser and CSV writer
# ---------------------------------------------------------------------------


def parse_balluff_raw_acceleration_payload(payload: bytes) -> RawAccelerationPayload:
    """
    Parse BCM raw acceleration payload.

    Layout:
        byte 0..3   UINT32 timestamp, ms
        byte 4      UINT8 trigger source
        byte 5      UINT8 trigger mode
        byte 6      UINT8 recording time, LSB = 0.5 s
        byte 7..10  FLOAT32 sampling interval, ms
        byte 11..   INT16 raw acceleration samples, LSB = 0.488 mg
    """
    if len(payload) < 11:
        raise ValueError(f"Raw acceleration payload too short: {len(payload)} bytes")

    timestamp_ms = int.from_bytes(payload[0:4], byteorder="big", signed=False)
    trigger_source = payload[4]
    trigger_mode = payload[5]
    recording_time_code = payload[6]
    recording_time_s = recording_time_code * 0.5
    sampling_interval_ms = struct.unpack(">f", payload[7:11])[0]

    sample_bytes = payload[11:]
    usable_len = len(sample_bytes) - (len(sample_bytes) % 2)
    if usable_len != len(sample_bytes):
        print(f"Warning: ignoring one trailing byte in raw sample data: {sample_bytes[-1]:02X}")

    raw_samples = [
        int.from_bytes(sample_bytes[i:i + 2], byteorder="big", signed=True)
        for i in range(0, usable_len, 2)
    ]

    acceleration_mg = [raw * MG_PER_COUNT for raw in raw_samples]
    acceleration_g = [raw * G_PER_COUNT for raw in raw_samples]

    return RawAccelerationPayload(
        timestamp_ms=timestamp_ms,
        trigger_source=trigger_source,
        trigger_mode=trigger_mode,
        recording_time_code=recording_time_code,
        recording_time_s=recording_time_s,
        sampling_interval_ms=sampling_interval_ms,
        raw_samples=raw_samples,
        acceleration_mg=acceleration_mg,
        acceleration_g=acceleration_g,
    )


def write_axis_csv(axis: AxisInfo, parsed: RawAccelerationPayload, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / axis.csv_name

    sample_interval_s = parsed.sampling_interval_ms / 1000.0

    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["sample_index", "time_s", "g_value", "acceleration_mg", "raw_int16"])

        for sample_index, raw_value in enumerate(parsed.raw_samples):
            writer.writerow([
                sample_index,
                sample_index * sample_interval_s,
                parsed.acceleration_g[sample_index],
                parsed.acceleration_mg[sample_index],
                raw_value,
            ])

    return path


def print_payload_summary(axis: AxisInfo, parsed: RawAccelerationPayload) -> None:
    if parsed.sampling_interval_ms > 0:
        sample_rate_hz = 1000.0 / parsed.sampling_interval_ms
    else:
        sample_rate_hz = float("nan")

    print(f"{axis.name.upper()} payload summary:")
    print(f"  timestamp_ms         : {parsed.timestamp_ms}")
    print(f"  trigger_source       : {parsed.trigger_source}")
    print(f"  trigger_mode         : {parsed.trigger_mode}")
    print(f"  recording_time_code  : {parsed.recording_time_code}")
    print(f"  recording_time_s     : {parsed.recording_time_s}")
    print(f"  sampling_interval_ms : {parsed.sampling_interval_ms}")
    print(f"  sample_rate_hz       : {sample_rate_hz}")
    print(f"  samples              : {len(parsed.raw_samples)}")


# ---------------------------------------------------------------------------
# Command-line parsing
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--env-file", default=None, help="Path to env file")
    pre_args, remaining_argv = pre_parser.parse_known_args()

    env, env_path = merged_environment(pre_args.env_file)

    parser = argparse.ArgumentParser(
        description="Read Balluff BCM0003 raw acceleration BLOBs over ICE2/3 Modbus TCP.",
        parents=[pre_parser],
    )

    parser.add_argument("--host", default=env_str(env, "BCM_HOST", "192.168.137.21"))
    parser.add_argument("--tcp-port", type=int, default=env_int(env, "BCM_TCP_PORT", 502))
    parser.add_argument("--unit-id", type=int, default=env_int(env, "BCM_UNIT_ID", 1))
    parser.add_argument("--iol-port", type=int, default=env_int(env, "BCM_IOL_PORT", 4))

    parser.add_argument("--trigger-source", type=int, default=env_int(env, "BCM_TRIGGER_SOURCE", 2))
    parser.add_argument("--trigger-mode", type=int, default=env_int(env, "BCM_TRIGGER_MODE", 0))
    parser.add_argument("--axis-selection", type=int, default=env_int(env, "BCM_AXIS_SELECTION", 6))
    parser.add_argument("--target", type=int, default=env_int(env, "BCM_TARGET", 0))

    parser.add_argument("--status-timeout", type=float, default=env_float(env, "BCM_STATUS_TIMEOUT_S", 120.0))
    parser.add_argument("--blob-timeout", type=float, default=env_float(env, "BCM_BLOB_TIMEOUT_S", 600.0))
    parser.add_argument("--poll-s", type=float, default=env_float(env, "BCM_POLL_S", 0.25))
    parser.add_argument("--isdu-delay-s", type=float, default=env_float(env, "BCM_ISDU_DELAY_S", 0.2))
    parser.add_argument("--timeout-s", type=float, default=env_float(env, "BCM_MODBUS_TIMEOUT_S", 5.0))
    parser.add_argument("--max-isdu-len", type=int, default=env_int(env, "BCM_MAX_ISDU_LEN", 201))

    parser.add_argument("--output-dir", default=env_str(env, "BCM_OUTPUT_DIR", "."))
    parser.add_argument("--save-raw-bin", action="store_true", default=env_bool(env, "BCM_SAVE_RAW_BIN", False))
    parser.add_argument("--no-initial-abort", action="store_true", default=env_bool(env, "BCM_NO_INITIAL_ABORT", False))
    parser.add_argument("--no-reset-at-end", action="store_true", default=env_bool(env, "BCM_NO_RESET_AT_END", False))
    parser.add_argument("--debug", action="store_true", default=env_bool(env, "BCM_DEBUG", False))

    args = parser.parse_args(remaining_argv)
    args.env_path = env_path
    return args


# ---------------------------------------------------------------------------
# Main sequence
# ---------------------------------------------------------------------------


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)

    print(f"Using env file: {args.env_path}")
    print(f"Connecting to ICE2/3 Modbus TCP {args.host}:{args.tcp_port}, unit_id={args.unit_id}")
    print(f"IO-Link port: {args.iol_port}")

    reader = Ice2BalluffBcmBlobReader(
        host=args.host,
        tcp_port=args.tcp_port,
        unit_id=args.unit_id,
        iol_port=args.iol_port,
        max_isdu_len=args.max_isdu_len,
        isdu_delay_s=args.isdu_delay_s,
        timeout_s=args.timeout_s,
        debug=args.debug,
    )

    try:
        try:
            current_blob_id = reader.read_blob_id()
            print(f"Initial BLOB_ID: {current_blob_id}")
        except Exception as exc:
            print(f"Could not read initial BLOB_ID: {exc}")

        if not args.no_initial_abort:
            print("Sending BLOB_Abort once to clear any old active BLOB transfer")
            try:
                reader.blob_abort()
                time.sleep(0.5)
            except Exception as exc:
                print(f"Initial BLOB_Abort failed or was not needed: {exc}")

        reader.configure_raw_data_provider(
            trigger_source=args.trigger_source,
            trigger_mode=args.trigger_mode,
            axis_selection=args.axis_selection,
            target=args.target,
        )

        print("Waiting for X/Y/Z raw acceleration status = 1 (waiting for trigger)")
        reader.wait_for_all_axes_ready_for_trigger(args.status_timeout, args.poll_s)

        reader.send_isdu_trigger()

        print("Waiting for X/Y/Z raw acceleration status = 2 (preparing) or 3 (ready)")
        reader.wait_for_all_axes_preparing_or_ready(args.status_timeout, args.poll_s)

        print("Waiting for X/Y/Z raw acceleration status = 3 (ready for BLOB transfer)")
        reader.wait_for_all_axes_ready_for_blob(args.status_timeout, args.poll_s)
        reader.print_all_raw_statuses()

        for axis in AXES:
            print("-" * 72)
            print(f"Reading {axis.name.upper()} raw acceleration BLOB")

            axis_status = reader.read_axis_status(axis)
            if axis_status != 3:
                raise RuntimeError(
                    f"{axis.name.upper()} is not ready for BLOB transfer: "
                    f"status {axis_status} ({status_text(axis_status)})"
                )

            reader.blob_start(axis.blob_id)
            payload, local_crc, device_crc = reader.read_blob_verified(args.blob_timeout)

            if args.save_raw_bin:
                output_dir.mkdir(parents=True, exist_ok=True)
                raw_path = output_dir / axis.raw_bin_name
                raw_path.write_bytes(payload)
                print(f"Saved raw payload: {raw_path.resolve()}")

            parsed = parse_balluff_raw_acceleration_payload(payload)
            print_payload_summary(axis, parsed)
            csv_path = write_axis_csv(axis, parsed, output_dir)
            print(f"Saved CSV: {csv_path.resolve()}")
            print(f"CRC verified for {axis.name.upper()}: 0x{local_crc:08X}")

        if not args.no_reset_at_end:
            print("Resetting provider feature after all three BLOB transfers")
            reader.reset_feature()

        print("Done. Created g_values_x.csv, g_values_y.csv, and g_values_z.csv")
        return 0

    except KeyboardInterrupt:
        print("Interrupted by user", file=sys.stderr)
        return 130

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
