
#!/usr/bin/env python3
"""
vim_blob_crc_reader.py

Reads raw BLOB data from a VIM sensor through an ICE2/3 IO-Link master
using Modbus TCP, verifies the IO-Link BLOB CRC-32, saves the raw payload,
and exports converted g-values to CSV.

Install:
    pip install pyModbusTCP

Example using the default env file next to this script:
    python vim_blob_crc_reader.py

Example with a specific environment file:
    python vim_blob_crc_reader.py --env-file .env

Command-line arguments still work and override values from the environment file:
    python vim_blob_crc_reader.py --host 192.168.137.21 --iol-port 4

Notes:
    - BLOB_ID index = 49
    - BLOB_CH index = 50
    - Default BLOB_ID = -4096, encoded as 0xF000
    - Default max BLOB_CH read length = 201 bytes
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path
from typing import List, Tuple

try:
    from pyModbusTCP.client import ModbusClient
except ModuleNotFoundError:
    ModbusClient = None  # type: ignore[assignment]


# IO-Link BLOB profile indices.
BLOB_ID_INDEX = 49
BLOB_CH_INDEX = 50

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


class ModbusError(RuntimeError):
    """Raised for Modbus-level communication failures."""


class BlobTransferError(RuntimeError):
    """Raised for BLOB protocol, length, flow-control, or CRC failures."""


def words_to_bytes(words: List[int]) -> bytes:
    """Convert 16-bit Modbus words to bytes, MSB first."""
    out = bytearray()
    for word in words:
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


def i16_from_u16(value: int) -> int:
    """Interpret a 16-bit unsigned register as signed INT16."""
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def iolink_blob_crc32_update(data: bytes, previous_crc32: int = 1) -> int:
    """
    IO-Link BLOB CRC-32 update.

    For a complete BLOB, call this once with previous_crc32=1.
    For streaming segments, feed the returned CRC into the next call.
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


def parse_g_values_from_payload(
    payload: bytes,
    scale: float = 1969.3568,
    offset: float = 50.0,
) -> List[float]:
    """
    Convert the verified BLOB payload to g-values.

    This follows the original script's interpretation:
      - 4 payload bytes per raw sample
      - unsigned big-endian raw value
      - g = raw / 1969.3568 - 50
    """
    usable_len = len(payload) - (len(payload) % 4)
    values: List[float] = []

    for i in range(0, usable_len, 4):
        raw = int.from_bytes(payload[i:i + 4], byteorder="big", signed=False)
        values.append((raw / scale) - offset)

    return values


def write_g_values_csv(g_values: List[float], csv_path: str) -> None:
    with open(csv_path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["sample_index", "g_value"])
        for sample_index, g_value in enumerate(g_values):
            writer.writerow([sample_index, g_value])


class Ice2VimBlobReader:
    """
    ICE2/3 Modbus TCP helper for VIM BLOB reads.

    Register mapping follows the original script:
      - ISDU write request register: port * 1000 + 300
      - ISDU read response register: port * 1000 + 100
      - process data register base:  port * 1000
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
        self.pdi_register = iol_port * 1000

        if ModbusClient is None:
            raise RuntimeError(
                "pyModbusTCP is not installed. Install it with: pip install pyModbusTCP"
            )

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
        Write raw bytes to an IO-Link ISDU index through the ICE2/3 Modbus mapping.

        Request format from original script:
          [2, index, subindex, payload_length, data_words...]
        """
        words = bytes_to_words(payload)
        request = [2, index, subindex, len(payload), *words]
        self._debug(f"ISDU write index={index}, len={len(payload)}, request={request}")
        self._write_multiple(
            self.write_register,
            request,
            context=f"ISDU write index {index}",
        )

    def isdu_read_bytes(
        self,
        index: int,
        length: int,
        subindex: int = 0,
        delay_s: float | None = None,
    ) -> Tuple[bytes, List[int]]:
        """
        Read raw bytes from an IO-Link ISDU index through the ICE2/3 Modbus mapping.

        Request format from original script:
          [1, index, subindex, requested_length]
        Response format used by original script:
          [status, index, subindex, actual_length, data_words...]
        """
        request = [1, index, subindex, length]
        self._debug(f"ISDU read request index={index}, len={length}, request={request}")
        self._write_multiple(
            self.write_register,
            request,
            context=f"ISDU read request index {index}",
        )

        time.sleep(self.isdu_delay_s if delay_s is None else delay_s)

        registers_to_read = 4 + ((length + 1) // 2)
        response = self._read_holding(
            self.read_register,
            registers_to_read,
            context=f"ISDU read response index {index}",
        )

        status = response[0]
        response_index = response[1]
        response_subindex = response[2]
        response_len = response[3]

        self._debug(
            f"ISDU response status={status}, index={response_index}, "
            f"subindex={response_subindex}, len={response_len}, raw={response}"
        )

        if response_index != index:
            self._debug(
                f"Response index {response_index} does not match requested index {index}"
            )

        if response_subindex != subindex:
            self._debug(
                f"Response subindex {response_subindex} does not match requested subindex {subindex}"
            )

        if response_len < 0:
            raise ModbusError(f"ISDU read index {index}: invalid response length {response_len}")

        # Some gateways return the requested length even if the actual IO-Link ISDU
        # is shorter. The BLOB parser below uses the BLOB header and expected BLOB
        # length to ignore trailing gateway/register padding where appropriate.
        response_len = min(response_len, length)
        data = words_to_bytes(response[4:])[:response_len]

        return data, response

    def read_u8(self, index: int) -> int:
        data, _ = self.isdu_read_bytes(index, 1)
        if len(data) < 1:
            raise ModbusError(f"Index {index}: expected 1 byte, got {len(data)}")
        return data[0]

    def write_u8(self, index: int, value: int) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError(f"U8 value out of range: {value}")
        self.isdu_write_bytes(index, bytes([value]))

    def read_u16(self, index: int) -> int:
        data, _ = self.isdu_read_bytes(index, 2)
        if len(data) < 2:
            raise ModbusError(f"Index {index}: expected 2 bytes, got {len(data)}")
        return int.from_bytes(data[:2], byteorder="big", signed=False)

    def write_u16(self, index: int, value: int) -> None:
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"U16 value out of range: {value}")
        self.isdu_write_bytes(index, value.to_bytes(2, byteorder="big"))

    def read_u32(self, index: int) -> int:
        data, _ = self.isdu_read_bytes(index, 4)
        if len(data) < 4:
            raise ModbusError(f"Index {index}: expected 4 bytes, got {len(data)}")
        return int.from_bytes(data[:4], byteorder="big", signed=False)

    def read_blob_id(self) -> int:
        data, _ = self.isdu_read_bytes(BLOB_ID_INDEX, 2)
        if len(data) < 2:
            raise ModbusError(f"BLOB_ID: expected 2 bytes, got {len(data)}")
        return int.from_bytes(data[:2], byteorder="big", signed=True)

    def blob_start(self, blob_id: int = -4096) -> None:
        """
        Write BLOB_Start to BLOB_CH.

        BLOB_ID -4096 is encoded as 0xF000, matching the original trigger:
          [2, 50, 0, 3, 0xF1F0, 0x0000]
        """
        blob_id_u16 = blob_id & 0xFFFF
        payload = bytes([
            (BLOB_COMMAND << 4) | CMD_START,
            (blob_id_u16 >> 8) & 0xFF,
            blob_id_u16 & 0xFF,
        ])
        self.isdu_write_bytes(BLOB_CH_INDEX, payload)

    def blob_abort(self) -> None:
        payload = bytes([(BLOB_COMMAND << 4) | CMD_ABORT])
        self.isdu_write_bytes(BLOB_CH_INDEX, payload)

    def blob_finish(self) -> None:
        # BLOB_Finish is one byte: 0xF2. It is not 0x00F2 length 2.
        payload = bytes([(BLOB_COMMAND << 4) | CMD_FINISH])
        self.isdu_write_bytes(BLOB_CH_INDEX, payload)

    def read_blob_ch_once(self) -> Tuple[int, int, bytes, List[int]]:
        """
        Perform exactly one BLOB_CH read and parse the header.

        Do not call this more than once for the same segment. A BLOB_CH read advances
        the device BLOB state machine.
        """
        data, response = self.isdu_read_bytes(BLOB_CH_INDEX, self.max_isdu_len)

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

        return function, subfunction, body, response

    def get_blob_pdi_status(self) -> int:
        """
        Read the BLOB process-data status bits using the same location as original code.

        Original:
          read register base = port * 1000
          response[11] low byte
          status = (low_byte >> 5) & 0x07
        """
        response = self._read_holding(self.pdi_register, 15, context="Read process data")

        if len(response) < 12:
            raise ModbusError(f"Process data response too short: {response}")

        low_byte = response[11] & 0xFF
        return (low_byte >> 5) & 0x07

    def wait_for_pdi_status(
        self,
        target_status: int,
        timeout_s: float,
        poll_s: float,
        label: str,
    ) -> None:
        start = time.monotonic()
        last_status = None

        while True:
            status = self.get_blob_pdi_status()

            if status != last_status:
                print(f"{label}: PDI BLOB status = {status}")
                last_status = status

            if status == target_status:
                return

            elapsed = time.monotonic() - start
            if elapsed >= timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for PDI BLOB status {target_status}; "
                    f"last status was {status}"
                )

            time.sleep(poll_s)

    def configure_blob(
        self,
        ssc_trigger: int = 0,
        raw_data_sampling_rate: int = 0,
        raw_data_memory_size: int = 5,
    ) -> int:
        """
        Configure the VIM BLOB capture settings.

        These indices and meanings are taken from the original script:
          101 = SSC trigger
           97 = raw data sampling rate
           98 = raw data memory size
           99 = raw data recording time
          100 = raw data transfer time
        """
        print("Configuring VIM BLOB parameters...")

        self.write_u8(101, ssc_trigger)
        time.sleep(1.0)
        print(f"SSC trigger is set to {self.read_u8(101)}")

        self.write_u8(97, raw_data_sampling_rate)
        time.sleep(1.0)
        print(f"Raw Data Sampling Rate is set to {self.read_u8(97)}")

        self.write_u16(98, raw_data_memory_size)
        time.sleep(1.0)
        print(f"Raw Data Memory Size is set to {self.read_u16(98)}")

        recording_time = self.read_u32(99)
        print(
            f"BLOB Raw Data Recording Time is {recording_time:.2f} seconds "
            f"or {recording_time / 60:.2f} minutes"
        )

        transfer_time = self.read_u16(100)
        print(
            f"BLOB Raw Data Transfer Time is {transfer_time:.2f} seconds "
            f"or {transfer_time / 60:.2f} minutes"
        )

        return recording_time

    def read_blob_verified(self) -> Tuple[bytes, int, int]:
        """
        Read the active BLOB from the device and verify:
          - BLOB_Info_Read length
          - BLOB_Segment flow counter modulo 16
          - BLOB_Last length
          - BLOB_CRC matches locally calculated IO-Link BLOB CRC-32

        Sends BLOB_Finish only after length and CRC checks pass.
        Sends BLOB_Abort on protocol/length/CRC failure.
        """
        try:
            function, subfunction, body, _ = self.read_blob_ch_once()

            if function != BLOB_INFO or subfunction != 0x0:
                raise BlobTransferError(
                    f"Expected BLOB_Info_Read header 0x10, got "
                    f"function=0x{function:X}, subfunction=0x{subfunction:X}"
                )

            if len(body) < 4:
                raise BlobTransferError(
                    f"BLOB_Info_Read body too short: expected 4 length bytes, got {len(body)}"
                )

            expected_blob_len = int.from_bytes(body[:4], byteorder="big", signed=False)
            print(f"Expected BLOB payload length: {expected_blob_len} bytes")

            payload = bytearray()
            local_crc = 1
            expected_flow = 0
            segment_count = 0
            last_progress_print = time.monotonic()

            while True:
                function, subfunction, body, _ = self.read_blob_ch_once()

                if function == BLOB_SEGMENT:
                    if subfunction != expected_flow:
                        raise BlobTransferError(
                            f"BLOB flow counter mismatch: expected {expected_flow}, "
                            f"got {subfunction}"
                        )

                    payload.extend(body)
                    local_crc = iolink_blob_crc32_update(body, local_crc)

                    segment_count += 1
                    expected_flow = (expected_flow + 1) & 0x0F

                    if len(payload) > expected_blob_len:
                        raise BlobTransferError(
                            f"Received more payload bytes than expected: "
                            f"{len(payload)} > {expected_blob_len}"
                        )

                    now = time.monotonic()
                    if now - last_progress_print >= 1.0:
                        print(
                            f"Received {len(payload)} / {expected_blob_len} bytes "
                            f"({segment_count} BLOB_Segment reads)"
                        )
                        last_progress_print = now

                elif function == BLOB_LAST:
                    if subfunction != 0x0:
                        raise BlobTransferError(
                            f"Invalid BLOB_Last subfunction: expected 0, got {subfunction}"
                        )

                    remaining = expected_blob_len - len(payload)

                    if remaining < 0:
                        raise BlobTransferError(
                            f"Already received more bytes than expected before BLOB_Last: "
                            f"{len(payload)} > {expected_blob_len}"
                        )

                    if len(body) < remaining:
                        raise BlobTransferError(
                            f"BLOB_Last too short: got {len(body)} bytes, "
                            f"expected {remaining}"
                        )

                    # The IO-Link last segment has no padding. Some Modbus gateways,
                    # however, may expose zero/stale bytes past the ISDU length because
                    # we read a fixed register count. Only the expected remainder is
                    # part of the BLOB and CRC.
                    last_data = body[:remaining]
                    payload.extend(last_data)
                    local_crc = iolink_blob_crc32_update(last_data, local_crc)
                    print(f"Received final BLOB_Last: {len(last_data)} bytes")
                    break

                else:
                    raise BlobTransferError(
                        f"Expected BLOB_Segment or BLOB_Last, got "
                        f"function=0x{function:X}, subfunction=0x{subfunction:X}"
                    )

            if len(payload) != expected_blob_len:
                raise BlobTransferError(
                    f"Incomplete BLOB: received {len(payload)} bytes, "
                    f"expected {expected_blob_len}"
                )

            function, subfunction, body, _ = self.read_blob_ch_once()

            if function != BLOB_CRC or subfunction != 0x0:
                raise BlobTransferError(
                    f"Expected BLOB_CRC header 0x40, got "
                    f"function=0x{function:X}, subfunction=0x{subfunction:X}"
                )

            if len(body) < 4:
                raise BlobTransferError(
                    f"BLOB_CRC body too short: expected 4 CRC bytes, got {len(body)}"
                )

            device_crc = int.from_bytes(body[:4], byteorder="big", signed=False)

            print(f"Local CRC : 0x{local_crc:08X}")
            print(f"Device CRC: 0x{device_crc:08X}")

            if local_crc != device_crc:
                raise BlobTransferError(
                    f"BLOB CRC mismatch: local=0x{local_crc:08X}, "
                    f"device=0x{device_crc:08X}"
                )

            self.blob_finish()
            print("BLOB_Finish sent. BLOB transfer verified successfully.")

            return bytes(payload), local_crc, device_crc

        except Exception:
            # Best effort abort on any transfer failure.
            try:
                self.blob_abort()
                print("BLOB_Abort sent after transfer failure.")
            except Exception as abort_error:
                print(f"Could not send BLOB_Abort: {abort_error}", file=sys.stderr)
            raise



def default_env_file_path() -> str:
    """Default config file path: vim_blob_crc_reader.env next to this script."""
    return str(Path(__file__).with_suffix(".env"))


def parse_env_line(line: str, line_number: int) -> tuple[str, str] | None:
    """Parse one KEY=VALUE line from the environment file."""
    stripped = line.strip()

    if not stripped or stripped.startswith("#"):
        return None

    if stripped.startswith("export "):
        stripped = stripped[len("export "):].lstrip()

    if "=" not in stripped:
        raise ValueError(f"line {line_number}: expected KEY=value, got {line.rstrip()!r}")

    key, value = stripped.split("=", 1)
    key = key.strip()
    value = value.strip()

    if not key:
        raise ValueError(f"line {line_number}: empty environment variable name")

    if (len(value) >= 2) and (value[0] == value[-1]) and value[0] in {"'", '"'}:
        value = value[1:-1]
    else:
        # Allow inline comments for unquoted values, for example:
        # VIM_IOL_PORT=4  # port 4
        for marker in (" #", "\t#"):
            comment_index = value.find(marker)
            if comment_index != -1:
                value = value[:comment_index].rstrip()
                break

    return key, value


def load_env_file(path: str, required: bool = False) -> dict[str, str]:
    """Load a small .env-style file without requiring python-dotenv."""
    env_path = Path(path).expanduser()

    if not env_path.exists():
        if required:
            raise FileNotFoundError(f"Env file not found: {env_path}")
        return {}

    values: dict[str, str] = {}

    with env_path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            parsed = parse_env_line(line, line_number)
            if parsed is None:
                continue
            key, value = parsed
            values[key] = value

    return values


def env_file_was_explicitly_requested(argv: List[str]) -> bool:
    return any(arg == "--env-file" or arg.startswith("--env-file=") for arg in argv)


def env_value(env_values: dict[str, str], name: str) -> str | None:
    """OS environment variables override values from the env file."""
    if name in os.environ:
        return os.environ[name]
    return env_values.get(name)


def env_str(env_values: dict[str, str], name: str, default: str) -> str:
    value = env_value(env_values, name)
    return default if value is None or value == "" else value


def env_int(env_values: dict[str, str], name: str, default: int) -> int:
    value = env_value(env_values, name)
    if value is None or value == "":
        return default
    try:
        return int(value, 0)
    except ValueError as exc:
        raise ValueError(f"{name}={value!r} is not an integer") from exc


def env_float(env_values: dict[str, str], name: str, default: float) -> float:
    value = env_value(env_values, name)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"{name}={value!r} is not a number") from exc


def env_optional_float(env_values: dict[str, str], name: str, default: float | None) -> float | None:
    value = env_value(env_values, name)
    if value is None or value.strip() == "":
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"{name}={value!r} is not a number or blank") from exc


def env_bool(env_values: dict[str, str], name: str, default: bool) -> bool:
    value = env_value(env_values, name)
    if value is None or value == "":
        return default

    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False

    raise ValueError(
        f"{name}={value!r} is not boolean. Use true/false, yes/no, on/off, or 1/0."
    )


def build_arg_defaults(env_values: dict[str, str]) -> dict[str, object]:
    """Build argparse defaults from env file values plus real OS environment variables."""
    return {
        "host": env_str(env_values, "VIM_HOST", "192.168.137.21"),
        "tcp_port": env_int(env_values, "VIM_TCP_PORT", 502),
        "unit_id": env_int(env_values, "VIM_UNIT_ID", 1),
        "iol_port": env_int(env_values, "VIM_IOL_PORT", 4),
        "blob_id": env_int(env_values, "VIM_BLOB_ID", -4096),
        "ssc_trigger": env_int(env_values, "VIM_SSC_TRIGGER", 0),
        "sample_rate": env_int(env_values, "VIM_SAMPLE_RATE", 0),
        "memory_size": env_int(env_values, "VIM_MEMORY_SIZE", 5),
        "skip_config": env_bool(env_values, "VIM_SKIP_CONFIG", False),
        "max_isdu_len": env_int(env_values, "VIM_MAX_ISDU_LEN", 201),
        "isdu_delay": env_float(env_values, "VIM_ISDU_DELAY", 0.2),
        "recording_wait": env_optional_float(env_values, "VIM_RECORDING_WAIT", None),
        "recording_buffer": env_float(env_values, "VIM_RECORDING_BUFFER", 1.0),
        "no_initial_abort": env_bool(env_values, "VIM_NO_INITIAL_ABORT", False),
        "use_pdi_wait": env_bool(env_values, "VIM_USE_PDI_WAIT", False),
        "poll": env_float(env_values, "VIM_POLL", 0.1),
        "start_timeout": env_float(env_values, "VIM_START_TIMEOUT", 60.0),
        "finish_timeout": env_float(env_values, "VIM_FINISH_TIMEOUT", 900.0),
        "timeout": env_float(env_values, "VIM_TIMEOUT", 5.0),
        "raw_output": env_str(env_values, "VIM_RAW_OUTPUT", "blob_payload.bin"),
        "csv_output": env_str(env_values, "VIM_CSV_OUTPUT", "g_values.csv"),
        "no_parse": env_bool(env_values, "VIM_NO_PARSE", False),
        "debug": env_bool(env_values, "VIM_DEBUG", False),
    }

def parse_args(argv: List[str] | None = None) -> argparse.Namespace:
    """
    Parse configuration using this precedence:
      1. Command-line options, highest priority
      2. Real OS environment variables, for example VIM_HOST
      3. Env file values, default file vim_blob_crc_reader.env next to this script
      4. Built-in defaults
    """
    if argv is None:
        argv = sys.argv[1:]

    env_probe = argparse.ArgumentParser(add_help=False)
    env_probe.add_argument("--env-file", default=os.environ.get("VIM_ENV_FILE", default_env_file_path()))
    env_probe.add_argument("--no-env-file", action="store_true")
    env_args, _ = env_probe.parse_known_args(argv)

    env_values: dict[str, str] = {}
    env_file_loaded = False

    if not env_args.no_env_file:
        try:
            env_values = load_env_file(
                env_args.env_file,
                required=env_file_was_explicitly_requested(argv),
            )
            env_file_loaded = bool(env_values)
        except Exception as exc:
            env_probe.error(str(exc))

    try:
        defaults = build_arg_defaults(env_values)
    except ValueError as exc:
        env_probe.error(str(exc))

    parser = argparse.ArgumentParser(
        description="Read and CRC-check VIM raw BLOB data through ICE2/3 Modbus TCP.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--env-file",
        default=env_args.env_file,
        help="Env file to read before command-line options are applied.",
    )
    parser.add_argument(
        "--no-env-file",
        action="store_true",
        default=env_args.no_env_file,
        help="Ignore the env file and use only OS environment variables, CLI options, and defaults.",
    )

    parser.add_argument("--host", default=defaults["host"], help="ICE2/3 Modbus TCP IP address")
    parser.add_argument("--tcp-port", type=int, default=defaults["tcp_port"], help="Modbus TCP port")
    parser.add_argument("--unit-id", type=int, default=defaults["unit_id"], help="Modbus unit ID")
    parser.add_argument("--iol-port", type=int, default=defaults["iol_port"], help="ICE2/3 IO-Link port number")

    parser.add_argument(
        "--blob-id",
        type=int,
        default=defaults["blob_id"],
        help="BLOB_ID to start/read. Default -4096 encodes to 0xF000.",
    )

    parser.add_argument("--ssc-trigger", type=int, default=defaults["ssc_trigger"], help="VIM SSC trigger setting")
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=defaults["sample_rate"],
        help=(
            "Raw data sampling rate: 0=64kHz, 1=32kHz, 2=16kHz, "
            "3=8kHz, 4=4kHz, 5=2kHz"
        ),
    )
    parser.add_argument("--memory-size", type=int, default=defaults["memory_size"], help="Raw data memory size")
    parser.add_argument(
        "--skip-config",
        dest="skip_config",
        action="store_true",
        default=defaults["skip_config"],
        help="Skip writing VIM config indices 97, 98, 101.",
    )
    parser.add_argument(
        "--do-config",
        dest="skip_config",
        action="store_false",
        default=argparse.SUPPRESS,
        help="Write VIM config indices even if VIM_SKIP_CONFIG=true in the env file.",
    )

    parser.add_argument(
        "--max-isdu-len",
        type=int,
        default=defaults["max_isdu_len"],
        help="Maximum BLOB_CH ISDU read length in bytes. Original script used 201.",
    )
    parser.add_argument(
        "--isdu-delay",
        type=float,
        default=defaults["isdu_delay"],
        help="Delay after each ISDU request before reading Modbus response.",
    )
    parser.add_argument(
        "--recording-wait",
        type=float,
        default=defaults["recording_wait"],
        help=(
            "Seconds to wait after BLOB_Start before reading BLOB_CH. "
            "Default: use the VIM recording time from index 99 plus --recording-buffer."
        ),
    )
    parser.add_argument(
        "--recording-buffer",
        type=float,
        default=defaults["recording_buffer"],
        help="Extra seconds added to the VIM recording time for the default timed wait.",
    )
    parser.add_argument(
        "--no-initial-abort",
        dest="no_initial_abort",
        action="store_true",
        default=defaults["no_initial_abort"],
        help="Do not send a best-effort BLOB_Abort cleanup before BLOB_Start.",
    )
    parser.add_argument(
        "--initial-abort",
        dest="no_initial_abort",
        action="store_false",
        default=argparse.SUPPRESS,
        help="Send the initial BLOB_Abort cleanup even if VIM_NO_INITIAL_ABORT=true.",
    )
    parser.add_argument(
        "--use-pdi-wait",
        dest="use_pdi_wait",
        action="store_true",
        default=defaults["use_pdi_wait"],
        help=(
            "Use the old PDI status wait for start/finish. Default is off because "
            "some ICE2/3 mappings never report exactly status 1."
        ),
    )
    parser.add_argument(
        "--no-pdi-wait",
        dest="use_pdi_wait",
        action="store_false",
        default=argparse.SUPPRESS,
        help="Use timed recording wait even if VIM_USE_PDI_WAIT=true.",
    )
    parser.add_argument("--poll", type=float, default=defaults["poll"], help="PDI status polling period")
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=defaults["start_timeout"],
        help="Seconds to wait for raw recording to start when --use-pdi-wait is enabled",
    )
    parser.add_argument(
        "--finish-timeout",
        type=float,
        default=defaults["finish_timeout"],
        help="Seconds to wait for raw recording to finish when --use-pdi-wait is enabled",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=defaults["timeout"],
        help="Modbus TCP socket timeout in seconds",
    )

    parser.add_argument("--raw-output", default=defaults["raw_output"], help="Raw verified BLOB output file")
    parser.add_argument("--csv-output", default=defaults["csv_output"], help="Converted g-value CSV output file")
    parser.add_argument(
        "--no-parse",
        dest="no_parse",
        action="store_true",
        default=defaults["no_parse"],
        help="Only save raw BLOB payload; do not convert to g_values.csv.",
    )
    parser.add_argument(
        "--parse",
        dest="no_parse",
        action="store_false",
        default=argparse.SUPPRESS,
        help="Parse g-values even if VIM_NO_PARSE=true.",
    )

    parser.add_argument(
        "--debug",
        dest="debug",
        action="store_true",
        default=defaults["debug"],
        help="Print raw ISDU/Modbus debug data",
    )
    parser.add_argument(
        "--no-debug",
        dest="debug",
        action="store_false",
        default=argparse.SUPPRESS,
        help="Disable debug output even if VIM_DEBUG=true.",
    )

    args = parser.parse_args(argv)
    args.env_file_loaded = env_file_loaded
    return args


def main() -> int:
    args = parse_args()

    if not args.no_env_file:
        if args.env_file_loaded:
            print(f"Loaded settings from env file: {Path(args.env_file).expanduser().resolve()}")
        else:
            print(f"No env file loaded. Looked for: {Path(args.env_file).expanduser()}")

    reader = None

    try:
        reader = Ice2VimBlobReader(
            host=args.host,
            tcp_port=args.tcp_port,
            unit_id=args.unit_id,
            iol_port=args.iol_port,
            max_isdu_len=args.max_isdu_len,
            isdu_delay_s=args.isdu_delay,
            timeout_s=args.timeout,
            debug=args.debug,
        )

        recording_time = None
        if not args.skip_config:
            recording_time = reader.configure_blob(
                ssc_trigger=args.ssc_trigger,
                raw_data_sampling_rate=args.sample_rate,
                raw_data_memory_size=args.memory_size,
            )
        else:
            # Still try to read the VIM recording-time parameter so the default
            # wait is not a blind delay when config writes are skipped.
            try:
                recording_time = reader.read_u32(99)
                print(
                    f"BLOB Raw Data Recording Time is {recording_time:.2f} seconds "
                    f"or {recording_time / 60:.2f} minutes"
                )
            except Exception as exc:
                print(f"Could not read recording time from index 99: {exc}")

        time.sleep(1.0)
        print(f"Initial BLOB_ID: {reader.read_blob_id()}")

        # Clear any transfer left active by a previous timeout or failed finish.
        # If no BLOB transfer is active, the device/master may reject this; that is OK.
        if not args.no_initial_abort:
            print("Sending BLOB_Abort once to clear any old active transfer...")
            try:
                reader.blob_abort()
            except Exception as exc:
                print(f"BLOB_Abort was not accepted or not needed: {exc}")
            time.sleep(0.5)

        print(f"Starting BLOB capture/read with BLOB_ID {args.blob_id}...")
        reader.blob_start(args.blob_id)
        time.sleep(0.5)

        try:
            print(f"BLOB_ID after start: {reader.read_blob_id()}")
        except Exception as exc:
            print(f"Could not read BLOB_ID after start: {exc}")

        if args.use_pdi_wait:
            print("Waiting for raw data recording to start using PDI status...")
            reader.wait_for_pdi_status(
                target_status=1,
                timeout_s=args.start_timeout,
                poll_s=args.poll,
                label="Start wait",
            )
            print("Raw data recording started.")

            print("Waiting for raw data recording to finish using PDI status...")
            reader.wait_for_pdi_status(
                target_status=2,
                timeout_s=args.finish_timeout,
                poll_s=args.poll,
                label="Finish wait",
            )
            print("Raw data recording finished. Reading BLOB data...")
        else:
            if args.recording_wait is not None:
                wait_seconds = max(float(args.recording_wait), 0.0)
                wait_source = "--recording-wait / VIM_RECORDING_WAIT"
            elif recording_time is not None:
                wait_seconds = max(
                    float(recording_time) + float(args.recording_buffer),
                    float(args.recording_buffer),
                )
                wait_source = "index 99 recording time + recording buffer"
            else:
                wait_seconds = max(float(args.recording_buffer), 1.0)
                wait_source = "fallback wait"

            print(
                f"Waiting {wait_seconds:.2f} seconds for raw data recording "
                f"({wait_source}) instead of waiting on PDI status..."
            )
            time.sleep(wait_seconds)
            print("Recording wait finished. Reading BLOB data...")

        try:
            print(f"Active BLOB_ID before read: {reader.read_blob_id()}")
        except Exception as exc:
            print(f"Could not read active BLOB_ID before read: {exc}")

        payload, local_crc, device_crc = reader.read_blob_verified()

        raw_path = Path(args.raw_output)
        raw_path.write_bytes(payload)
        print(f"Saved verified raw BLOB payload to: {raw_path.resolve()}")

        if not args.no_parse:
            g_values = parse_g_values_from_payload(payload)
            write_g_values_csv(g_values, args.csv_output)
            print(f"Saved {len(g_values)} g-values to: {Path(args.csv_output).resolve()}")

        time.sleep(1.0)
        try:
            print(f"Final PDI BLOB status: {reader.get_blob_pdi_status()}")
        except Exception as exc:
            print(f"Could not read final PDI BLOB status: {exc}")

        try:
            print(f"Final BLOB_ID: {reader.read_blob_id()}")
        except Exception as exc:
            print(f"Could not read final BLOB_ID: {exc}")

        print(
            f"Done. CRC verified: local=0x{local_crc:08X}, device=0x{device_crc:08X}, "
            f"bytes={len(payload)}"
        )
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user. Sending BLOB_Abort...", file=sys.stderr)
        if reader is not None:
            try:
                reader.blob_abort()
            except Exception as exc:
                print(f"Could not send BLOB_Abort: {exc}", file=sys.stderr)
        return 130

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        if reader is not None:
            reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
