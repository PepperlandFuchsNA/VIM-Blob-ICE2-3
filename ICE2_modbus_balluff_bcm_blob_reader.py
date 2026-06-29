#!/usr/bin/env python3
"""
ICE2_modbus_balluff_bcm_blob_reader.py

Read raw acceleration BLOBs from a Balluff BCM R16E-004-CI02/BCM0003
through a Pepperl+Fuchs ICE2/ICE3 IO-Link master using Modbus/TCP.

BCM fresh-response version v9 + timeout-safe pacing + Balluff manual-aligned raw payload parser:
This keeps the working Pepperl+Fuchs VIM32PP BLOB pattern, but adds one
important guard for BCM0003 + ICE2/ICE3 Modbus/TCP:
- before every stateful BLOB_CH read request, snapshot the ICE ISDU response
  mailbox;
- after the BLOB_CH read request, poll the response mailbox until it changes;
- only then parse the BLOB header/body;
- add a real quiet gap before every state-advancing BLOB_CH read request, including
  the first segment read after BLOB_Info;
- default to fail-fast on 0x3001 BLOB_CH read failures. Testing showed retrying
  a failed stateful BLOB_CH read can advance/consume a segment inside the master
  and produce a flow skip, so retries are now opt-in only.
- parse Balluff raw acceleration payload using the documented 11-byte header:
  timestamp, trigger source, trigger mode, recording time, and sampling rate;
- guard target values so this raw-acceleration script does not accidentally parse
  spectrum payloads as raw acceleration;
- make stateful BLOB_CH retries/freshness bypasses explicitly unsafe diagnostics;
- print the sensor's estimated raw-axis transfer time when available;
- avoid the old double segment-gap delay that could stretch a 269 kB transfer
  beyond the observed approximately six-minute active-transfer boundary;
- print projected minimum transfer time and elapsed time during progress/error output.

This prevents stale Modbus response images from being mistaken for duplicate
BLOB segments. A BLOB_CH read is stateful, so the code must not recover from a
stale duplicate by blindly issuing another BLOB_CH read request. Exact payload
length and IO-Link BLOB CRC32 remain the final file-save gate.

Install:
pip install pyModbusTCP

Typical run:
python ICE2_modbus_balluff_bcm_blob_reader.py

Useful test run:
python balluff_bcm_blob_reader_v9_timeout_safe.py --max-isdu-len 201 --flow-policy strict --unsafe-blob-read-retries 0 --once

Default behavior:
- sends BLOB_Finish -> BLOB_Abort -> BLOB_Finish at startup to recover from an aborted run
- configures Balluff raw acceleration provider
- triggers one recording
- transfers selected raw acceleration axes, verifies BLOB CRC, saves .bin and .csv files
- waits for Enter before the next recording round unless --once is used

Env file:
By default, reads balluff_bcm_blob_reader.env next to this script.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

try:
    from pyModbusTCP.client import ModbusClient
except ModuleNotFoundError:  # pragma: no cover
    ModbusClient = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# IO-Link / Balluff indices
# ---------------------------------------------------------------------------

# Standard IO-Link BLOB profile indices.
BLOB_ID_INDEX = 49  # 0x0031
BLOB_CH_INDEX = 50  # 0x0032

# Balluff BCM raw acceleration and spectrum provider indices.
RAW_CONFIG_INDEX = 8603  # 0x219B
ISDU_TRIGGER_INDEX = 8605  # 0x219D: ISDU data trigger
RESET_FEATURE_INDEX = 8606  # 0x219E: restart feature
STATUS_INDEX = 8607  # 0x219F: raw/spectrum provider status
ESTIMATED_TRANSFER_TIME_INDEX = 8608  # 0x21A0: estimated transfer times

# RAW_CONFIG_INDEX subindices.
SUB_TRIGGER_SOURCE = 1  # DPTG: data provider trigger source
SUB_TRIGGER_MODE = 2  # RADPTM: raw acceleration data provider trigger mode
SUB_AXIS_SELECTION = 3  # DCAS: data collection axis selection
SUB_TARGET = 4  # DCT: data collection target

# Balluff target values at 0x219B:4. This script only parses target 0.
TARGET_RAW_ACCELERATION = 0
TARGET_SPECTRUM_ONLY = 1
TARGET_RAW_AND_SPECTRUM = 2
SUPPORTED_TARGETS = {TARGET_RAW_ACCELERATION}

# ESTIMATED_TRANSFER_TIME_INDEX subindices.
EST_TRANSFER_RAW_ACCEL_PER_AXIS = 1
EST_TRANSFER_AMPLITUDE_SPECTRUM_PER_AXIS = 2
EST_TRANSFER_ENVELOPE_SPECTRUM_PER_AXIS = 3

# STATUS_INDEX subindices for raw acceleration data.
STATUS_RAW_X = 1
STATUS_RAW_Y = 2
STATUS_RAW_Z = 3

# Raw acceleration BLOB_IDs for Balluff BCM.
BLOB_ID_RAW_X = -4096  # 0xF000
BLOB_ID_RAW_Y = -4097  # 0xEFFF
BLOB_ID_RAW_Z = -4098  # 0xEFFE

# BLOB_CH functions, taken from BLOB header high nibble.
BLOB_INFO = 0x1
BLOB_SEGMENT = 0x2
BLOB_LAST = 0x3
BLOB_CRC = 0x4
BLOB_COMMAND = 0xF

# BLOB command subfunctions, low nibble when high nibble is 0xF.
CMD_ABORT = 0x0
CMD_START = 0x1
CMD_FINISH = 0x2

# IO-Link BLOB CRC-32 polynomial, reversed representation.
CRC32_POLY_REVERSED = 0xEB31D82E
U32_MASK = 0xFFFFFFFF

# Modbus limits. Function 03 allows reading up to 125 holding registers in one
# request. BLOB_CH reads need four response header registers plus data words.
MAX_MODBUS_READ_REGISTERS = 125
MAX_BLOB_CH_READ_BYTES = (MAX_MODBUS_READ_REGISTERS - 4) * 2

# Modbus function 16 typically allows writing up to 123 registers in one request.
# This is generous for this script because BLOB commands and Balluff config values
# are small, but the guard prevents accidental oversize ISDU writes.
MAX_MODBUS_WRITE_REGISTERS = 123
MAX_ISDU_WRITE_BYTES = (MAX_MODBUS_WRITE_REGISTERS - 4) * 2

LOGGER = logging.getLogger("balluff_bcm_blob_reader")

# Balluff raw acceleration scale: INT16, LSB = 0.488 mg.
MG_PER_COUNT = 0.488
G_PER_COUNT = MG_PER_COUNT / 1000.0

# ICE2/ICE3 ISDU response control word fields.
ISDU_STATUS_NOP = 0
ISDU_STATUS_IN_PROCESS = 1
ISDU_STATUS_SUCCESS = 2
ISDU_STATUS_FAILURE = 3
ISDU_STATUS_TIMEOUT = 4

ISDU_TYPE_NOP = 0
ISDU_TYPE_READ = 1
ISDU_TYPE_WRITE = 2

# BLOB flow-counter validation modes.
# strict  : fail immediately on any unexpected modulo-16 counter.
# warn    : warn and re-sync to the received counter; exact length + CRC remain hard gates.
# crc_only: do not warn for flow-counter mismatches; exact length + CRC remain hard gates.
FLOW_POLICY_STRICT = "strict"
FLOW_POLICY_WARN = "warn"
FLOW_POLICY_CRC_ONLY = "crc_only"
VALID_FLOW_POLICIES = {FLOW_POLICY_STRICT, FLOW_POLICY_WARN, FLOW_POLICY_CRC_ONLY}

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class AxisInfo:
    name: str
    status_subindex: int
    blob_id: int


AXES: Tuple[AxisInfo, ...] = (
    AxisInfo("x", STATUS_RAW_X, BLOB_ID_RAW_X),
    AxisInfo("y", STATUS_RAW_Y, BLOB_ID_RAW_Y),
    AxisInfo("z", STATUS_RAW_Z, BLOB_ID_RAW_Z),
)


def selected_axes_from_axis_selection(axis_selection: int) -> Tuple[AxisInfo, ...]:
    """Return only the axes enabled by Balluff DCAS / axis selection.

    Axis selection mapping:
    0 = X
    1 = Y
    2 = Z
    3 = X + Y
    4 = X + Z
    5 = Y + Z
    6 = X + Y + Z
    """
    mapping = {
        0: (AXES[0],),
        1: (AXES[1],),
        2: (AXES[2],),
        3: (AXES[0], AXES[1]),
        4: (AXES[0], AXES[2]),
        5: (AXES[1], AXES[2]),
        6: (AXES[0], AXES[1], AXES[2]),
    }

    try:
        return mapping[int(axis_selection)]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported axis selection {axis_selection}. "
            "Use 0=X, 1=Y, 2=Z, 3=X+Y, 4=X+Z, 5=Y+Z, 6=X+Y+Z."
        ) from exc


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class ModbusError(RuntimeError):
    """Raised for Modbus-level or ICE ISDU communication failures."""


class BlobTransferError(RuntimeError):
    """Raised for BLOB protocol, flow, length, or CRC failures."""


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
    return [(padded[i] << 8) | padded[i + 1] for i in range(0, len(padded), 2)]


def u16_from_i16(value: int) -> int:
    """Encode a signed INT16 into an unsigned 16-bit Modbus word."""
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


def isdu_status(control_word: int) -> int:
    """Bits 12-15 of ICE ISDU response word: 1=in process, 2=success, etc."""
    return (control_word >> 12) & 0x0F


def isdu_type(control_word: int) -> int:
    """Bits 0-3 of ICE ISDU response word: 1=read, 2=write, etc."""
    return control_word & 0x0F


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
# Main reader class
# ---------------------------------------------------------------------------

class Ice2BalluffBcmBlobReader:
    """
    ICE2/3 Modbus TCP helper for Balluff BCM raw acceleration BLOB reads.

    ICE2/3 Modbus mapping, using zero-based addresses for pyModbusTCP:
    - ISDU request write register = IO-Link port * 1000 + 300
    - ISDU response read register = IO-Link port * 1000 + 100

    The manual tables show base-1 addresses such as 1301 and 1101 for port 1;
    pyModbusTCP uses zero-based addresses, so this script uses 1300 and 1100.
    """

    def __init__(
        self,
        host: str,
        tcp_port: int = 502,
        unit_id: int = 1,
        iol_port: int = 1,
        max_isdu_len: int = 201,
        isdu_delay_s: float = 0.2,
        blob_segment_gap_s: float = 0.05,
        blob_response_poll_s: float = 0.02,
        blob_response_timeout_s: float = 5.0,
        blob_read_retries: int = 0,
        blob_read_retry_delay_s: float = 0.5,
        disable_blob_response_freshness: bool = False,
        max_duplicate_rereads: int = 20,
        modbus_timeout_s: float = 5.0,
        modbus_retries: int = 2,
        modbus_retry_delay_s: float = 0.05,
        flow_policy: str = FLOW_POLICY_STRICT,
        debug: bool = False,
    ) -> None:
        if ModbusClient is None:
            raise RuntimeError("pyModbusTCP is not installed. Install it with: pip install pyModbusTCP")

        self._validate_init_args(
            host=host,
            tcp_port=tcp_port,
            unit_id=unit_id,
            iol_port=iol_port,
            max_isdu_len=max_isdu_len,
            isdu_delay_s=isdu_delay_s,
            blob_segment_gap_s=blob_segment_gap_s,
            blob_response_poll_s=blob_response_poll_s,
            blob_response_timeout_s=blob_response_timeout_s,
            blob_read_retries=blob_read_retries,
            blob_read_retry_delay_s=blob_read_retry_delay_s,
            max_duplicate_rereads=max_duplicate_rereads,
            modbus_timeout_s=modbus_timeout_s,
            modbus_retries=modbus_retries,
            modbus_retry_delay_s=modbus_retry_delay_s,
            flow_policy=flow_policy,
        )

        self.host = host
        self.tcp_port = tcp_port
        self.unit_id = unit_id
        self.iol_port = iol_port
        self.max_isdu_len = max_isdu_len
        self.isdu_delay_s = isdu_delay_s
        self.blob_segment_gap_s = blob_segment_gap_s
        self.blob_response_poll_s = blob_response_poll_s
        self.blob_response_timeout_s = blob_response_timeout_s
        self.blob_read_retries = blob_read_retries
        self.blob_read_retry_delay_s = blob_read_retry_delay_s
        self.disable_blob_response_freshness = disable_blob_response_freshness
        self.max_duplicate_rereads = max_duplicate_rereads
        self.modbus_retries = modbus_retries
        self.modbus_retry_delay_s = modbus_retry_delay_s
        self.flow_policy = flow_policy
        self.debug = debug

        self.write_register = iol_port * 1000 + 300
        self.read_register = iol_port * 1000 + 100

        self.client = ModbusClient(
            host=host,
            port=tcp_port,
            unit_id=unit_id,
            auto_open=True,
            auto_close=False,
            timeout=modbus_timeout_s,
        )

    @staticmethod
    def _validate_init_args(
        *,
        host: str,
        tcp_port: int,
        unit_id: int,
        iol_port: int,
        max_isdu_len: int,
        isdu_delay_s: float,
        blob_segment_gap_s: float,
        blob_response_poll_s: float,
        blob_response_timeout_s: float,
        blob_read_retries: int,
        blob_read_retry_delay_s: float,
        max_duplicate_rereads: int,
        modbus_timeout_s: float,
        modbus_retries: int,
        modbus_retry_delay_s: float,
        flow_policy: str,
    ) -> None:
        if not host:
            raise ValueError("host must not be empty")
        if not 1 <= tcp_port <= 65535:
            raise ValueError(f"tcp_port must be 1..65535, got {tcp_port}")
        if not 0 <= unit_id <= 255:
            raise ValueError(f"unit_id must be 0..255, got {unit_id}")
        if not 1 <= iol_port <= 8:
            raise ValueError(f"iol_port must be 1..8, got {iol_port}")
        if not 1 <= max_isdu_len <= MAX_BLOB_CH_READ_BYTES:
            raise ValueError(
                f"max_isdu_len must be 1..{MAX_BLOB_CH_READ_BYTES} bytes for one Modbus read, "
                f"got {max_isdu_len}"
            )
        if isdu_delay_s < 0:
            raise ValueError(f"isdu_delay_s must be >= 0, got {isdu_delay_s}")
        if blob_segment_gap_s < 0:
            raise ValueError(f"blob_segment_gap_s must be >= 0, got {blob_segment_gap_s}")
        if blob_response_poll_s <= 0:
            raise ValueError(f"blob_response_poll_s must be > 0, got {blob_response_poll_s}")
        if blob_response_timeout_s <= 0:
            raise ValueError(f"blob_response_timeout_s must be > 0, got {blob_response_timeout_s}")
        if blob_read_retries < 0:
            raise ValueError(f"blob_read_retries must be >= 0, got {blob_read_retries}")
        if blob_read_retry_delay_s < 0:
            raise ValueError(f"blob_read_retry_delay_s must be >= 0, got {blob_read_retry_delay_s}")
        if max_duplicate_rereads < 0:
            raise ValueError(f"max_duplicate_rereads must be >= 0, got {max_duplicate_rereads}")
        if modbus_timeout_s <= 0:
            raise ValueError(f"modbus_timeout_s must be > 0, got {modbus_timeout_s}")
        if modbus_retries < 0:
            raise ValueError(f"modbus_retries must be >= 0, got {modbus_retries}")
        if modbus_retry_delay_s < 0:
            raise ValueError(f"modbus_retry_delay_s must be >= 0, got {modbus_retry_delay_s}")
        if flow_policy not in VALID_FLOW_POLICIES:
            raise ValueError(
                f"flow_policy must be one of {sorted(VALID_FLOW_POLICIES)}, got {flow_policy!r}"
            )

    def __enter__(self) -> "Ice2BalluffBcmBlobReader":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def close(self) -> None:
        self.client.close()

    def _debug(self, message: str) -> None:
        if self.debug:
            LOGGER.debug(message)

    def _read_holding(self, address: int, count: int, context: str) -> List[int]:
        if not 1 <= count <= MAX_MODBUS_READ_REGISTERS:
            raise ValueError(
                f"{context}: invalid Modbus read count {count}; "
                f"allowed range is 1..{MAX_MODBUS_READ_REGISTERS} registers"
            )

        last_error = "no response"
        for attempt in range(self.modbus_retries + 1):
            response = self.client.read_holding_registers(address, count)
            if response is not None and len(response) >= count:
                return response

            if response is None:
                last_error = "no Modbus response"
            else:
                last_error = f"short response: expected {count} registers, got {len(response)}"

            if attempt < self.modbus_retries and self.modbus_retry_delay_s > 0:
                time.sleep(self.modbus_retry_delay_s)

        raise ModbusError(f"{context}: {last_error} from register {address}")

    def _write_multiple(self, address: int, values: List[int], context: str) -> None:
        if not values:
            raise ValueError(f"{context}: no Modbus values supplied")
        if len(values) > MAX_MODBUS_WRITE_REGISTERS:
            raise ValueError(
                f"{context}: write has {len(values)} registers; "
                f"maximum is {MAX_MODBUS_WRITE_REGISTERS}"
            )

        for attempt in range(self.modbus_retries + 1):
            success = self.client.write_multiple_registers(address, values)
            if success:
                return
            if attempt < self.modbus_retries and self.modbus_retry_delay_s > 0:
                time.sleep(self.modbus_retry_delay_s)

        raise ModbusError(f"{context}: failed to write Modbus registers at {address}: {values}")

    def isdu_write_bytes(self, index: int, payload: bytes, subindex: int = 0) -> None:
        """
        VIM-style ISDU write through the ICE2/3 Modbus mapping.

        Request format:
        [2, index, subindex, payload_length, data_words...]

        This intentionally does not poll the ISDU write response. The working
        VIM32PP BLOB reader uses this simple pattern, and BLOB command writes at
        index 50 can leave misleading data in the response mailbox.
        """
        self._validate_isdu_address(index, subindex)
        if len(payload) > MAX_ISDU_WRITE_BYTES:
            raise ValueError(
                f"ISDU write payload too large: {len(payload)} bytes; "
                f"maximum single request payload is {MAX_ISDU_WRITE_BYTES} bytes"
            )

        request = [ISDU_TYPE_WRITE, index, subindex, len(payload), *bytes_to_words(payload)]
        self._debug(
            f"ISDU WRITE request index={index}, sub={subindex}, "
            f"payload={payload.hex(' ')}, request={request}"
        )

        self._write_multiple(
            self.write_register,
            request,
            f"ISDU write index {index} subindex {subindex}",
        )

        if self.isdu_delay_s > 0:
            time.sleep(self.isdu_delay_s)

    def isdu_read_bytes(
        self,
        index: int,
        length: int,
        subindex: int = 0,
        delay_s: Optional[float] = None,
        *,
        strict_response: bool = True,
    ) -> Tuple[bytes, List[int]]:
        """
        VIM-style ISDU read through the ICE2/3 Modbus mapping.

        Request format:
        [1, index, subindex, requested_length]

        Response format used by the VIM reference:
        [status/control_word, index, subindex, actual_length, data_words...]

        For normal ISDU reads, strict_response=True validates status, type,
        index, and subindex so stale mailbox data is not silently accepted.
        BLOB_CH reads call this with strict_response=False because the BLOB
        parser itself validates the header sequence, length, flow, and CRC.
        """
        self._validate_isdu_address(index, subindex)
        if length <= 0:
            raise ValueError(f"ISDU read length must be positive, got {length}")
        if length > MAX_BLOB_CH_READ_BYTES:
            raise ValueError(
                f"ISDU read length {length} exceeds one-read Modbus limit "
                f"of {MAX_BLOB_CH_READ_BYTES} bytes"
            )

        request = [ISDU_TYPE_READ, index, subindex, length]
        self._debug(
            f"ISDU READ request index={index}, sub={subindex}, "
            f"length={length}, request={request}, strict={strict_response}"
        )

        self._write_multiple(
            self.write_register,
            request,
            f"ISDU read request index {index} subindex {subindex}",
        )

        wait_s = self.isdu_delay_s if delay_s is None else delay_s
        if wait_s < 0:
            raise ValueError(f"delay_s must be >= 0, got {wait_s}")
        if wait_s > 0:
            time.sleep(wait_s)

        registers_to_read = 4 + ((length + 1) // 2)
        response = self._read_holding(
            self.read_register,
            registers_to_read,
            f"ISDU read response index {index} subindex {subindex}",
        )

        control_word = response[0]
        response_status = isdu_status(control_word)
        response_type = isdu_type(control_word)
        response_index = response[1]
        response_subindex = response[2]
        response_len = response[3]

        self._debug(
            f"ISDU response control/status=0x{control_word:04X}, "
            f"status={response_status}, type={response_type}, "
            f"index={response_index}, subindex={response_subindex}, "
            f"len={response_len}, raw={response}"
        )

        if strict_response:
            if response_status != ISDU_STATUS_SUCCESS:
                raise ModbusError(
                    f"ISDU read index {index} subindex {subindex} did not succeed: "
                    f"status={response_status}, control_word=0x{control_word:04X}, raw={response}"
                )
            if response_type != ISDU_TYPE_READ:
                raise ModbusError(
                    f"ISDU read index {index} subindex {subindex} returned wrong type: "
                    f"type={response_type}, control_word=0x{control_word:04X}, raw={response}"
                )
            if response_index != index:
                raise ModbusError(
                    f"ISDU stale/mismatched response: requested index {index}, got {response_index}"
                )
            if response_subindex != subindex:
                raise ModbusError(
                    f"ISDU stale/mismatched response: requested subindex {subindex}, got {response_subindex}"
                )
        else:
            if response_index != index:
                self._debug(f"Non-strict ISDU response index mismatch: requested {index}, got {response_index}")
            if response_subindex != subindex:
                self._debug(f"Non-strict ISDU response subindex mismatch: requested {subindex}, got {response_subindex}")

        # Some gateways expose the requested length even if the actual IO-Link
        # data is shorter. The BLOB parser uses BLOB_Info length and BLOB_Last
        # remainder to ignore fixed-register padding.
        response_len = min(max(int(response_len), 0), length)
        data = words_to_bytes(response[4:])[:response_len]

        return data, response

    @staticmethod
    def _validate_isdu_address(index: int, subindex: int) -> None:
        if not 0 <= index <= 0xFFFF:
            raise ValueError(f"ISDU index out of range 0..65535: {index}")
        if not 0 <= subindex <= 0xFF:
            raise ValueError(f"ISDU subindex out of range 0..255: {subindex}")

    # ----- typed ISDU helpers -----

    def write_u8(self, index: int, subindex: int, value: int) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError(f"UINT8 out of range: {value}")
        self.isdu_write_bytes(index, bytes([value]), subindex=subindex)

    def read_u8(self, index: int, subindex: int = 0) -> int:
        data, _ = self.isdu_read_bytes(index, 1, subindex=subindex)
        if len(data) < 1:
            raise ModbusError(f"ISDU index {index} subindex {subindex} returned less than 1 byte")
        return data[0]

    def write_u16(self, index: int, subindex: int, value: int) -> None:
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"UINT16 out of range: {value}")
        self.isdu_write_bytes(index, value.to_bytes(2, byteorder="big"), subindex=subindex)

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

    # ----- BLOB commands -----

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

    def startup_blob_recovery(self) -> None:
        """Clear old active transfer state after a previous program abort."""
        print("Running startup BLOB recovery: Finish -> Abort -> Finish")
        for label, command in (
            ("BLOB_Finish", self.blob_finish),
            ("BLOB_Abort", self.blob_abort),
            ("BLOB_Finish", self.blob_finish),
        ):
            try:
                command()
                print(f"  {label} sent")
            except Exception as exc:
                print(f"  {label} ignored/failed during recovery: {exc}")
            time.sleep(0.1)

    def _blob_ch_register_count(self) -> int:
        return 4 + ((self.max_isdu_len + 1) // 2)

    def _read_blob_ch_response_mailbox(self, context: str) -> Tuple[bytes, List[int]]:
        """Read the ICE ISDU response mailbox only; do not issue a new ISDU request."""
        response = self._read_holding(
            self.read_register,
            self._blob_ch_register_count(),
            context,
        )

        control_word = response[0]
        response_status = isdu_status(control_word)
        response_type = isdu_type(control_word)
        response_index = response[1]
        response_subindex = response[2]
        response_len = response[3]

        self._debug(
            f"BLOB_CH mailbox control/status=0x{control_word:04X}, "
            f"status={response_status}, type={response_type}, "
            f"index={response_index}, subindex={response_subindex}, "
            f"len={response_len}, raw={response}"
        )

        if response_status == ISDU_STATUS_IN_PROCESS:
            return b"", response

        if response_status != ISDU_STATUS_SUCCESS:
            raise ModbusError(
                f"{context}: BLOB_CH ISDU response did not succeed: "
                f"status={response_status}, control_word=0x{control_word:04X}, raw={response}"
            )
        if response_type != ISDU_TYPE_READ:
            raise ModbusError(
                f"{context}: BLOB_CH response has wrong ISDU type: "
                f"type={response_type}, control_word=0x{control_word:04X}, raw={response}"
            )
        if response_index != BLOB_CH_INDEX or response_subindex != 0:
            raise ModbusError(
                f"{context}: BLOB_CH stale/mismatched response: "
                f"index={response_index}, subindex={response_subindex}, raw={response}"
            )

        response_len = min(max(int(response_len), 0), self.max_isdu_len)
        data = words_to_bytes(response[4:])[:response_len]
        return data, response

    def _request_blob_ch_read(self) -> None:
        """Issue exactly one state-advancing BLOB_CH ISDU read request."""
        request = [ISDU_TYPE_READ, BLOB_CH_INDEX, 0, self.max_isdu_len]
        self._debug(
            f"BLOB_CH READ request index={BLOB_CH_INDEX}, sub=0, "
            f"length={self.max_isdu_len}, request={request}"
        )
        self._write_multiple(
            self.write_register,
            request,
            f"BLOB_CH read request index {BLOB_CH_INDEX}",
        )

    def _blob_ch_payload_signature(self, response: List[int]) -> Tuple[int, int, int, Tuple[int, ...]]:
        """
        Signature used for mailbox freshness.

        IMPORTANT:
        Do not include response[0] / control-status word in this signature.
        In real ICE2/ICE3 logs, the mailbox can change from 0x2001 to 0x3001
        while still carrying the exact same stale BLOB_CH payload. Treating the
        control word as freshness caused v3 to parse stale data as a new reply.
        """
        if len(response) < 4:
            return (-1, -1, -1, tuple())
        return (int(response[1]), int(response[2]), int(response[3]), tuple(response[4:]))

    def _try_extract_success_blob_ch_data(
        self,
        response: List[int],
        context: str,
    ) -> Tuple[Optional[bytes], str]:
        """
        Return BLOB_CH data only when the ICE mailbox contains a successful,
        matching BLOB_CH read response.

        Non-success responses such as 0x3001 are treated as transient/stale
        during BLOB transfer polling. They are not parsed as BLOB segments and
        they do not immediately abort the transfer; the caller keeps polling
        the response mailbox until a valid 0x2001 read response arrives or the
        freshness timeout expires.
        """
        if len(response) < 4:
            return None, f"short response: {response}"

        control_word = response[0]
        response_status = isdu_status(control_word)
        response_type = isdu_type(control_word)
        response_index = response[1]
        response_subindex = response[2]
        response_len = response[3]

        self._debug(
            f"{context}: control/status=0x{control_word:04X}, "
            f"status={response_status}, type={response_type}, "
            f"index={response_index}, subindex={response_subindex}, "
            f"len={response_len}, raw={response}"
        )

        if response_status != ISDU_STATUS_SUCCESS:
            return None, (
                f"waiting for successful BLOB_CH read response; "
                f"status={response_status}, control_word=0x{control_word:04X}"
            )
        if response_type != ISDU_TYPE_READ:
            return None, (
                f"waiting for BLOB_CH read response; "
                f"type={response_type}, control_word=0x{control_word:04X}"
            )
        if response_index != BLOB_CH_INDEX or response_subindex != 0:
            return None, (
                f"waiting for matching BLOB_CH response; "
                f"index={response_index}, subindex={response_subindex}"
            )

        response_len = min(max(int(response_len), 0), self.max_isdu_len)
        if response_len <= 0:
            return None, "waiting for non-empty BLOB_CH response"

        data = words_to_bytes(response[4:])[:response_len]
        return data, "ok"

    def read_blob_ch_once(self) -> Tuple[int, int, bytes, List[int]]:
        """
        Perform one stateful BLOB_CH read and parse the BLOB header.

        v6 freshness rules:
        1. Snapshot the current ICE response mailbox before the state-advancing
           BLOB_CH read request.
        2. Send one BLOB_CH read request.
        3. Poll the ICE response mailbox; do not parse stale payloads.
        4. Accept only successful 0x2001 BLOB_CH read responses with a changed
           payload signature.
        5. If the ICE reports a failed BLOB_CH read such as 0x3001 while still
           carrying the old payload, treat that individual read request as
           failed and retry the BLOB_CH read request. This is different from
           blindly accepting or appending a duplicate. However BCM/ICE testing
           showed that retrying a failed stateful read can still skip a segment,
           so v7 defaults blob_read_retries to 0. Strict flow validation
           after any successful response still catches any real advance/skip.
        """
        if self.disable_blob_response_freshness:
            data, response = self.isdu_read_bytes(
                BLOB_CH_INDEX,
                self.max_isdu_len,
                strict_response=False,
            )
            return self._parse_blob_ch_data(data, response)

        last_response: Optional[List[int]] = None
        last_reason = "no response read yet"
        total_same_count = 0
        total_invalid_count = 0

        for attempt in range(self.blob_read_retries + 1):
            snapshot = self._read_holding(
                self.read_register,
                self._blob_ch_register_count(),
                "BLOB_CH response pre-snapshot",
            )
            snapshot_signature = self._blob_ch_payload_signature(snapshot)

            # Critical BCM/ICE2 pacing rule:
            # BLOB_CH reads are stateful. After BLOB_Info, BLOB_Last, or any
            # failed 0x3001 response, the next read request must not be sent
            # immediately. Earlier versions only slept after accepted segments,
            # which left the BLOB_Info -> first BLOB_Segment transition exposed.
            if self.blob_segment_gap_s > 0:
                time.sleep(self.blob_segment_gap_s)

            self._request_blob_ch_read()

            if self.isdu_delay_s > 0:
                time.sleep(self.isdu_delay_s)

            deadline = time.monotonic() + self.blob_response_timeout_s
            same_count = 0
            invalid_count = 0
            retry_this_request = False

            while True:
                response = self._read_holding(
                    self.read_register,
                    self._blob_ch_register_count(),
                    "BLOB_CH response fresh-poll",
                )
                last_response = response
                response_signature = self._blob_ch_payload_signature(response)
                control_word = response[0] if response else 0
                response_status = isdu_status(control_word)

                if response_signature == snapshot_signature:
                    same_count += 1
                    total_same_count += 1

                    # This is the key v5 fix. In BCM/ICE logs the mailbox can
                    # keep the old BLOB payload while the control word changes
                    # to 0x3001. That means this ISDU read request failed; it
                    # did not provide a new segment to append. Polling the same
                    # response forever will not help. Retrying the BLOB_CH read
                    # request is safer than appending stale data, and strict
                    # flow checking below will still detect a real skipped
                    # segment if the device did advance.
                    if response_status in {ISDU_STATUS_FAILURE, ISDU_STATUS_TIMEOUT}:
                        last_reason = (
                            "BLOB_CH read request failed while mailbox payload stayed unchanged; "
                            f"status={response_status}, control_word=0x{control_word:04X}"
                        )
                        retry_this_request = True
                        self._debug(
                            f"{last_reason}; attempt={attempt + 1}/{self.blob_read_retries + 1}; "
                            "will retry the BLOB_CH read request if retries remain"
                        )
                        break

                    if same_count <= 3 or same_count % 25 == 0:
                        self._debug(
                            "BLOB_CH response payload unchanged after read request; "
                            f"attempt={attempt + 1}/{self.blob_read_retries + 1}, "
                            f"poll_count={same_count}, timeout_s={self.blob_response_timeout_s}"
                        )
                else:
                    data, reason = self._try_extract_success_blob_ch_data(
                        response,
                        "BLOB_CH candidate response",
                    )
                    last_reason = reason
                    if data:
                        return self._parse_blob_ch_data(data, response)

                    invalid_count += 1
                    total_invalid_count += 1

                    # If the mailbox payload changed but the ISDU transaction is
                    # explicitly failed/timed out, do not parse the payload. Treat
                    # this request as failed and retry. A successful retry must
                    # still pass strict BLOB flow validation.
                    if response_status in {ISDU_STATUS_FAILURE, ISDU_STATUS_TIMEOUT}:
                        retry_this_request = True
                        self._debug(
                            "BLOB_CH response changed but the ISDU read failed; "
                            f"reason={reason}; attempt={attempt + 1}/{self.blob_read_retries + 1}; "
                            "will retry the BLOB_CH read request if retries remain"
                        )
                        break

                    if invalid_count <= 3 or invalid_count % 25 == 0:
                        self._debug(
                            "BLOB_CH response changed but is not a successful fresh payload yet; "
                            f"reason={reason}; invalid_count={invalid_count}; "
                            f"timeout_s={self.blob_response_timeout_s}"
                        )

                if time.monotonic() >= deadline:
                    last_reason = (
                        f"timeout waiting for successful fresh BLOB_CH response on attempt "
                        f"{attempt + 1}/{self.blob_read_retries + 1}"
                    )
                    retry_this_request = True
                    self._debug(last_reason)
                    break

                time.sleep(self.blob_response_poll_s)

            if retry_this_request and attempt < self.blob_read_retries:
                # Treat an explicit 0x3001/timeout response as a failed read
                # transaction, not as a BLOB segment. Give the IO-Link master and
                # sensor a real recovery gap before another stateful BLOB_CH read.
                retry_delay = max(self.blob_read_retry_delay_s, self.blob_segment_gap_s)
                if retry_delay > 0:
                    time.sleep(retry_delay)
                continue

            raise BlobTransferError(
                "Timed out or failed waiting for a successful fresh BLOB_CH response. "
                "No stale BLOB payload was appended. "
                "For BCM/ICE, do not blindly retry 0x3001 failures because a failed stateful BLOB_CH read may still advance the transfer. Try a smaller --max-isdu-len, for example 101 or 51, and keep --blob-read-retries 0. "
                f"attempts={attempt + 1}/{self.blob_read_retries + 1}, "
                f"per_request_timeout_s={self.blob_response_timeout_s:.3f}, "
                f"total_same_count={total_same_count}, total_invalid_count={total_invalid_count}, "
                f"last_reason={last_reason}, last_response={last_response}"
            )

        raise BlobTransferError(
            "Unexpected BLOB_CH retry loop exit without a successful fresh response"
        )

    def _read_blob_ch_response_from_registers(
        self,
        response: List[int],
        context: str,
    ) -> Tuple[bytes, List[int]]:
        """Parse a supplied BLOB_CH mailbox register block without re-reading it."""
        data, reason = self._try_extract_success_blob_ch_data(response, context)
        if not data:
            raise ModbusError(f"{context}: no successful BLOB_CH data available: {reason}; raw={response}")
        return data, response

    def _parse_blob_ch_data(self, data: bytes, response: List[int]) -> Tuple[int, int, bytes, List[int]]:
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

    # ----- Balluff raw acceleration provider -----

    def configure_raw_data_provider(
        self,
        trigger_source: int = 2,
        trigger_mode: int = 0,
        axis_selection: int = 6,
        target: int = 0,
    ) -> None:
        """
        Configure Balluff raw acceleration data collection.

        Defaults follow the Balluff raw-acceleration flow:
        8603:1 = 2 -> trigger by ISDU
        8603:2 = 0 -> record after trigger
        8603:3 = 6 -> collect X, Y, Z axes
        8603:4 = 0 -> collect raw acceleration data only

        This script currently parses only target=0. Spectrum targets require a
        separate parser because their BLOB payload layout is different.
        """
        if target not in SUPPORTED_TARGETS:
            raise ValueError(
                f"Unsupported target={target}. This script currently supports only "
                "target=0 (raw acceleration data). Do not parse spectrum payloads "
                "with the raw acceleration CSV parser."
            )

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

    def read_estimated_raw_axis_transfer_time(self) -> int:
        """Read 0x21A0:1, estimated transfer time for raw acceleration per axis."""
        return self.read_u16(ESTIMATED_TRANSFER_TIME_INDEX, EST_TRANSFER_RAW_ACCEL_PER_AXIS)

    def read_axis_status(self, axis: AxisInfo) -> int:
        return self.read_u8(STATUS_INDEX, axis.status_subindex)

    def wait_for_axis_status(
        self,
        axis: AxisInfo,
        target_status: int,
        timeout_s: float,
        poll_s: float,
        allow_already_past_ready: bool = True,
    ) -> int:
        start = time.monotonic()
        last_status: Optional[int] = None

        while True:
            value = self.read_axis_status(axis)

            if value != last_status:
                print(f"{axis.name.upper()} status = {value} ({status_text(value)})")
                last_status = value

            if value == target_status:
                return value

            if allow_already_past_ready and target_status == 2 and value == 3:
                return value

            if time.monotonic() - start > timeout_s:
                raise TimeoutError(
                    f"Timed out waiting for {axis.name.upper()} status {target_status}; "
                    f"last status was {value} ({status_text(value)})"
                )

            time.sleep(poll_s)

    def wait_for_axes_ready_for_trigger(self, axes: Tuple[AxisInfo, ...], timeout_s: float, poll_s: float) -> None:
        for axis in axes:
            self.wait_for_axis_status(axis, 1, timeout_s, poll_s)

    def wait_for_axes_preparing_or_ready(self, axes: Tuple[AxisInfo, ...], timeout_s: float, poll_s: float) -> None:
        for axis in axes:
            self.wait_for_axis_status(axis, 2, timeout_s, poll_s)

    def wait_for_axes_ready_for_blob(self, axes: Tuple[AxisInfo, ...], timeout_s: float, poll_s: float) -> None:
        for axis in axes:
            self.wait_for_axis_status(axis, 3, timeout_s, poll_s)

    def read_blob_verified(self, blob_timeout_s: float = 900.0) -> Tuple[bytes, int, int]:
        """
        Read the active BLOB and verify length and CRC.

        This follows the working VIM32PP BLOB mechanism:
        - 0x10 -> BLOB_Info with expected payload length
        - 0x20..0x2F -> BLOB_Segment with modulo-16 flow counter
        - 0x30 -> BLOB_Last with final bytes
        - 0x40 -> BLOB_CRC
        - 0xF2 -> BLOB_Finish only after local CRC == device CRC

        Balluff/ICE behavior observed during testing:
        Sometimes the next BLOB_CH read returns the exact same BLOB segment
        that was already accepted. This is treated as a stale duplicate only
        when BOTH the flow counter and body bytes match the previous accepted
        segment. The duplicate is not appended and CRC is not updated.

        Safety rules:
        - duplicate segments with identical flow+body are ignored
        - flow counter anomalies are handled according to self.flow_policy
        - payload must match BLOB_Info length exactly
        - local CRC must match device BLOB_CRC before BLOB_Finish/files

        For BCM0003 + ICE2/ICE3, the recommended diagnostic policy is now "strict"
        with the BLOB_CH response freshness barrier enabled. If strict mode still
        reports a flow skip, that skip is no longer being caused by stale Modbus
        mailbox data.
        """
        start_time = time.monotonic()

        def check_timeout() -> None:
            if time.monotonic() - start_time > blob_timeout_s:
                raise TimeoutError(f"Timed out reading BLOB after {blob_timeout_s} seconds")

        try:
            # 1) Read BLOB_Info. One BLOB_CH read after BLOB_Start should return 0x10.
            check_timeout()
            function, subfunction, body, _ = self.read_blob_ch_once()
            if function != BLOB_INFO or subfunction != 0:
                header = (function << 4) | subfunction
                raise BlobTransferError(
                    f"Expected BLOB_Info 0x10, got header 0x{header:02X}, body_len={len(body)}"
                )

            if len(body) < 4:
                raise BlobTransferError(f"BLOB_Info body too short: {body.hex(' ')}")

            expected_blob_len = int.from_bytes(body[:4], byteorder="big", signed=False)
            if expected_blob_len <= 0:
                raise BlobTransferError(f"Invalid BLOB_Info payload length: {expected_blob_len}")
            print(f"Expected BLOB payload size: {expected_blob_len} bytes")

            normal_segment_body_len = max(self.max_isdu_len - 1, 1)
            projected_segment_reads = math.ceil(expected_blob_len / normal_segment_body_len)
            projected_delay_only_s = projected_segment_reads * (
                self.isdu_delay_s + self.blob_segment_gap_s
            )
            print(
                "Projected transfer floor from configured waits: "
                f"~{projected_delay_only_s:.1f}s for ~{projected_segment_reads} data reads "
                "(plus Modbus/IO-Link processing overhead)"
            )

            payload = bytearray()
            local_crc = 1
            expected_flow = 0
            segment_count = 0
            last_progress_print = time.monotonic()

            # Duplicate guard state. These refer only to the previous ACCEPTED segment.
            last_flow: Optional[int] = None
            last_body: Optional[bytes] = None
            duplicate_count = 0
            max_duplicate_rereads = self.max_duplicate_rereads
            flow_anomaly_count = 0

            def handle_flow_anomaly(reason: str) -> None:
                nonlocal flow_anomaly_count
                flow_anomaly_count += 1

                message = (
                    f"BLOB flow anomaly #{flow_anomaly_count}: {reason}; "
                    f"policy={self.flow_policy}, segment_count={segment_count}, "
                    f"received={len(payload)}/{expected_blob_len}"
                )

                if self.flow_policy == FLOW_POLICY_STRICT:
                    raise BlobTransferError(message)

                if self.flow_policy == FLOW_POLICY_WARN:
                    if flow_anomaly_count <= 10 or flow_anomaly_count % 25 == 0:
                        print("WARNING: " + message + ". Continuing; length + CRC remain final gate.")
                else:
                    self._debug(message + ". Continuing; length + CRC remain final gate.")

            # 2) Read BLOB_Segment packets until BLOB_Last.
            while True:
                check_timeout()
                function, subfunction, body, _ = self.read_blob_ch_once()

                # BLOB_Last: append only the remaining expected bytes, then stop segment loop.
                if function == BLOB_LAST and subfunction == 0:
                    remaining = expected_blob_len - len(payload)
                    if remaining < 0:
                        raise BlobTransferError(
                            f"BLOB_Last arrived after too many bytes: {len(payload)} > {expected_blob_len}"
                        )
                    if len(body) < remaining:
                        elapsed_s = time.monotonic() - start_time
                        raise BlobTransferError(
                            "Premature BLOB_Last: the device ended the transfer before the "
                            "BLOB_Info length was delivered. "
                            f"needed={remaining} final bytes, got={len(body)}, "
                            f"received={len(payload)}/{expected_blob_len}, "
                            f"segments={segment_count}, elapsed={elapsed_s:.1f}s. "
                            "This is not normal final-packet padding. On this ICE2/BCM path, "
                            "check whether the transfer duration is approaching a device/master "
                            "session limit and reduce per-segment pacing without enabling unsafe "
                            "stateful BLOB_CH retries."
                        )

                    last_bytes = bytes(body[:remaining])
                    payload.extend(last_bytes)
                    local_crc = iolink_blob_crc32_update(last_bytes, local_crc)
                    print(
                        f"BLOB_Last: bytes={len(last_bytes)}, "
                        f"received={len(payload)}/{expected_blob_len}, crc=0x{local_crc:08X}"
                    )
                    break

                if function != BLOB_SEGMENT:
                    header = (function << 4) | subfunction
                    raise BlobTransferError(
                        "Expected BLOB_Segment 0x2n or BLOB_Last 0x30, got "
                        f"header=0x{header:02X}, body_len={len(body)}"
                    )

                body_bytes = bytes(body)
                flow_anomaly_already_handled = False

                # ------------------------------------------------------------
                # Stale duplicate of the previous ACCEPTED segment.
                # ------------------------------------------------------------
                if last_flow is not None and subfunction == last_flow:
                    if body_bytes == last_body:
                        duplicate_count += 1

                        if duplicate_count <= 10 or duplicate_count % 25 == 0:
                            print(
                                "Ignoring duplicate BLOB segment: "
                                f"flow={subfunction}, expected={expected_flow}, "
                                f"duplicate_count={duplicate_count}, "
                                f"segment_count={segment_count}, "
                                f"received={len(payload)}/{expected_blob_len}"
                            )

                        if duplicate_count > max_duplicate_rereads:
                            raise BlobTransferError(
                                "Too many duplicate BLOB segments while waiting for next data. "
                                f"Repeated flow={subfunction}, duplicate_count={duplicate_count}, "
                                f"received={len(payload)}/{expected_blob_len}. "
                                "Try increasing BCM_BLOB_SEGMENT_GAP_S or reducing BCM_MAX_ISDU_LEN."
                            )

                        # read_blob_ch_once() applies the configured quiet gap before
                        # the next state-advancing BLOB_CH request. Do not sleep here too.
                        continue

                    handle_flow_anomaly(
                        f"repeated flow counter {subfunction} with changed body; "
                        f"expected={expected_flow}, body_len={len(body_bytes)}"
                    )
                    flow_anomaly_already_handled = True

                # ------------------------------------------------------------
                # Normal segment: validate or observe modulo-16 flow counter.
                # On BCM0003 + ICE2/ICE3, occasional counter anomalies can be
                # caused by gateway/mailbox behavior. Therefore the production
                # default is warn+re-sync, while exact payload length and CRC32
                # remain the hard data-integrity gates before saving files.
                # ------------------------------------------------------------
                if not flow_anomaly_already_handled and subfunction != expected_flow:
                    handle_flow_anomaly(
                        f"unexpected flow counter got={subfunction}, expected={expected_flow}, "
                        f"body_len={len(body_bytes)}"
                    )

                if not body_bytes:
                    raise BlobTransferError(
                        f"Empty BLOB_Segment body at flow={subfunction}, "
                        f"received={len(payload)}/{expected_blob_len}"
                    )

                if len(payload) + len(body_bytes) > expected_blob_len:
                    raise BlobTransferError(
                        f"Received too many BLOB bytes: {len(payload)} + {len(body_bytes)} > {expected_blob_len}"
                    )

                payload.extend(body_bytes)
                local_crc = iolink_blob_crc32_update(body_bytes, local_crc)
                segment_count += 1

                if segment_count <= 25 or segment_count % 25 == 0:
                    elapsed_s = time.monotonic() - start_time
                    print(
                        f"Segment {segment_count}: flow={subfunction}, body_len={len(body_bytes)}, "
                        f"received={len(payload)}/{expected_blob_len}, crc=0x{local_crc:08X}, "
                        f"elapsed={elapsed_s:.1f}s"
                    )
                else:
                    now = time.monotonic()
                    if now - last_progress_print >= 2.0:
                        elapsed_s = time.monotonic() - start_time
                        rate_bps = len(payload) / elapsed_s if elapsed_s > 0 else 0.0
                        eta_s = (expected_blob_len - len(payload)) / rate_bps if rate_bps > 0 else math.inf
                        print(
                            f"Received {len(payload)} / {expected_blob_len} bytes "
                            f"({segment_count} BLOB_Segment reads, crc=0x{local_crc:08X}, "
                            f"elapsed={elapsed_s:.1f}s, rate={rate_bps:.1f} B/s, eta={eta_s:.1f}s)"
                        )
                        last_progress_print = now

                last_flow = subfunction
                last_body = body_bytes
                duplicate_count = 0
                # Re-sync expected flow to the accepted segment. In strict mode
                # this is equivalent to expected_flow + 1; in warn/crc_only mode
                # it lets the reader recover after a gateway-visible flow anomaly.
                expected_flow = (subfunction + 1) & 0x0F

                # read_blob_ch_once() applies the configured quiet gap before
                # the next state-advancing BLOB_CH request. The previous extra sleep
                # here doubled the segment gap and could push large transfers beyond
                # the active BLOB session duration.

            # 3) Exact length check.
            if len(payload) != expected_blob_len:
                raise BlobTransferError(f"Incomplete BLOB: received {len(payload)}, expected {expected_blob_len}")

            # 4) Read BLOB_CRC. The VIM reference expects one CRC packet after BLOB_Last.
            check_timeout()
            function, subfunction, body, _ = self.read_blob_ch_once()
            if function != BLOB_CRC or subfunction != 0:
                header = (function << 4) | subfunction
                raise BlobTransferError(
                    f"Expected BLOB_CRC 0x40, got header 0x{header:02X}, body_len={len(body)}"
                )

            if len(body) < 4:
                raise BlobTransferError(f"BLOB_CRC body too short: {body.hex(' ')}")

            device_crc = int.from_bytes(body[:4], byteorder="big", signed=False)
            print(f"Local CRC : 0x{local_crc:08X}")
            print(f"Device CRC: 0x{device_crc:08X}")

            if local_crc != device_crc:
                raise BlobTransferError(
                    f"BLOB CRC mismatch: local=0x{local_crc:08X}, device=0x{device_crc:08X}"
                )

            # 5) Finish only after CRC passes.
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
# Output parsing and files
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RawAccelerationPayload:
    timestamp_ms: int
    trigger_source: int
    trigger_mode: int
    recording_time_s: float
    sampling_rate_ms: float
    raw_counts: List[int]
    acceleration_mg: List[float]
    acceleration_g: List[float]


def parse_raw_acceleration_payload(payload: bytes) -> RawAccelerationPayload:
    """
    Parse Balluff raw acceleration payload.

    Documented raw acceleration data layout:
    - Byte 0..3  : Time from raw acquisition start to BLOB transfer start, UINT32, LSB 1 ms
    - Byte 4     : Trigger Source, UINT8
    - Byte 5     : Trigger Mode, UINT8
    - Byte 6     : Recording Time, UINT8, LSB 0.5 s
    - Byte 7..10 : Sampling Rate, FLOAT32, unit ms
    - Byte 11..  : Raw acceleration samples, INT16, LSB 0.488 mg

    The previous parser treated bytes 4..10 as acceleration samples. That was
    wrong for BCM0003 raw acceleration BLOBs and shifted every CSV sample.
    """
    if len(payload) < 11:
        raise ValueError(
            f"Raw acceleration payload too short: {len(payload)} bytes; "
            "expected at least 11 bytes for the Balluff raw acceleration header"
        )

    timestamp_ms = int.from_bytes(payload[0:4], byteorder="big", signed=False)
    trigger_source = payload[4]
    trigger_mode = payload[5]
    recording_time_s = payload[6] * 0.5

    # IO-Link/Balluff payload fields are big-endian.
    sampling_rate_ms = struct.unpack(">f", payload[7:11])[0]

    sample_bytes = payload[11:]
    if len(sample_bytes) % 2:
        raise ValueError(
            f"Raw acceleration sample section has odd length: {len(sample_bytes)} bytes. "
            "Expected INT16 samples starting at byte 11."
        )

    raw_counts: List[int] = []
    acceleration_mg: List[float] = []
    acceleration_g: List[float] = []

    for offset in range(0, len(sample_bytes), 2):
        raw = int.from_bytes(sample_bytes[offset:offset + 2], byteorder="big", signed=True)
        mg = raw * MG_PER_COUNT
        raw_counts.append(raw)
        acceleration_mg.append(mg)
        acceleration_g.append(mg / 1000.0)

    return RawAccelerationPayload(
        timestamp_ms=timestamp_ms,
        trigger_source=trigger_source,
        trigger_mode=trigger_mode,
        recording_time_s=recording_time_s,
        sampling_rate_ms=sampling_rate_ms,
        raw_counts=raw_counts,
        acceleration_mg=acceleration_mg,
        acceleration_g=acceleration_g,
    )

def _atomic_temp_path(path: Path) -> Path:
    return path.with_name(f".{path.name}.{os.getpid()}.tmp")


def atomic_write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = _atomic_temp_path(path)
    try:
        tmp_path.write_bytes(data)
        tmp_path.replace(path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def save_raw_acceleration_csv(payload: bytes, csv_path: Path) -> None:
    parsed = parse_raw_acceleration_payload(payload)

    sample_count = len(parsed.raw_counts)
    expected_duration_ms = parsed.recording_time_s * 1000.0
    duration_from_sample_count_ms = math.nan
    duration_error_ms = math.nan
    duration_error_percent = math.nan

    if sample_count > 0 and parsed.sampling_rate_ms > 0 and math.isfinite(parsed.sampling_rate_ms):
        # The manual defines sampling rate as acquisition time / number of samples.
        duration_from_sample_count_ms = sample_count * parsed.sampling_rate_ms
        duration_error_ms = duration_from_sample_count_ms - expected_duration_ms
        if expected_duration_ms > 0:
            duration_error_percent = (duration_error_ms / expected_duration_ms) * 100.0
            if abs(duration_error_percent) > 2.0:
                LOGGER.warning(
                    "Raw acceleration duration check is off by %.3f%%: "
                    "recording_time_ms=%.6f, sample_count=%d, sampling_rate_ms=%.9f, "
                    "sample_count_duration_ms=%.6f",
                    duration_error_percent,
                    expected_duration_ms,
                    sample_count,
                    parsed.sampling_rate_ms,
                    duration_from_sample_count_ms,
                )
    else:
        LOGGER.warning(
            "Raw acceleration duration check skipped: sample_count=%d, sampling_rate_ms=%r",
            sample_count,
            parsed.sampling_rate_ms,
        )

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = _atomic_temp_path(csv_path)
    try:
        with tmp_path.open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["blob_timestamp_ms_time_from_acquisition_start_to_blob_transfer", parsed.timestamp_ms])
            writer.writerow(["trigger_source", parsed.trigger_source])
            writer.writerow(["trigger_mode", parsed.trigger_mode])
            writer.writerow(["recording_time_s", f"{parsed.recording_time_s:.6f}"])
            writer.writerow(["recording_time_ms", f"{expected_duration_ms:.6f}"])
            writer.writerow(["sampling_rate_ms", f"{parsed.sampling_rate_ms:.9f}"])
            writer.writerow(["sample_count", sample_count])
            writer.writerow(["duration_from_sample_count_ms", f"{duration_from_sample_count_ms:.6f}"])
            writer.writerow(["duration_error_ms", f"{duration_error_ms:.6f}"])
            writer.writerow(["duration_error_percent", f"{duration_error_percent:.6f}"])
            writer.writerow([])
            writer.writerow([
                "sample_index",
                "sample_time_from_payload_start_ms",
                "raw_counts_int16",
                "acceleration_mg",
                "acceleration_g",
            ])

            for sample_index, (raw, mg, g) in enumerate(
                zip(parsed.raw_counts, parsed.acceleration_mg, parsed.acceleration_g)
            ):
                sample_time_ms = sample_index * parsed.sampling_rate_ms
                writer.writerow([
                    sample_index,
                    f"{sample_time_ms:.9f}",
                    raw,
                    f"{mg:.6f}",
                    f"{g:.9f}",
                ])
        tmp_path.replace(csv_path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()

def ensure_output_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Round orchestration
# ---------------------------------------------------------------------------

def run_recording_round(
    reader: Ice2BalluffBcmBlobReader,
    *,
    round_number: int,
    axes: Tuple[AxisInfo, ...],
    trigger_source: int,
    trigger_mode: int,
    axis_selection: int,
    target: int,
    status_timeout_s: float,
    status_poll_s: float,
    blob_timeout_s: float,
    output_dir: Path,
    skip_config: bool,
) -> None:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    if target not in SUPPORTED_TARGETS:
        raise ValueError(
            f"Unsupported target={target}. This script currently supports only target=0 "
            "(raw acceleration data). Spectrum targets require a different parser."
        )

    axis_text = "/".join(axis.name.upper() for axis in axes)

    print("=" * 72)
    print(f"Starting recording round {round_number} ({timestamp})")
    print(f"Selected axes: {axis_text}")
    print("=" * 72)

    if not skip_config:
        reader.configure_raw_data_provider(
            trigger_source=trigger_source,
            trigger_mode=trigger_mode,
            axis_selection=axis_selection,
            target=target,
        )
    else:
        print("Skipping Balluff raw provider configuration")

    print(f"Waiting for {axis_text} raw acceleration status = 1 (waiting for trigger)")
    reader.wait_for_axes_ready_for_trigger(axes, status_timeout_s, status_poll_s)

    reader.send_isdu_trigger()

    print(f"Waiting for {axis_text} raw acceleration status = 2 (preparing) or 3 (ready)")
    reader.wait_for_axes_preparing_or_ready(axes, status_timeout_s, status_poll_s)

    print(f"Waiting for {axis_text} raw acceleration status = 3 (ready for BLOB transfer)")
    reader.wait_for_axes_ready_for_blob(axes, status_timeout_s, status_poll_s)

    status_parts = []
    for axis in axes:
        status_value = reader.read_axis_status(axis)
        status_parts.append(f"{axis.name.upper()}={status_value} ({status_text(status_value)})")
    status_summary = ", ".join(status_parts)
    print(f"Selected raw acceleration status: {status_summary}")

    try:
        estimated_transfer_time = reader.read_estimated_raw_axis_transfer_time()
        print(f"Estimated raw-axis BLOB transfer time from 0x21A0:1: {estimated_transfer_time}")
    except Exception as exc:
        print(f"Could not read estimated raw-axis transfer time from 0x21A0:1: {exc}")

    ensure_output_dir(output_dir)

    for axis in axes:
        print("-" * 72)
        print(f"Reading {axis.name.upper()} raw acceleration BLOB")

        reader.blob_start(axis.blob_id)
        payload, local_crc, device_crc = reader.read_blob_verified(blob_timeout_s=blob_timeout_s)

        base = f"round_{round_number:03d}_{timestamp}_port{reader.iol_port}_{axis.name}"
        bin_path = output_dir / f"{base}_payload.bin"
        csv_path = output_dir / f"{base}_raw_accel.csv"

        atomic_write_bytes(bin_path, payload)
        save_raw_acceleration_csv(payload, csv_path)

        print(f"Saved binary payload: {bin_path}")
        print(f"Saved CSV       : {csv_path}")
        print(f"Verified CRC    : local=0x{local_crc:08X}, device=0x{device_crc:08X}")

    print("-" * 72)
    print(f"Recording round {round_number} complete")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def default_env_file_path() -> str:
    return str(Path(__file__).with_suffix(".env"))


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    if argv is None:
        argv = sys.argv[1:]

    env_probe = argparse.ArgumentParser(add_help=False)
    env_probe.add_argument("--env-file", default=os.environ.get("BCM_ENV_FILE", default_env_file_path()))
    env_probe.add_argument("--no-env-file", action="store_true")
    env_args, _ = env_probe.parse_known_args(argv)

    env_values: Dict[str, str] = {}
    env_file_loaded = False
    if not env_args.no_env_file:
        env_path = Path(env_args.env_file)
        env_values = load_env_file(env_path)
        env_file_loaded = bool(env_values)

    # OS environment overrides env file defaults.
    merged_env = dict(env_values)
    merged_env.update(os.environ)

    parser = argparse.ArgumentParser(
        description="Read and CRC-check Balluff BCM raw acceleration BLOBs through ICE2/3 Modbus TCP.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--env-file", default=env_args.env_file)
    parser.add_argument("--no-env-file", action="store_true", default=env_args.no_env_file)

    parser.add_argument("--host", default=env_str(merged_env, "BCM_HOST", "192.168.137.5"))
    parser.add_argument("--tcp-port", type=int, default=env_int(merged_env, "BCM_TCP_PORT", 502))
    parser.add_argument("--unit-id", type=int, default=env_int(merged_env, "BCM_UNIT_ID", 1))
    parser.add_argument("--iol-port", type=int, default=env_int(merged_env, "BCM_IOL_PORT", 1))

    parser.add_argument("--trigger-source", type=int, default=env_int(merged_env, "BCM_TRIGGER_SOURCE", 2))
    parser.add_argument("--trigger-mode", type=int, default=env_int(merged_env, "BCM_TRIGGER_MODE", 0))
    parser.add_argument("--axis-selection", type=int, default=env_int(merged_env, "BCM_AXIS_SELECTION", 6))
    parser.add_argument("--target", type=int, default=env_int(merged_env, "BCM_TARGET", 0))

    parser.add_argument("--max-isdu-len", type=int, default=env_int(merged_env, "BCM_MAX_ISDU_LEN", 232))
    parser.add_argument("--isdu-delay-s", type=float, default=env_float(merged_env, "BCM_ISDU_DELAY_S", 0.1))
    parser.add_argument(
        "--blob-segment-gap-s",
        type=float,
        default=env_float(merged_env, "BCM_BLOB_SEGMENT_GAP_S", 0.01),
        help=(
            "Quiet time before each BLOB_CH read request. This paces Balluff BCM BLOB "
            "segments without retrying index 50 or accepting duplicate packets."
        ),
    )
    parser.add_argument(
        "--blob-response-poll-s",
        type=float,
        default=env_float(merged_env, "BCM_BLOB_RESPONSE_POLL_S", 0.01),
        help="Polling interval for re-reading the ICE ISDU response mailbox while waiting for a fresh BLOB_CH response.",
    )
    parser.add_argument(
        "--blob-response-timeout-s",
        type=float,
        default=env_float(merged_env, "BCM_BLOB_RESPONSE_TIMEOUT_S", 5.0),
        help="Maximum time to wait for the ICE ISDU response mailbox to change after one BLOB_CH read request.",
    )
    parser.add_argument(
        "--unsafe-blob-read-retries",
        "--blob-read-retries",
        dest="unsafe_blob_read_retries",
        type=int,
        default=env_int(
            merged_env,
            "BCM_UNSAFE_BLOB_READ_RETRIES",
            env_int(merged_env, "BCM_BLOB_READ_RETRIES", 0),
        ),
        help=(
            "UNSAFE diagnostic only: retry the stateful BLOB_CH read request after a failed "
            "ICE mailbox result such as 0x3001. The Balluff manual says BLOB_CH reads trigger "
            "internal state changes and cannot be performed twice, so keep this 0 for normal use."
        ),
    )
    parser.add_argument(
        "--blob-read-retry-delay-s",
        type=float,
        default=env_float(merged_env, "BCM_BLOB_READ_RETRY_DELAY_S", 0.5),
        help=(
            "Delay before retrying a failed BLOB_CH read request after a 0x3001/timeout mailbox result. "
            "For BCM0003 through ICE2/ICE3, keep this conservative; too-fast retries can keep the "
            "master/device stuck returning 0x3001 with the old BLOB_Info or segment payload."
        ),
    )
    parser.add_argument(
        "--disable-blob-response-freshness",
        action="store_true",
        default=env_bool(merged_env, "BCM_DISABLE_BLOB_RESPONSE_FRESHNESS", False),
        help=(
            "UNSAFE diagnostic only: disable the mailbox freshness barrier and use the older "
            "single-wait BLOB_CH read behavior. Requires --unsafe-allow-disable-freshness."
        ),
    )
    parser.add_argument(
        "--unsafe-allow-disable-freshness",
        action="store_true",
        default=env_bool(merged_env, "BCM_UNSAFE_ALLOW_DISABLE_FRESHNESS", False),
        help="Allow --disable-blob-response-freshness. Keep false for normal use.",
    )

    parser.add_argument(
        "--max-duplicate-rereads",
        type=int,
        default=env_int(merged_env, "BCM_MAX_DUPLICATE_REREADS", 20),
        help=(
            "Maximum exact duplicate BLOB segments to ignore/re-read while waiting for the next "
            "flow. The duplicate must have the same flow and identical body."
        ),
    )
    parser.add_argument(
        "--flow-policy",
        choices=sorted(VALID_FLOW_POLICIES),
        default=env_str(merged_env, "BCM_FLOW_POLICY", FLOW_POLICY_STRICT),
        help=(
            "BLOB flow-counter handling. With the v4 fresh-response barrier, 'strict' "
            "is recommended first. Use 'warn' only for diagnosis after checking CRC behavior."
        ),
    )
    parser.add_argument("--modbus-timeout-s", type=float, default=env_float(merged_env, "BCM_MODBUS_TIMEOUT_S", 5.0))
    parser.add_argument("--modbus-retries", type=int, default=env_int(merged_env, "BCM_MODBUS_RETRIES", 2))
    parser.add_argument(
        "--modbus-retry-delay-s",
        type=float,
        default=env_float(merged_env, "BCM_MODBUS_RETRY_DELAY_S", 0.05),
    )

    parser.add_argument("--status-timeout-s", type=float, default=env_float(merged_env, "BCM_STATUS_TIMEOUT_S", 120.0))
    parser.add_argument("--status-poll-s", type=float, default=env_float(merged_env, "BCM_STATUS_POLL_S", 0.25))
    parser.add_argument("--blob-timeout-s", type=float, default=env_float(merged_env, "BCM_BLOB_TIMEOUT_S", 900.0))

    parser.add_argument("--output-dir", default=env_str(merged_env, "BCM_OUTPUT_DIR", "blob_output"))
    parser.add_argument("--once", action="store_true", default=env_bool(merged_env, "BCM_ONCE", False))
    parser.add_argument("--skip-config", action="store_true", default=env_bool(merged_env, "BCM_SKIP_CONFIG", False))
    parser.add_argument("--no-startup-recovery", action="store_true", default=env_bool(merged_env, "BCM_NO_STARTUP_RECOVERY", False))
    parser.add_argument("--debug", action="store_true", default=env_bool(merged_env, "BCM_DEBUG", False))
    parser.add_argument("--log-file", default=env_str(merged_env, "BCM_LOG_FILE", ""))

    args = parser.parse_args(argv)
    args.env_file_loaded = env_file_loaded
    return args


def configure_logging(debug: bool, log_file: str = "") -> None:
    handlers: List[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(log_path, encoding="utf-8"))

    logging.basicConfig(
        level=logging.DEBUG if debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=handlers,
    )


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    configure_logging(args.debug, args.log_file)

    if not args.no_env_file:
        print(f"Using env file: {args.env_file}")

    if args.target not in SUPPORTED_TARGETS:
        print(
            f"ERROR: target={args.target} is not supported by this script. "
            "This script currently supports only target=0 (raw acceleration data). "
            "Targets 1 and 2 include spectrum payloads and need a different parser.",
            file=sys.stderr,
        )
        return 2

    if args.unsafe_blob_read_retries > 0:
        print(
            "WARNING: --unsafe-blob-read-retries is greater than 0. BLOB_CH reads are "
            "state-changing; retrying a failed read can skip or consume a segment.",
            file=sys.stderr,
        )

    if args.disable_blob_response_freshness and not args.unsafe_allow_disable_freshness:
        print(
            "ERROR: --disable-blob-response-freshness is unsafe for this hardware path. "
            "Add --unsafe-allow-disable-freshness only for controlled diagnostics.",
            file=sys.stderr,
        )
        return 2

    axes = selected_axes_from_axis_selection(args.axis_selection)
    output_dir = Path(args.output_dir)

    print(f"Connecting to ICE2/3 Modbus TCP {args.host}:{args.tcp_port}, unit_id={args.unit_id}")
    print(f"IO-Link port: {args.iol_port}")
    print(f"ISDU delay: {args.isdu_delay_s:.3f}s")
    print(f"Max BLOB_CH ISDU length: {args.max_isdu_len} bytes")
    print(f"Expected normal segment body length: {args.max_isdu_len - 1} bytes")
    print(f"BLOB segment quiet gap: {args.blob_segment_gap_s:.3f}s")
    print(f"BLOB response poll: {args.blob_response_poll_s:.3f}s")
    print(f"BLOB response fresh timeout: {args.blob_response_timeout_s:.3f}s")
    print(f"UNSAFE BLOB read retries after 0x3001/timeout: {args.unsafe_blob_read_retries} x {args.blob_read_retry_delay_s:.3f}s")
    print(f"BLOB response freshness barrier: {not args.disable_blob_response_freshness}")
    print(f"Max duplicate re-reads: {args.max_duplicate_rereads}")
    print(f"Flow-counter policy: {args.flow_policy}")
    print(f"Modbus retries: {args.modbus_retries} x {args.modbus_retry_delay_s:.3f}s")

    typical_blob_len = 269_515
    typical_body_len = max(args.max_isdu_len - 1, 1)
    typical_reads = math.ceil(typical_blob_len / typical_body_len)
    typical_wait_floor_s = typical_reads * (args.isdu_delay_s + args.blob_segment_gap_s)
    print(
        f"Typical 269515-byte BLOB configured-wait floor: ~{typical_wait_floor_s:.1f}s "
        f"for ~{typical_reads} reads (transport overhead not included)"
    )
    if typical_wait_floor_s >= 330.0:
        print(
            "WARNING: configured waits alone approach six minutes. A large BCM raw BLOB may "
            "terminate before completion. Reduce BCM_ISDU_DELAY_S and/or "
            "BCM_BLOB_SEGMENT_GAP_S; do not enable unsafe BLOB_CH retries.",
            file=sys.stderr,
        )

    reader = Ice2BalluffBcmBlobReader(
        host=args.host,
        tcp_port=args.tcp_port,
        unit_id=args.unit_id,
        iol_port=args.iol_port,
        max_isdu_len=args.max_isdu_len,
        isdu_delay_s=args.isdu_delay_s,
        blob_segment_gap_s=args.blob_segment_gap_s,
        blob_response_poll_s=args.blob_response_poll_s,
        blob_response_timeout_s=args.blob_response_timeout_s,
        blob_read_retries=args.unsafe_blob_read_retries,
        blob_read_retry_delay_s=args.blob_read_retry_delay_s,
        disable_blob_response_freshness=args.disable_blob_response_freshness,
        max_duplicate_rereads=args.max_duplicate_rereads,
        modbus_timeout_s=args.modbus_timeout_s,
        modbus_retries=args.modbus_retries,
        modbus_retry_delay_s=args.modbus_retry_delay_s,
        flow_policy=args.flow_policy,
        debug=args.debug,
    )

    try:
        try:
            print(f"Initial BLOB_ID: {reader.read_blob_id()}")
        except Exception as exc:
            print(f"Could not read initial BLOB_ID: {exc}")

        if not args.no_startup_recovery:
            reader.startup_blob_recovery()

        round_number = 1
        while True:
            run_recording_round(
                reader,
                round_number=round_number,
                axes=axes,
                trigger_source=args.trigger_source,
                trigger_mode=args.trigger_mode,
                axis_selection=args.axis_selection,
                target=args.target,
                status_timeout_s=args.status_timeout_s,
                status_poll_s=args.status_poll_s,
                blob_timeout_s=args.blob_timeout_s,
                output_dir=output_dir,
                skip_config=args.skip_config,
            )

            if args.once:
                break

            user_input = input(
                "\nPress Enter to record/transfer the next round, or type q then Enter to quit: "
            ).strip().lower()
            if user_input in {"q", "quit", "exit"}:
                break
            round_number += 1

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user. Sending BLOB_Abort for safety...")
        try:
            reader.blob_abort()
            print("BLOB_Abort sent")
        except Exception as exc:
            print(f"Could not send BLOB_Abort: {exc}")
        return 130

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())