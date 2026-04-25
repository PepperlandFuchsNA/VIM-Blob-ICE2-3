from __future__ import annotations

import csv
import logging
import os
import struct
from typing import Any, Iterable, Optional

try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    pass

import modbus_ISDU as ISDU

logger = logging.getLogger(__name__)


# ============================================================
# Connection configuration
# ============================================================
# You can override these without editing the file:
#   Windows PowerShell:
#       $env:ICE_HOST="192.168.137.21"
#   CMD:
#       set ICE_HOST=192.168.137.21
#
# Defaults match your current test setup.

ICE_HOST = os.getenv("ICE_HOST", "192.168.137.21")
ICE_IOL_PORT = int(os.getenv("ICE_IOL_PORT", "1"))
ICE_TCP_PORT = int(os.getenv("ICE_TCP_PORT", "502"))
ICE_UNIT_ID = int(os.getenv("ICE_UNIT_ID", "1"))

ICE_ISDU = ISDU.ISDU(
    host=ICE_HOST,
    iol_port=ICE_IOL_PORT,
    tcp_port=ICE_TCP_PORT,
    unit_id=ICE_UNIT_ID,
)


def set_ice_isdu_client(
    host=ICE_HOST,
    iol_port=ICE_IOL_PORT,
    tcp_port=ICE_TCP_PORT,
    unit_id=ICE_UNIT_ID,
    timeout_s: float = ISDU.DEFAULT_TIMEOUT_S,
    poll_interval_s: float = ISDU.DEFAULT_POLL_INTERVAL_S,
    response_registers: int = ISDU.DEFAULT_RESPONSE_REGISTERS,
    base0: bool = ISDU.DEFAULT_BASE0,
):
    """
    Recreate the global ISDU client for a different ICE IO-Link port.

    iol_port = physical IO-Link port number on the ICE master.
    tcp_port = Modbus/TCP port, usually 502.
    """
    global ICE_HOST, ICE_IOL_PORT, ICE_TCP_PORT, ICE_UNIT_ID, ICE_ISDU

    ICE_HOST = str(host)
    ICE_IOL_PORT = int(iol_port)
    ICE_TCP_PORT = int(tcp_port)
    ICE_UNIT_ID = int(unit_id)

    ICE_ISDU = ISDU.ISDU(
        host=ICE_HOST,
        iol_port=ICE_IOL_PORT,
        tcp_port=ICE_TCP_PORT,
        unit_id=ICE_UNIT_ID,
        timeout_s=timeout_s,
        poll_interval_s=poll_interval_s,
        response_registers=response_registers,
        base0=base0,
    )

    logger.info(
        "ICE ISDU client set: host=%s, iol_port=%s, tcp_port=%s, unit_id=%s",
        ICE_HOST, ICE_IOL_PORT, ICE_TCP_PORT, ICE_UNIT_ID,
    )

    return ICE_ISDU


def get_ice_connection_info() -> dict[str, int | str]:
    return {
        "host": ICE_HOST,
        "iol_port": ICE_IOL_PORT,
        "tcp_port": ICE_TCP_PORT,
        "unit_id": ICE_UNIT_ID,
    }

# ============================================================
# Balluff BCM BLOB indexes / constants
# ============================================================

BLOB_CHANNEL_INDEX = 50          # 0x0032 BLOB_CH
BLOB_CHANNEL_SUBINDEX = 0x00

DATA_PROVIDER_CONFIGURATION_INDEX = 8603  # 0x219B
DATA_PROVIDER_TRIGGER_INDEX = 8605        # 0x219D
DATA_PROVIDER_RESTART_INDEX = 8606        # 0x219E
DATA_PROVIDER_STATUS_INDEX = 8607         # 0x219F

# BLOB command bytes
BLOB_ABORT_COMMAND = 0xF0
BLOB_START_COMMAND = 0xF1
BLOB_FINISH_COMMAND = 0xF2

# Positive IDs are used for parsing/export labels.
BLOB_TYPE_TO_ID = {
    "rawX": 4096,
    "rawY": 4097,
    "rawZ": 4098,
    "AmpSpecX": 4099,
    "AmpSpecY": 4100,
    "AmpSpecZ": 4101,
    "EnvSpecX": 4102,
    "EnvSpecY": 4103,
    "EnvSpecZ": 4104,
}

# Negative IDs are used inside the BLOB_Start command when reading the BLOB.
# Example rawX: -4096 = 0xF000 as signed INT16.
BLOB_TYPE_TO_READ_ID = {
    blob_type: -blob_id
    for blob_type, blob_id in BLOB_TYPE_TO_ID.items()
}

# Status byte positions when reading index 8607, subindex 0, length 9.
# Manual layout:
#   subindex 1 rawX, 2 rawY, 3 rawZ,
#   subindex 4 AmpSpecX, 5 AmpSpecY, 6 AmpSpecZ,
#   subindex 7 EnvSpecX, 8 EnvSpecY, 9 EnvSpecZ.
BLOB_TYPE_TO_STATUS_SUBINDEX = {
    "rawX": 1,
    "rawY": 2,
    "rawZ": 3,
    "AmpSpecX": 4,
    "AmpSpecY": 5,
    "AmpSpecZ": 6,
    "EnvSpecX": 7,
    "EnvSpecY": 8,
    "EnvSpecZ": 9,
}

BLOB_STATUS_TEXT = {
    0: "Data collection disabled",
    1: "Waiting for trigger",
    2: "Data provider preparing data",
    3: "Data ready for BLOB transfer",
}


# ============================================================
# ISDU response helpers
# ============================================================

def response_to_bytes(response: Any, word_byte_order: str = "big") -> bytes:
    """
    Convert different response styles into a clean bytes object.

    Supports:
      - modbus_ISDU.ISDUResponse objects
      - bytes / bytearray
      - list[int] containing byte values
      - list[int] containing Modbus 16-bit register values
      - older full response lists:
          [control_word, index, subindex, byte_count, data_registers...]
    """

    if response is None:
        return b""

    # New modbus_ISDU.ISDUResponse path.
    if hasattr(response, "data"):
        return bytes(response.data)

    if isinstance(response, bytes):
        return response

    if isinstance(response, bytearray):
        return bytes(response)

    if not isinstance(response, list):
        raise ValueError(f"Unsupported response type: {type(response)}")

    if not response:
        return b""

    # Older full ISDU response list.
    if len(response) >= 5:
        possible_byte_count = response[3]
        data_words = response[4:]

        if isinstance(possible_byte_count, int) and 0 <= possible_byte_count <= 232:
            payload = bytearray()

            for word in data_words:
                payload.extend(
                    int(word).to_bytes(
                        2,
                        byteorder=word_byte_order,
                        signed=False,
                    )
                )

            return bytes(payload[:possible_byte_count])

    # Already byte values.
    if all(0 <= int(x) <= 0xFF for x in response):
        return bytes(int(x) for x in response)

    # Otherwise treat as 16-bit Modbus registers.
    payload = bytearray()

    for word in response:
        payload.extend(
            int(word).to_bytes(
                2,
                byteorder=word_byte_order,
                signed=False,
            )
        )

    return bytes(payload)


def response_to_unsigned_int(response: Any, byteorder: str = "big") -> int:
    data = response_to_bytes(response)
    if not data:
        return 0
    return int.from_bytes(data, byteorder=byteorder, signed=False)


def response_to_signed_int(response: Any, byteorder: str = "big") -> int:
    data = response_to_bytes(response)
    if not data:
        return 0
    return int.from_bytes(data, byteorder=byteorder, signed=True)


# ============================================================
# Balluff BLOB control functions
# ============================================================

def Read_Blob_Status(length: int = 9):
    """
    Read Balluff data provider status from index 8607, subindex 0.

    length=9 returns the full status array:
      rawX/rawY/rawZ, AmpSpecX/Y/Z, EnvSpecX/Y/Z.
    """
    return ICE_ISDU.read_isdu(
        index=DATA_PROVIDER_STATUS_INDEX,
        subindex=0x00,
        length=length,
    )


def Read_Blob_Status_Bytes() -> bytes:
    data = response_to_bytes(Read_Blob_Status(length=9))

    if len(data) < 9:
        raise ValueError(
            f"Expected 9 BLOB status bytes from index {DATA_PROVIDER_STATUS_INDEX}, "
            f"got {len(data)} bytes: {data.hex(' ').upper()}"
        )

    return data[:9]


def Read_Blob_Status_Map() -> dict[str, int]:
    data = Read_Blob_Status_Bytes()

    return {
        blob_type: data[subindex - 1]
        for blob_type, subindex in BLOB_TYPE_TO_STATUS_SUBINDEX.items()
    }


def Format_Blob_Status_Map(status_map: dict[str, int]) -> str:
    parts = []

    for blob_type, status in status_map.items():
        text = BLOB_STATUS_TEXT.get(status, f"Unknown status {status}")
        parts.append(f"{blob_type}={status}({text})")

    return ", ".join(parts)


def _validate_config_value(name: str, value: int, allowed: set[int]) -> int:
    value_i = int(value)
    if value_i not in allowed:
        raise ValueError(f"{name} must be one of {sorted(allowed)}; got {value_i}")
    return value_i


def validate_blob_configuration(
    DPTG_value: int,
    RADPTM_value: int,
    DCAS_value: int,
    DCT_value: int,
) -> tuple[int, int, int, int]:
    return (
        _validate_config_value("DPTG_value", DPTG_value, {0, 1, 2, 3, 4}),
        _validate_config_value("RADPTM_value", RADPTM_value, {0, 1, 2}),
        _validate_config_value("DCAS_value", DCAS_value, {0, 1, 2, 3, 4, 5, 6}),
        _validate_config_value("DCT_value", DCT_value, {0, 1, 2}),
    )


def Sensor_Blob_Configuration(
    DPTG_value: int = 2,
    RADPTM_value: int = 0,
    DCAS_value: int = 6,
    DCT_value: int = 0,
    wait_for_response: bool = False,
):
    """
    Configure Balluff BCM data provider.

    DPTG:
      0 = disabled
      1 = triggered by process data
      2 = triggered by ISDU
      3 = triggered by Pin 2
      4 = triggered by alarm signal or logic block

    RADPTM:
      0 = raw acceleration starts after trigger
      1 = starts before and ends after trigger
      2 = ends with trigger

    DCAS:
      0 = X
      1 = Y
      2 = Z
      3 = X and Y
      4 = X and Z
      5 = Y and Z
      6 = X, Y and Z

    DCT:
      0 = raw acceleration only
      1 = amplitude/envelope spectrum only
      2 = raw acceleration + amplitude/envelope spectrum
    """

    DPTG_value, RADPTM_value, DCAS_value, DCT_value = validate_blob_configuration(
        DPTG_value=DPTG_value,
        RADPTM_value=RADPTM_value,
        DCAS_value=DCAS_value,
        DCT_value=DCT_value,
    )

    values = {
        0x01: DPTG_value,
        0x02: RADPTM_value,
        0x03: DCAS_value,
        0x04: DCT_value,
    }

    for subindex, value in values.items():
        ICE_ISDU.write_isdu(
            index=DATA_PROVIDER_CONFIGURATION_INDEX,
            subindex=subindex,
            data=value,
            data_length=1,
            wait_for_response=wait_for_response,
        )

    return (
        "Sensor BLOB configuration set: "
        f"DPTG_value={DPTG_value}, "
        f"RADPTM_value={RADPTM_value}, "
        f"DCAS_value={DCAS_value}, "
        f"DCT_value={DCT_value}"
    )


def Trigger_Start_Collection(wait_for_response: bool = False):
    """
    Trigger data collection by writing 1 to index 8605, subindex 0.
    """
    ICE_ISDU.write_isdu(
        index=DATA_PROVIDER_TRIGGER_INDEX,
        subindex=0x00,
        data=1,
        data_length=1,
        wait_for_response=wait_for_response,
    )

    return "ISDU trigger sent. The sensor should start collecting data."




def Restart_Raw_Data_Feature(wait_for_response: bool = False):
    """
    Restart the Balluff raw/spectrum data provider feature.

    This is useful before a new acquisition or after a failed/interrupted run.
    """
    ICE_ISDU.write_isdu(
        index=DATA_PROVIDER_RESTART_INDEX,
        subindex=0x00,
        data=1,
        data_length=1,
        wait_for_response=wait_for_response,
    )

    return "Raw data feature restart command sent."

def Get_Blob_Data_cmd(Type: str = "rawX", wait_for_response: bool = False):
    """
    Send BLOB_Start to BLOB_CH index 50.

    Payload format:
      Byte 0: 0xF1
      Byte 1-2: signed INT16 read BLOB_ID, big-endian

    Example:
      rawX read id = -4096 = F0 00
      payload = F1 F0 00
    """
    if Type not in BLOB_TYPE_TO_READ_ID:
        raise ValueError(f"Unsupported BLOB type: {Type}")

    read_blob_id = BLOB_TYPE_TO_READ_ID[Type]

    payload = bytes([BLOB_START_COMMAND]) + int(read_blob_id).to_bytes(
        2,
        byteorder="big",
        signed=True,
    )

    ICE_ISDU.write_isdu(
        index=BLOB_CHANNEL_INDEX,
        subindex=BLOB_CHANNEL_SUBINDEX,
        data=payload,
        data_length=len(payload),
        wait_for_response=wait_for_response,
    )

    return (
        f"BLOB_Start sent for {Type}. "
        f"Read BLOB_ID={read_blob_id}, payload={payload.hex(' ').upper()}."
    )


def Write_Blob_Abort(wait_for_response: bool = False):
    """
    Send BLOB_Abort to BLOB_CH index 50.

    Use this as recovery when a BLOB transfer times out or is interrupted.
    """
    payload = bytes([BLOB_ABORT_COMMAND])

    ICE_ISDU.write_isdu(
        index=BLOB_CHANNEL_INDEX,
        subindex=BLOB_CHANNEL_SUBINDEX,
        data=payload,
        data_length=1,
        wait_for_response=wait_for_response,
    )

    return f"BLOB_Abort sent. Payload={payload.hex(' ').upper()}."


def Write_Blob_Finish(wait_for_response: bool = False):
    """
    Send BLOB_Finish to BLOB_CH index 50.
    """
    payload = bytes([BLOB_FINISH_COMMAND])

    ICE_ISDU.write_isdu(
        index=BLOB_CHANNEL_INDEX,
        subindex=BLOB_CHANNEL_SUBINDEX,
        data=payload,
        data_length=1,
        wait_for_response=wait_for_response,
    )

    return f"BLOB_Finish sent. Payload={payload.hex(' ').upper()}."


def Read_Blob_Data(length: int = 232):
    """
    Read BLOB_CH index 50.

    A read of this channel advances the BLOB stream state, so do not call this
    casually while a transfer is active.
    """
    return ICE_ISDU.read_isdu(
        index=BLOB_CHANNEL_INDEX,
        subindex=BLOB_CHANNEL_SUBINDEX,
        length=length,
    )


def Read_Blob_Data_Bytes(length: int = 232) -> bytes:
    return response_to_bytes(Read_Blob_Data(length=length))


# ============================================================
# CSV / parser helpers
# ============================================================

RAW_BLOB_IDS = {
    4096: "X",
    4097: "Y",
    4098: "Z",
}

AMPLITUDE_BLOB_IDS = {
    4099: "X",
    4100: "Y",
    4101: "Z",
}

ENVELOPE_BLOB_IDS = {
    4102: "X",
    4103: "Y",
    4104: "Z",
}


def _data_to_bytes(data: Any, word_byte_order: str = "big") -> bytes:
    """
    Converts either:
      - bytes / bytearray
      - ISDUResponse object
      - list of 8-bit integers
      - list of 16-bit integers from PLC/master

    into a continuous byte stream.
    """
    if hasattr(data, "data"):
        return bytes(data.data)

    if isinstance(data, (bytes, bytearray)):
        return bytes(data)

    data = list(data)

    if not data:
        return b""

    if all(0 <= int(x) <= 0xFF for x in data):
        return bytes(int(x) for x in data)

    payload = bytearray()

    for value in data:
        value_int = int(value)

        if not 0 <= value_int <= 0xFFFF:
            raise ValueError(f"Value out of 16-bit range: {value_int}")

        payload.extend(
            value_int.to_bytes(
                2,
                byteorder=word_byte_order,
                signed=False,
            )
        )

    return bytes(payload)


def _write_rows_to_csv(csv_path: Optional[str], fieldnames: list[str], rows: list[dict]) -> None:
    if not csv_path:
        return

    folder = os.path.dirname(csv_path)
    if folder:
        os.makedirs(folder, exist_ok=True)

    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0

    with open(csv_path, "a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)

        if write_header:
            writer.writeheader()

        writer.writerows(rows)


def parse_balluff_raw_acceleration_blob(
    data: Any,
    blob_id: Optional[int] = None,
    axis: Optional[str] = None,
    word_byte_order: str = "big",
    object_endian: str = "big",
    csv_path: Optional[str] = "balluff_raw_acceleration.csv",
    print_values: bool = True,
):
    """
    Parser for Balluff BCM raw acceleration BLOB payload.

    Payload layout:
      Byte 1-4: UINT32 timestamp of trigger signal, unit ms
      Byte 5-6 onward: repeated INT16 raw acceleration samples

    Scaling:
      physical value in mg = raw_int16 * 0.488
      physical value in g  = mg / 1000
    """
    payload = _data_to_bytes(data, word_byte_order=word_byte_order)

    if blob_id is not None:
        axis = RAW_BLOB_IDS.get(blob_id, axis)

    if axis is None:
        axis = "UNKNOWN"

    if len(payload) < 6:
        raise ValueError(
            f"Raw acceleration payload too short. Need at least 6 bytes, got {len(payload)}."
        )

    if (len(payload) - 4) % 2 != 0:
        raise ValueError(
            f"Invalid raw acceleration payload length: {len(payload)} bytes. "
            "After the 4-byte timestamp, the remaining bytes must be divisible by 2."
        )

    endian_prefix = ">" if object_endian == "big" else "<"

    timestamp_ms = struct.unpack_from(endian_prefix + "I", payload, 0)[0]

    rows = []

    for sample_index, offset in enumerate(range(4, len(payload), 2)):
        raw_int16 = struct.unpack_from(endian_prefix + "h", payload, offset)[0]
        accel_mg = raw_int16 * 0.488
        accel_g = accel_mg / 1000.0

        rows.append({
            "timestamp_ms": timestamp_ms,
            "axis": axis,
            "sample_index": sample_index,
            "raw_int16": raw_int16,
            "accel_mg": accel_mg,
            "accel_g": accel_g,
        })

    if print_values:
        print("Raw Acceleration BLOB")
        print(f"Timestamp: {timestamp_ms} ms")
        print(f"Axis: {axis}")
        print(f"Samples: {len(rows)}")
        print(f"First 10 values in mg: {[round(r['accel_mg'], 4) for r in rows[:10]]}")
        print(f"First 10 values in g: {[round(r['accel_g'], 6) for r in rows[:10]]}")

    _write_rows_to_csv(
        csv_path,
        ["timestamp_ms", "axis", "sample_index", "raw_int16", "accel_mg", "accel_g"],
        rows,
    )

    return {
        "timestamp_ms": timestamp_ms,
        "axis": axis,
        "sample_count": len(rows),
        "samples": rows,
    }


def parse_balluff_spectrum_blob(
    data: Any,
    blob_id: Optional[int] = None,
    axis: Optional[str] = None,
    spectrum_type: Optional[str] = None,
    frequency_resolution_hz: Optional[float] = None,
    word_byte_order: str = "big",
    object_endian: str = "big",
    csv_path: Optional[str] = "balluff_spectrum.csv",
    print_values: bool = True,
):
    """
    Parser for Balluff BCM amplitude/envelope spectrum BLOB payload.

    Payload layout:
      Byte 1-4: UINT32 timestamp of trigger signal, unit ms
      Byte 5-8 onward: repeated FLOAT32 spectrum values, unit g
    """
    payload = _data_to_bytes(data, word_byte_order=word_byte_order)

    if blob_id is not None:
        if blob_id in AMPLITUDE_BLOB_IDS:
            axis = AMPLITUDE_BLOB_IDS[blob_id]
            spectrum_type = "amplitude"
        elif blob_id in ENVELOPE_BLOB_IDS:
            axis = ENVELOPE_BLOB_IDS[blob_id]
            spectrum_type = "envelope"

    if axis is None:
        axis = "UNKNOWN"

    if spectrum_type is None:
        spectrum_type = "UNKNOWN"

    if len(payload) < 8:
        raise ValueError(
            f"Spectrum payload too short. Need at least 8 bytes, got {len(payload)}."
        )

    if (len(payload) - 4) % 4 != 0:
        raise ValueError(
            f"Invalid spectrum payload length: {len(payload)} bytes. "
            "After the 4-byte timestamp, the remaining bytes must be divisible by 4."
        )

    endian_prefix = ">" if object_endian == "big" else "<"

    timestamp_ms = struct.unpack_from(endian_prefix + "I", payload, 0)[0]

    rows = []

    for bin_index, offset in enumerate(range(4, len(payload), 4)):
        value_g = struct.unpack_from(endian_prefix + "f", payload, offset)[0]

        row = {
            "timestamp_ms": timestamp_ms,
            "axis": axis,
            "spectrum_type": spectrum_type,
            "bin_index": bin_index,
            "value_g": value_g,
        }

        if frequency_resolution_hz is not None:
            row["frequency_hz"] = bin_index * float(frequency_resolution_hz)

        rows.append(row)

    fieldnames = ["timestamp_ms", "axis", "spectrum_type", "bin_index", "value_g"]

    if frequency_resolution_hz is not None:
        fieldnames.append("frequency_hz")

    if print_values:
        print(f"{spectrum_type.title()} Spectrum BLOB")
        print(f"Timestamp: {timestamp_ms} ms")
        print(f"Axis: {axis}")
        print(f"Bins: {len(rows)}")
        print(f"First 10 values in g: {[round(r['value_g'], 6) for r in rows[:10]]}")

    _write_rows_to_csv(csv_path, fieldnames, rows)

    return {
        "timestamp_ms": timestamp_ms,
        "axis": axis,
        "spectrum_type": spectrum_type,
        "bin_count": len(rows),
        "bins": rows,
    }


def parse_balluff_blob(
    data: Any,
    blob_id: int,
    word_byte_order: str = "big",
    object_endian: str = "big",
    csv_path: Optional[str] = None,
    frequency_resolution_hz: Optional[float] = None,
    print_values: bool = True,
):
    """
    Auto-select parser based on Balluff positive BLOB_ID.
    """
    if blob_id in RAW_BLOB_IDS:
        return parse_balluff_raw_acceleration_blob(
            data=data,
            blob_id=blob_id,
            word_byte_order=word_byte_order,
            object_endian=object_endian,
            csv_path=csv_path or "balluff_raw_acceleration.csv",
            print_values=print_values,
        )

    if blob_id in AMPLITUDE_BLOB_IDS or blob_id in ENVELOPE_BLOB_IDS:
        return parse_balluff_spectrum_blob(
            data=data,
            blob_id=blob_id,
            word_byte_order=word_byte_order,
            object_endian=object_endian,
            csv_path=csv_path or "balluff_spectrum.csv",
            frequency_resolution_hz=frequency_resolution_hz,
            print_values=print_values,
        )

    raise ValueError(f"Unsupported Balluff BLOB_ID: {blob_id}")


if __name__ == "__main__":
    print(f"ICE host: {ICE_HOST}")
    print(f"ICE IO-Link port: {ICE_IOL_PORT}")
    print(f"ICE TCP port: {ICE_TCP_PORT}")
    print(f"ICE unit id: {ICE_UNIT_ID}")

    status_map = Read_Blob_Status_Map()
    print("Current BLOB status:")
    print(Format_Blob_Status_Map(status_map))
