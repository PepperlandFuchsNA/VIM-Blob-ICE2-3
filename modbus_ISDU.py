from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Union

import ModbusClientWrapper as MCW


# =========================
# ICE3 / Modbus defaults
# =========================

DEFAULT_HOST = "192.168.1.250"
DEFAULT_TCP_PORT = 502
DEFAULT_UNIT_ID = 1

# pyModbusTCP uses base-0 register addressing.
DEFAULT_BASE0 = True

# Max Modbus FC3 read is 125 holding registers.
DEFAULT_RESPONSE_REGISTERS = 125

DEFAULT_TIMEOUT_S = 20.0
DEFAULT_POLL_INTERVAL_S = 0.05

# ICE response mailboxes can retain a previous response briefly, especially after writes.
# These defaults make readback verification more tolerant without hiding real failures.
DEFAULT_READ_RETRIES = 2
DEFAULT_READ_RETRY_DELAY_S = 0.25
DEFAULT_POST_WRITE_DELAY_S = 0.05


# =========================
# ISDU command constants
# =========================

TYPE_NOP = 0
TYPE_READ = 1
TYPE_WRITE = 2
TYPE_READ_WRITE_OR = 3
TYPE_READ_WRITE_AND = 4

CONTROL_SINGLE_LAST = 0

BYTE_SWAP_NONE = 0
BYTE_SWAP_WORD_16 = 1
BYTE_SWAP_DWORD_32 = 2

STATUS_NOP = 0
STATUS_IN_PROCESS = 1
STATUS_SUCCESS = 2
STATUS_FAILURE = 3
STATUS_TIMEOUT = 4


IntLike = Union[int, str]
DataLike = Union[int, bytes, bytearray, str, Sequence[int]]


class ISDUError(Exception):
    """Base exception for ICE3 ISDU communication."""


class ISDUCommunicationError(ISDUError):
    """Raised when Modbus communication fails."""


class ISDUTimeoutError(ISDUError):
    """Raised when an ISDU operation does not complete within timeout."""


class ISDURejectedError(ISDUError):
    """Raised when the IO-Link device rejects the ISDU request."""


class ISDUDeviceTimeoutError(ISDUError):
    """Raised when the IO-Link device does not respond to the ISDU request."""


@dataclass(frozen=True)
class ISDUResponse:
    raw_registers: list[int]
    control_word: int
    command_type: int
    control: int
    byte_swap: int
    status: int
    index: int
    subindex: int
    data_length: int
    data: bytes

    @property
    def ok(self) -> bool:
        return self.status == STATUS_SUCCESS

    @property
    def data_hex(self) -> str:
        return self.data.hex(" ").upper()

    @property
    def data_decimal(self) -> int:
        """Unsigned decimal value of response data, MS byte first."""
        return bytes_to_decimal(self.data, signed=False)

    @property
    def data_signed_decimal(self) -> int:
        """Signed decimal value of response data, MS byte first."""
        return bytes_to_decimal(self.data, signed=True)

    @property
    def data_registers(self) -> list[int]:
        return bytes_to_registers(self.data)

    @property
    def control_word_hex(self) -> str:
        return decimal_to_hex(self.control_word, width=4)

    @property
    def index_hex(self) -> str:
        return decimal_to_hex(self.index, width=4)

    @property
    def subindex_hex(self) -> str:
        return decimal_to_hex(self.subindex, width=2)


def hex_to_decimal(value: Union[str, int]) -> int:
    """
    Convert a hex string to decimal.

    Examples:
        hex_to_decimal("3C")       -> 60
        hex_to_decimal("0x3C")     -> 60
        hex_to_decimal("01 5E")    -> 350
        hex_to_decimal("0x01_5E")  -> 350
    """
    if isinstance(value, int):
        return value

    if not isinstance(value, str):
        raise TypeError("hex value must be str or int")

    clean = _clean_hex_string(value)

    if not clean:
        raise ValueError("hex value cannot be empty")

    return int(clean, 16)


def decimal_to_hex(value: int, width: int = 0, prefix: bool = True) -> str:
    """
    Convert decimal int to hex string.

    Examples:
        decimal_to_hex(60)             -> '0x3C'
        decimal_to_hex(60, width=4)    -> '0x003C'
        decimal_to_hex(350, width=4)   -> '0x015E'
    """
    _validate_range("value", value, 0, 0xFFFFFFFF)
    body = f"{value:0{width}X}" if width else f"{value:X}"
    return f"0x{body}" if prefix else body


def bytes_to_decimal(data: Union[bytes, bytearray], signed: bool = False) -> int:
    """
    Convert big-endian bytes to decimal.

    Example:
        bytes_to_decimal(b"\x01\x5E") -> 350
    """
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("data must be bytes or bytearray")

    if len(data) == 0:
        return 0

    return int.from_bytes(bytes(data), byteorder="big", signed=signed)


def make_control_word(
    command_type: int,
    byte_swap: int = BYTE_SWAP_NONE,
    control: int = CONTROL_SINGLE_LAST,
) -> int:
    """
    Build ICE3 ISDU control/type/byte-swap word.

    Bits 0-3:   Type
    Bits 4-7:   Control
    Bits 8-11:  Byte swap mode
    Bits 12-15: Unused for request, status for response
    """
    _validate_range("command_type", command_type, 0, 15)
    _validate_range("control", control, 0, 15)
    _validate_range("byte_swap", byte_swap, 0, 15)

    return (byte_swap << 8) | (control << 4) | command_type


def parse_control_word(control_word: int) -> tuple[int, int, int, int]:
    """
    Return command_type, control, byte_swap, status.
    """
    _validate_range("control_word", control_word, 0, 0xFFFF)

    command_type = control_word & 0x000F
    control = (control_word >> 4) & 0x000F
    byte_swap = (control_word >> 8) & 0x000F
    status = (control_word >> 12) & 0x000F

    return command_type, control, byte_swap, status


def bytes_to_registers(data: bytes) -> list[int]:
    """
    Pack bytes into 16-bit Modbus registers, MS byte first.

    Examples:
        b'\x01'             -> [0x0100]
        b'\x12\x34'        -> [0x1234]
        b'\x12\x34\x56'   -> [0x1234, 0x5600]
    """
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("data must be bytes or bytearray")

    payload = bytes(data)

    if len(payload) % 2:
        payload += b"\x00"

    return [
        (payload[i] << 8) | payload[i + 1]
        for i in range(0, len(payload), 2)
    ]


def registers_to_bytes(registers: Sequence[int], length: Optional[int] = None) -> bytes:
    """
    Unpack 16-bit Modbus registers into bytes, MS byte first.
    """
    output = bytearray()

    for reg in registers:
        _validate_range("register", reg, 0, 0xFFFF)
        output.append((reg >> 8) & 0xFF)
        output.append(reg & 0xFF)

    result = bytes(output)

    if length is not None:
        return result[:length]

    return result


def int_to_bytes(value: int, length: Optional[int] = None) -> bytes:
    """
    Convert an integer to big-endian bytes.

    If length is omitted:
        0x01       -> b'\x01'
        0x1234     -> b'\x12\x34'
        0x12345678 -> b'\x12\x34\x56\x78'
    """
    if not isinstance(value, int):
        raise TypeError("value must be int")

    if value < 0:
        raise ValueError("value must be >= 0")

    if length is None:
        length = max(1, (value.bit_length() + 7) // 8)

    _validate_range("length", length, 1, 232)

    max_value = (1 << (length * 8)) - 1
    if value > max_value:
        raise ValueError(
            f"value {value} does not fit in {length} byte(s); max is {max_value}"
        )

    return value.to_bytes(length, byteorder="big", signed=False)


def hex_string_to_bytes(value: str) -> bytes:
    """
    Convert strings like '01', '0x01', '12 34', '0x12 0x34' into bytes.
    """
    clean = _clean_hex_string(value)

    if not clean:
        return b""

    if len(clean) % 2:
        clean = "0" + clean

    return bytes.fromhex(clean)


def parse_int_auto(value: IntLike, name: str = "value") -> int:
    """
    Parse int or string into decimal int.

    Rules:
        60       -> 60
        "60"     -> 60 decimal
        "0x3C"   -> 60 decimal
        "3C"     -> 60 decimal, because it contains hex letters

    For pure numeric hex values, use the 0x prefix.
    Example: use "0x60" if you mean hex 60, decimal 96.
    """
    if isinstance(value, int):
        return value

    if not isinstance(value, str):
        raise TypeError(f"{name} must be int or str")

    text = value.strip()
    if not text:
        raise ValueError(f"{name} cannot be empty")

    compact = text.replace("_", "").replace(" ", "")

    if compact.lower().startswith("0x"):
        return int(compact, 16)

    # If it contains A-F, treat it as hex.
    if any(ch in "abcdefABCDEF" for ch in compact):
        return int(compact, 16)

    # Otherwise numeric strings are treated as decimal.
    return int(compact, 10)


class ISDU:
    """
    ISDU client for Pepperl+Fuchs ICE3 IO-Link masters over Modbus/TCP.

    Important:
    - Modbus/TCP must be enabled on the ICE3 web interface.
    - iol_port is the physical IO-Link port, 1 through 8.
    - tcp_port is the Modbus/TCP port, normally 502.
    - pyModbusTCP uses base-0 addresses, so base0=True is the default.
    """

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        iol_port: Optional[IntLike] = None,
        tcp_port: Optional[IntLike] = None,
        unit_id: IntLike = DEFAULT_UNIT_ID,
        byte_swap: IntLike = BYTE_SWAP_NONE,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
        response_registers: IntLike = DEFAULT_RESPONSE_REGISTERS,
        base0: bool = DEFAULT_BASE0,
        modbus_timeout_s: Optional[float] = None,
        read_retries: IntLike = DEFAULT_READ_RETRIES,
        read_retry_delay_s: float = DEFAULT_READ_RETRY_DELAY_S,
        post_write_delay_s: float = DEFAULT_POST_WRITE_DELAY_S,
        # Backward-compatible aliases used by older test files:
        port_number: Optional[IntLike] = None,
        port: Optional[IntLike] = None,
    ):
        if iol_port is None:
            iol_port = 1 if port_number is None else port_number

        if tcp_port is None:
            tcp_port = DEFAULT_TCP_PORT if port is None else port

        iol_port_i = parse_int_auto(iol_port, "iol_port")
        tcp_port_i = parse_int_auto(tcp_port, "tcp_port")
        unit_id_i = parse_int_auto(unit_id, "unit_id")
        byte_swap_i = parse_int_auto(byte_swap, "byte_swap")
        response_registers_i = parse_int_auto(response_registers, "response_registers")
        read_retries_i = parse_int_auto(read_retries, "read_retries")

        _validate_range("iol_port", iol_port_i, 1, 8)
        _validate_range("tcp_port", tcp_port_i, 1, 65535)
        _validate_range("unit_id", unit_id_i, 1, 247)
        _validate_range("byte_swap", byte_swap_i, 0, 2)
        _validate_range("response_registers", response_registers_i, 4, 125)
        _validate_range("read_retries", read_retries_i, 0, 10)

        self.host = host
        self.iol_port = iol_port_i
        self.tcp_port = tcp_port_i
        self.unit_id = unit_id_i
        self.byte_swap = byte_swap_i
        self.timeout_s = float(timeout_s)
        self.poll_interval_s = float(poll_interval_s)
        self.response_registers = response_registers_i
        self.base0 = bool(base0)
        self.modbus_timeout_s = None if modbus_timeout_s is None else float(modbus_timeout_s)
        self.read_retries = read_retries_i
        self.read_retry_delay_s = float(read_retry_delay_s)
        self.post_write_delay_s = float(post_write_delay_s)

        self.modbus_client = MCW.ModbusClientWrapper(
            host=host,
            port=tcp_port_i,
            unit_id=unit_id_i,
            timeout=self.modbus_timeout_s,
        )

        # Manual Base-1:
        #   ISDU response: port * 1000 + 101
        #   ISDU request:  port * 1000 + 301
        #
        # Base-0 for pyModbusTCP:
        #   ISDU response: port * 1000 + 100
        #   ISDU request:  port * 1000 + 300
        base_offset = 0 if self.base0 else 1

        self.response_addr = self.iol_port * 1000 + 100 + base_offset
        self.request_addr = self.iol_port * 1000 + 300 + base_offset

    def read_isdu(
        self,
        index: IntLike,
        subindex: IntLike = 0,
        length: IntLike = 232,
        byte_swap: Optional[IntLike] = None,
        timeout_s: Optional[float] = None,
        retry_count: Optional[IntLike] = None,
        retry_delay_s: Optional[float] = None,
    ) -> ISDUResponse:
        """
        Read an ISDU parameter.

        Args:
            index: IO-Link ISDU index. Accepts decimal int, decimal string, or hex string.
                   Examples: 60, "60", "0x3C", "3C".
            subindex: IO-Link ISDU subindex. Accepts decimal int/string or hex string.
            length: Number of bytes requested.
            byte_swap: Optional ISDU byte-swap mode.
            timeout_s: Optional operation timeout.
        """
        index_i = parse_int_auto(index, "index")
        subindex_i = parse_int_auto(subindex, "subindex")
        length_i = parse_int_auto(length, "length")

        _validate_index_subindex_length(index_i, subindex_i, length_i)

        actual_byte_swap = self.byte_swap if byte_swap is None else parse_int_auto(byte_swap, "byte_swap")
        _validate_range("byte_swap", actual_byte_swap, 0, 2)

        control_word = make_control_word(
            command_type=TYPE_READ,
            byte_swap=actual_byte_swap,
            control=CONTROL_SINGLE_LAST,
        )

        request_words = [
            control_word,
            index_i,
            subindex_i,
            length_i,
        ]

        attempts = self.read_retries if retry_count is None else parse_int_auto(retry_count, "retry_count")
        retry_delay = self.read_retry_delay_s if retry_delay_s is None else float(retry_delay_s)
        _validate_range("retry_count", attempts, 0, 10)

        last_error: Optional[ISDUTimeoutError] = None

        for attempt in range(attempts + 1):
            self._write_registers(self.request_addr, request_words)

            try:
                return self._poll_response(
                    expected_type=TYPE_READ,
                    expected_index=index_i,
                    expected_subindex=subindex_i,
                    timeout_s=self.timeout_s if timeout_s is None else float(timeout_s),
                )
            except ISDUTimeoutError as exc:
                last_error = exc

                if attempt >= attempts:
                    break

                time.sleep(retry_delay)

        raise last_error if last_error is not None else ISDUTimeoutError(
            f"Timed out waiting for ISDU read response. "
            f"index={index_i}, subindex={subindex_i}"
        )

    def write_isdu(
        self,
        index: IntLike,
        subindex: IntLike,
        data: DataLike,
        data_length: Optional[IntLike] = None,
        byte_swap: Optional[IntLike] = None,
        timeout_s: Optional[float] = None,
        verify_echo: bool = False,
        wait_for_response: bool = False,
    ) -> Optional[ISDUResponse]:
        """
        Write an ISDU parameter.

        Args:
            index: IO-Link ISDU index. Accepts decimal int/string or hex string.
            subindex: IO-Link ISDU subindex. Accepts decimal int/string or hex string.
            data:
                - int: converted to big-endian bytes
                - bytes/bytearray: sent exactly as given
                - str: parsed as hex data, e.g. "01", "01 5E", "0x015E"
                - sequence[int]: treated as byte values 0..255
            data_length:
                Optional explicit byte length.
                Strongly recommended for integer writes.

                Example:
                    value 350 as UINT16 -> data=350, data_length=2 sends 01 5E
                    value 1 as UINT16   -> data=1, data_length=2 sends 00 01
                    value 1 as USINT    -> data=1, data_length=1 sends 01
            byte_swap: Optional ISDU byte-swap mode.
            timeout_s: Optional operation timeout.
            verify_echo:
                If True, confirm the response data matches the written bytes.
                Most ICE3 write responses return success with data_length=0, so the default is False.
            wait_for_response:
                Default False. For this ICE3/IO-Link use case, writes are usually accepted
                at the Modbus layer but the ISDU response area can continue showing the
                previous read response. If True, wait for a matching ICE3 ISDU status
                response and confirm status=SUCCESS. If False, return immediately after
                the Modbus write request is accepted by the client.
        """
        index_i = parse_int_auto(index, "index")
        subindex_i = parse_int_auto(subindex, "subindex")

        explicit_length = None if data_length is None else parse_int_auto(data_length, "data_length")
        payload = self._coerce_data_to_bytes(data, data_length=explicit_length)

        _validate_index_subindex_length(index_i, subindex_i, len(payload))

        data_registers = bytes_to_registers(payload)

        return self.write_isdu_registers(
            index=index_i,
            subindex=subindex_i,
            registers=data_registers,
            data_length=len(payload),
            byte_swap=byte_swap,
            timeout_s=timeout_s,
            expected_data=payload if verify_echo else None,
            wait_for_response=wait_for_response,
        )

    def write_isdu_registers(
        self,
        index: IntLike,
        subindex: IntLike,
        registers: Sequence[int],
        data_length: IntLike,
        byte_swap: Optional[IntLike] = None,
        timeout_s: Optional[float] = None,
        expected_data: Optional[bytes] = None,
        wait_for_response: bool = False,
    ) -> Optional[ISDUResponse]:
        """
        Advanced write method when you already have Modbus data registers.

        Example:
            For one byte 0x01 with no byte swap:
                registers=[0x0100], data_length=1

            For UINT16 value 350:
                registers=[0x015E], data_length=2
        """
        index_i = parse_int_auto(index, "index")
        subindex_i = parse_int_auto(subindex, "subindex")
        data_length_i = parse_int_auto(data_length, "data_length")

        _validate_index_subindex_length(index_i, subindex_i, data_length_i)

        if not registers:
            raise ValueError("registers cannot be empty for write")

        for reg in registers:
            _validate_range("register", reg, 0, 0xFFFF)

        if data_length_i > len(registers) * 2:
            raise ValueError(
                f"data_length={data_length_i} is larger than register payload "
                f"capacity={len(registers) * 2} bytes"
            )

        actual_byte_swap = self.byte_swap if byte_swap is None else parse_int_auto(byte_swap, "byte_swap")
        _validate_range("byte_swap", actual_byte_swap, 0, 2)

        control_word = make_control_word(
            command_type=TYPE_WRITE,
            byte_swap=actual_byte_swap,
            control=CONTROL_SINGLE_LAST,
        )

        request_words = [
            control_word,
            index_i,
            subindex_i,
            data_length_i,
            *list(registers),
        ]

        self._write_registers(self.request_addr, request_words)

        if not wait_for_response:
            if self.post_write_delay_s > 0:
                time.sleep(self.post_write_delay_s)
            return None

        return self._poll_response(
            expected_type=TYPE_WRITE,
            expected_index=index_i,
            expected_subindex=subindex_i,
            expected_data=expected_data,
            allow_read_type_for_write=True,
            timeout_s=self.timeout_s if timeout_s is None else float(timeout_s),
        )

    def read_isdu_hex(
        self,
        index: IntLike,
        subindex: IntLike = 0,
        length: IntLike = 232,
    ) -> str:
        """
        Convenience helper that returns only the ISDU data as spaced hex.
        """
        response = self.read_isdu(
            index=index,
            subindex=subindex,
            length=length,
        )
        return response.data_hex

    def read_isdu_decimal(
        self,
        index: IntLike,
        subindex: IntLike = 0,
        length: IntLike = 2,
        signed: bool = False,
    ) -> int:
        """
        Convenience helper that reads an ISDU and returns its data as decimal.

        Example:
            data 01 5E -> 350
        """
        response = self.read_isdu(
            index=index,
            subindex=subindex,
            length=length,
        )
        return bytes_to_decimal(response.data, signed=signed)

    def read_isdu_ascii(
        self,
        index: IntLike,
        subindex: IntLike = 0,
        length: IntLike = 232,
        strip_nulls: bool = True,
    ) -> str:
        """
        Convenience helper for common text parameters.
        """
        response = self.read_isdu(
            index=index,
            subindex=subindex,
            length=length,
        )

        data = response.data

        if strip_nulls:
            data = data.rstrip(b"\x00")

        return data.decode("ascii", errors="replace")

    def _poll_response(
        self,
        expected_type: int,
        expected_index: int,
        expected_subindex: int,
        timeout_s: float,
        expected_data: Optional[bytes] = None,
        allow_read_type_for_write: bool = False,
    ) -> ISDUResponse:
        deadline = time.monotonic() + float(timeout_s)
        last_response: Optional[ISDUResponse] = None

        while time.monotonic() <= deadline:
            response = self._read_response_once()
            last_response = response

            index_matches = response.index == expected_index
            subindex_matches = response.subindex == expected_subindex

            type_matches = response.command_type == expected_type

            # Observed ICE3 behavior:
            # A successful write may return command_type=READ while still returning
            # status=SUCCESS, the requested index/subindex, and the written/readback data.
            if (
                allow_read_type_for_write
                and expected_type == TYPE_WRITE
                and response.command_type in (TYPE_WRITE, TYPE_READ)
            ):
                type_matches = True

            if not (index_matches and subindex_matches and type_matches):
                time.sleep(self.poll_interval_s)
                continue

            if expected_data is not None:
                returned_data = response.data[: len(expected_data)]

                if returned_data != expected_data:
                    time.sleep(self.poll_interval_s)
                    continue

            if response.status == STATUS_SUCCESS:
                return response

            if response.status == STATUS_FAILURE:
                raise ISDURejectedError(
                    f"ISDU request rejected by IO-Link device. "
                    f"index={expected_index}, subindex={expected_subindex}, "
                    f"response={response}"
                )

            if response.status == STATUS_TIMEOUT:
                raise ISDUDeviceTimeoutError(
                    f"IO-Link device timed out during ISDU request. "
                    f"index={expected_index}, subindex={expected_subindex}, "
                    f"response={response}"
                )

            if response.status in (STATUS_NOP, STATUS_IN_PROCESS):
                time.sleep(self.poll_interval_s)
                continue

            raise ISDUError(
                f"Unknown ISDU response status={response.status}. "
                f"index={expected_index}, subindex={expected_subindex}, "
                f"response={response}"
            )

        raise ISDUTimeoutError(
            f"Timed out waiting for ISDU response after {timeout_s} seconds. "
            f"index={expected_index}, subindex={expected_subindex}, "
            f"last_response={last_response}"
        )

    def _read_response_once(self) -> ISDUResponse:
        raw = self._read_registers(self.response_addr, self.response_registers)

        if raw is None:
            raise ISDUCommunicationError(
                f"Failed to read ISDU response registers at {self.response_addr}"
            )

        if len(raw) < 4:
            raise ISDUCommunicationError(
                f"Invalid ISDU response length. "
                f"Expected at least 4 registers, got {len(raw)}"
            )

        control_word = raw[0]
        command_type, control, byte_swap, status = parse_control_word(control_word)

        index = raw[1]
        subindex = raw[2]
        data_length = raw[3]

        data_registers = raw[4:]

        if data_length > len(data_registers) * 2:
            raise ISDUCommunicationError(
                f"Invalid ISDU data_length={data_length}; only "
                f"{len(data_registers) * 2} data byte(s) available in response"
            )

        data = registers_to_bytes(data_registers, length=data_length)

        return ISDUResponse(
            raw_registers=list(raw),
            control_word=control_word,
            command_type=command_type,
            control=control,
            byte_swap=byte_swap,
            status=status,
            index=index,
            subindex=subindex,
            data_length=data_length,
            data=data,
        )

    def _coerce_data_to_bytes(
        self,
        data: DataLike,
        data_length: Optional[int] = None,
    ) -> bytes:
        if isinstance(data, int):
            return int_to_bytes(data, length=data_length)

        if isinstance(data, (bytes, bytearray)):
            payload = bytes(data)

        elif isinstance(data, str):
            # Strings are treated as hex payloads, not decimal numbers.
            # Example: "015E", "01 5E", and "0x015E" all become b"\x01\x5E".
            payload = hex_string_to_bytes(data)

        elif isinstance(data, Sequence):
            payload = bytes(_validate_byte_sequence(data))

        else:
            raise TypeError(
                "data must be int, bytes, bytearray, hex string, or sequence of byte values"
            )

        if data_length is not None:
            _validate_range("data_length", data_length, 1, 232)

            if len(payload) != data_length:
                raise ValueError(
                    f"data_length={data_length} but payload length is {len(payload)}"
                )

        if not payload:
            raise ValueError("ISDU write payload cannot be empty")

        if len(payload) > 232:
            raise ValueError("ISDU payload cannot exceed 232 bytes")

        return payload

    def _read_registers(self, address: int, count: int) -> Optional[list[int]]:
        return self.modbus_client.read_register(
            register_address=address,
            number_of_registers=count,
        )

    def _write_registers(self, address: int, values: Sequence[int]) -> None:
        values_list = list(values)

        for value in values_list:
            _validate_range("register value", value, 0, 0xFFFF)

        result = self.modbus_client.write_multiple_register(
            address,
            values_list,
        )

        if result is not True:
            raise ISDUCommunicationError(
                f"Failed to write ISDU request registers at {address}"
            )


def _clean_hex_string(value: str) -> str:
    return (
        value.strip()
        .replace("0x", "")
        .replace("0X", "")
        .replace(" ", "")
        .replace("_", "")
        .replace("-", "")
        .replace(":", "")
    )


def _validate_range(name: str, value: int, minimum: int, maximum: int) -> None:
    if not isinstance(value, int):
        raise TypeError(f"{name} must be int")

    if value < minimum or value > maximum:
        raise ValueError(
            f"{name} must be between {minimum} and {maximum}; got {value}"
        )


def _validate_index_subindex_length(index: int, subindex: int, length: int) -> None:
    _validate_range("index", index, 0, 0xFFFF)
    _validate_range("subindex", subindex, 0, 0xFF)
    _validate_range("length", length, 1, 232)


def _validate_byte_sequence(values: Iterable[int]) -> list[int]:
    output = []

    for value in values:
        _validate_range("byte value", value, 0, 0xFF)
        output.append(value)

    return output


if __name__ == "__main__":
    # Example usage.
    # Make sure Modbus/TCP is enabled on the ICE3 first.

    client = ISDU(
        host="192.168.137.5",
        iol_port=1,
        tcp_port=502,
        unit_id=1,
    )

    # ---------------------------------------------------------
    # Hex/decimal helper examples
    # ---------------------------------------------------------
    print("0x3C decimal:", hex_to_decimal("0x3C"))
    print("01 5E decimal:", hex_to_decimal("01 5E"))
    print("350 hex:", decimal_to_hex(350, width=4))

    # ---------------------------------------------------------
    # Example: read index 0x3C / subindex 0x01 as 2-byte value
    # Your previous response showed 0x015E = 350.
    # ---------------------------------------------------------
    response = client.read_isdu(index="60", subindex="1", length=2)

    print("Raw registers:", response.raw_registers[:10], "...")
    print("Status:", response.status)
    print("Control word:", response.control_word_hex)
    print("Index:", response.index, response.index_hex)
    print("Subindex:", response.subindex, response.subindex_hex)
    print("Data hex:", response.data_hex)
    print("Data decimal:", response.data_decimal)

    # ---------------------------------------------------------
    # Example write: write decimal 350 as UINT16 to index 0x3C/subindex 0x01.
    # Only uncomment when you know this is safe for your connected IO-Link device.
    # ---------------------------------------------------------
    client.write_isdu(
        index="60",
        subindex="2",
        data=800,
        data_length=2,
        wait_for_response=False,
    )
    print("Write request sent for index 60 / subindex 2.")

    # Verify with a separate read. The ICE response mailbox can retain the
    # previous write/read response briefly, so retry this read a few times.
    verify_response = client.read_isdu(
        index="60",
        subindex="2",
        length=2,
        retry_count=3,
        retry_delay_s=0.5,
    )
    print("Readback hex:", verify_response.data_hex)
    print("Readback decimal:", verify_response.data_decimal)

    client.write_isdu(
        index="60",
        subindex="1",
        data=900,
        data_length=2,
        wait_for_response=False,
    )
    print("Write request sent for index 60 / subindex 2.")

    # Verify with a separate read. The ICE response mailbox can retain the
    # previous write/read response briefly, so retry this read a few times.
    verify_response = client.read_isdu(
        index="60",
        subindex="1",
        length=2,
        retry_count=3,
        retry_delay_s=0.5,
    )
    print("Readback hex:", verify_response.data_hex)
    print("Readback decimal:", verify_response.data_decimal)

    # ---------------------------------------------------------
    # Same write using hex payload instead of decimal int:
    # ---------------------------------------------------------
    # write_response = client.write_isdu(
    #     index="0x3C",
    #     subindex="0x01",
    #     data="01 5E",
    # )
    # print("Write confirmed:", write_response.ok)
