from __future__ import annotations

from typing import Iterable, Optional

from pyModbusTCP.client import ModbusClient


class ModbusClientWrapper:
    """
    Small wrapper around pyModbusTCP used by the ICE ISDU helper.

    Important behavior:
    - pyModbusTCP uses base-0 addressing by default in this project.
    - read_register returns a list[int] or None.
    - write_register and write_multiple_register return True/False so higher layers
      can detect communication failures instead of silently continuing.
    """

    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        auto_open: bool = True,
        auto_close: bool = True,
        timeout: Optional[float] = None,
    ):
        self.client = ModbusClient(
            host=host,
            port=int(port),
            unit_id=int(unit_id),
            auto_open=auto_open,
            auto_close=auto_close,
        )

        if timeout is not None:
            self.client.timeout = float(timeout)

    def read_register(self, register_address: int, number_of_registers: int = 1):
        """
        Read holding registers.

        Returns:
            list[int] on success, None on failure.
        """
        try:
            value = self.client.read_holding_registers(
                int(register_address),
                int(number_of_registers),
            )

            if value is None:
                print(f"Failed to read from register: {register_address}")
                return None

            return value

        except Exception as e:
            print(f"Error reading register {register_address}: {e}")
            return None

    def write_register(self, register_address: int, value: int) -> bool:
        """
        Write one holding register.

        Returns:
            True on Modbus success, False on failure.
        """
        try:
            success = self.client.write_single_register(
                int(register_address),
                int(value),
            )

            if not success:
                print(f"Failed to write to register: {register_address}")

            return bool(success)

        except Exception as e:
            print(f"Error writing register {register_address}: {e}")
            return False

    def write_multiple_register(self, register_address: int, *values) -> bool:
        """
        Write multiple holding registers.

        Compatible with both call styles:
            write_multiple_register(addr, [1, 2, 3])
            write_multiple_register(addr, 1, 2, 3)

        Returns:
            True on Modbus success, False on failure.
        """
        try:
            values_list = self._normalize_values(values)

            success = self.client.write_multiple_registers(
                int(register_address),
                values_list,
            )

            if not success:
                print(f"Failed to write to register: {register_address}")

            return bool(success)

        except Exception as e:
            print(f"Error writing register {register_address}: {e}")
            return False

    @staticmethod
    def _normalize_values(values) -> list[int]:
        if len(values) == 1 and isinstance(values[0], Iterable) and not isinstance(values[0], (bytes, bytearray, str)):
            values = list(values[0])
        else:
            values = list(values)

        output = []
        for value in values:
            value_int = int(value)
            if value_int < 0 or value_int > 0xFFFF:
                raise ValueError(f"Modbus register value out of range 0..65535: {value_int}")
            output.append(value_int)

        if not output:
            raise ValueError("At least one register value is required")

        return output
