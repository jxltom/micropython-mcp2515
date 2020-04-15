try:
    from typing import Any, Optional
except ImportError:
    pass

import sys
import time

try:
    from pyb import Pin
except ImportError:
    from machine import Pin

from . import SPI_DEFAULT_BAUDRATE, SPI_DUMMY_INT, SPI_TRANSFER_LEN, SPI_HOLD_US


class SPI:
    def __init__(self, cs: int, baudrate: int = SPI_DEFAULT_BAUDRATE) -> None:
        self._SPICS = Pin(cs, Pin.OUT)
        self._SPI = self.init(baudrate=baudrate)  # type: Any
        self.end()

    def init(self, baudrate: int) -> Any:
        raise NotImplementedError

    def start(self) -> None:
        self._SPICS.value(0)
        time.sleep_us(SPI_HOLD_US)  # type: ignore

    def end(self) -> None:
        self._SPICS.value(1)
        time.sleep_us(SPI_HOLD_US)  # type: ignore

    def transfer(self, value: int = SPI_DUMMY_INT, read: bool = False) -> Optional[int]:
        """Write int value to SPI and read SPI as int value simultaneously.
        This method supports transfer single byte only,
        and the system byte order doesn't matter because of that. The input and
        output int value are unsigned.
        """
        value_as_byte = value.to_bytes(SPI_TRANSFER_LEN, sys.byteorder)

        if read:
            output = bytearray(SPI_TRANSFER_LEN)
            self._SPI.write_readinto(value_as_byte, output)
            return int.from_bytes(output, sys.byteorder)
        self._SPI.write(value_as_byte)
        return None
