import sys
import time

from . import SPI_DEFAULT_BAUDRATE, SPI_DUMMY_INT, SPI_TRANSFER_LEN, SPI_HOLD_US

try:
    from pyb import Pin
except ImportError:
    from machine import Pin


class SPI:
    def __init__(self, cs, baudrate=SPI_DEFAULT_BAUDRATE):
        self._SPICS = Pin(cs, Pin.OUT)
        self._SPI = self.init(baudrate=baudrate)
        self.end()

    def init(self, baudrate):
        raise NotImplementedError

    def start(self):
        self._SPICS.value(0)
        time.sleep_us(SPI_HOLD_US)

    def end(self):
        self._SPICS.value(1)
        time.sleep_us(SPI_HOLD_US)

    def transfer(self, value=SPI_DUMMY_INT, read=False):
        """Write int value to SPI and read SPI as int value simultaneously.
        This method supports transfer single byte only,
        and the system byte order doesn't matter because of that. The input and
        output int value are unsigned.
        """
        value = value.to_bytes(SPI_TRANSFER_LEN, sys.byteorder)

        if read:
            output = bytearray(SPI_TRANSFER_LEN)
            self._SPI.write_readinto(value, output)
            return int.from_bytes(output, sys.byteorder)
        self._SPI.write(value)
        return None
