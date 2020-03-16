import sys

try:
    from pyb import Pin
except ImportError:
    from machine import Pin

from . import SPI_DEFAULT_BAUDRATE, SPI_DUMMY_BYTE


class SPI:
    def __init__(self, cs, baudrate=SPI_DEFAULT_BAUDRATE):
        self._SPICS = Pin(cs, Pin.OUT)
        self._SPI = self.init(baudrate=baudrate)
        self.end()

    def init(self, baudrate):
        raise NotImplementedError

    def start(self):
        self._SPICS.value(0)

    def end(self):
        self._SPICS.value(1)

    def transfer(self, byteAsInt=SPI_DUMMY_BYTE):
        byteAsInt = byteAsInt.to_bytes(1, sys.byteorder)

        byteBuffer = bytearray(len(byteAsInt))
        self._SPI.write_readinto(byteAsInt, byteBuffer)
    

        return int.from_bytes(byteBuffer, sys.byteorder)
