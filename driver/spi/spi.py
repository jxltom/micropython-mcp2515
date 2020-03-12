from machine import Pin

from . import SPI_DEFAULT_READINTO_BUFFER_SIZE, SPI_DEFAULT_BAUDRATE


class SPI:
    def __init__(self, cs, baudrate=SPI_DEFAULT_BAUDRATE):
        self._SPICS = Pin(cs, Pin.OUT)
        self._SPI = self.init(baudrate=baudrate)
        self.end()

    def init(self, baudrate):
        raise NotImplementedError

    def start(self):
        self._SPICS.low()

    def end(self):
        self._SPICS.high()

    def transfer(self, byte):
        buffer = bytearray(SPI_DEFAULT_READINTO_BUFFER_SIZE)
        self._SPI.write_readinto(byte, buffer)
        return buffer
