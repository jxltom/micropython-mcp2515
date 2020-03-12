from machine import Pin

from . import SPI_DEFAULT_READINTO_BUFFER_SIZE, SPI_DEFAULT_BAUDRATE, SPI_DUMMY_BYTE


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

    def transfer(self, byte=SPI_DUMMY_BYTE):
        if len(byte) != 1:
            raise Exception("Only one byte is supported in transfer function")

        buffer = bytearray(len(byte))
        self._SPI.write_readinto(byte, buffer)
        return buffer
