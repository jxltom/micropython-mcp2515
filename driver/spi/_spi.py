from . import SPI_DEFAULT_READINTO_BUFFER_SIZE


class _SPI:
    def __init__(self):
        raise NotImplementedError

    def transfer(self, byte):
        buffer = bytearray(SPI_DEFAULT_READINTO_BUFFER_SIZE)
        self.spi.write_readinto(byte, buffer)
        return buffer
