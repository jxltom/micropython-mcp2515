try:
    from typing import Any, Optional
except ImportError:
    pass

try:
    from pyb import Pin, SPI as MICROPYTHON_SPI
except ImportError:
    from machine import Pin, SPI as MICROPYTHON_SPI

from . import (
    SPI_DEFAULT_BAUDRATE,
    SPI_DEFAULT_FIRSTBIT,
    SPI_DEFAULT_PHASE,
    SPI_DEFAULT_POLARITY,
    SPI_ESP32_HARDWARE_CHANNEL,
    SPI_ESP32_MISO_PIN,
    SPI_ESP32_MOSI_PIN,
    SPI_ESP32_SCK_PIN,
)
from .spi import SPI


class SPIESP32(SPI):
    def init(self, baudrate: int) -> Any:
        return MICROPYTHON_SPI(
            SPI_ESP32_HARDWARE_CHANNEL,
            sck=Pin(SPI_ESP32_SCK_PIN),
            mosi=Pin(SPI_ESP32_MOSI_PIN),
            miso=Pin(SPI_ESP32_MISO_PIN),
            baudrate=baudrate,
            firstbit=SPI_DEFAULT_FIRSTBIT,
            polarity=SPI_DEFAULT_POLARITY,
            phase=SPI_DEFAULT_POLARITY,
        )
