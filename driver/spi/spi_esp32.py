from machine import PIN, SPI

from . import (
    SPI_BAUDRATE,
    SPI_ESP32_HARDWARE_CHANNEL,
    SPI_ESP32_MISO_PIN,
    SPI_ESP32_MOSI_PIN,
    SPI_ESP32_SCK_PIN,
)


class SPI(_SPI):
    def __init__(self):
        self.spi = SPI(
            SPI_ESP32_HARDWARE_CHANNEL,
            baudrate=SPI_BAUDRATE,
            sck=Pin(SPI_ESP32_SCK_PIN),
            mosi=Pin(SPI_ESP32_MOSI_PIN),
            miso=Pin(SPI_ESP32_MOSI_PIN),
            firstbit=SPI.MSB,
            polarity=0,
            phase=0,
        )
