class Machine:
    class Pin:
        OUT = None

        def __init__(self, cs, direction=None):
            pass

        def value(self, value):
            pass

    class SPI:
        MSB = None

        def __init__(
            self, channel, sck, mosi, miso, baudrate, firstbit, polarity, phase,
        ):
            pass

        def write_readinto(self, input_, output):
            pass


class Time:
    @staticmethod
    def sleep_ms(ms):
        pass

    @staticmethod
    def ticks_ms():
        pass

    @staticmethod
    def ticks_add(tick, delta):
        pass

    @staticmethod
    def ticks_diff(tick1, tick2):
        return 0


import sys

sys.modules["machine"] = Machine
sys.modules["time"] = Time
