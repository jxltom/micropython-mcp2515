from tests import fake_micropython  # for testing only

"""
Following SPI drivers are supported, please adjust by your hardware
from .driver import SPIESP8286 as SPI
from .driver import SPIESP32 as SPI
"""
import time

from driver import CAN, CAN_SPEED, CAN_CLOCK, CANFrame, ERROR
from driver import SPIESP32 as SPI


def main():
    """
    The CAN driver can be initialized with default baudrate 10MHz
    CAN(SPI(cs=YOUR_SPI_CS_PIN)) or
    CAN(SPI(cs=YOUR_SPI_CS_PIN, baudrate=YOUR_DESIRED_BAUDRATE))
    """
    can = CAN(SPI(cs=23))

    if can.reset() != ERROR.ERROR_OK:
        print("Can not reset for MCP2515")
        return
    if can.setBitrate(CAN_SPEED.CAN_500KBPS, CAN_CLOCK.MCP_8MHZ) != ERROR.ERROR_OK:
        print("Can not set bitrate for MCP2515")
        return
    if can.setNormalMode() != ERROR.ERROR_OK:
        print("Can not set normal mode for MCP2515")
        return

    end_time = time.ticks_add(time.ticks_ms(), 1000)
    while True:
        # Read message
        error, iframe = can.readMessage()
        if error == ERROR.ERROR_OK:
            print("RX  {}".format(iframe))

        # Send message every second
        if time.ticks_diff(time.ticks_ms(), end_time) >= 0:
            end_time = time.ticks_add(time.ticks_ms(), 1000)

            oframe = CANFrame(can_id=0x7FF, data=b"\x12\x34\x56\x78\x9A\xBC\xDE\xF0")  # standard frame
            oframe = CANFrame(can_id=0x80000800, data=b"\x12\x34\x56\x78\x9A\xBC\xDE\xF0")  # extended frame
            error = can.sendMessage(oframe)
            if error == ERROR.ERROR_OK:
                print("TX  {}".format(oframe))
            else:
                print("TX failed with error code {}".format(error))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
