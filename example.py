from tests import fake_micropython  # for testing only

"""
Following SPI drivers are supported, please adjust by your hardware
from .driver import SPIESP8286 as SPI
from .driver import SPIESP32 as SPI


The CAN driver can be initialized with default baudrate 10MHz
CAN(SPI(cs=YOUR_SPI_CS_PIN)) or
CAN(SPI(cs=YOUR_SPI_CS_PIN, baudrate=YOUR_DESIRED_BAUDRATE))

And here is an example to set filter for extended frame with ID 0x12345678
can.setFilter(RXF.RXF0, True, 0x12345678 | CAN_EFF_FLAG)
can.setFilterMask(MASK.MASK0, True, 0x1FFFFFFF | CAN_EFF_FLAG)
can.setFilterMask(MASK.MASK1, True, 0x1FFFFFFF | CAN_EFF_FLAG)
"""
import time

from driver import (
    CAN,
    CAN_CLOCK,
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
    CAN_SPEED,
    ERROR,
)
from driver import SPIESP32 as SPI
from driver import CANFrame


def main():
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

    end_time, n = time.ticks_add(time.ticks_ms(), 1000), -1
    while True:
        # Read message
        error, iframe = can.readMessage()
        if error == ERROR.ERROR_OK:
            print("RX  {}".format(iframe))

        # Send message every second
        if time.ticks_diff(time.ticks_ms(), end_time) >= 0:
            end_time = time.ticks_add(time.ticks_ms(), 1000)
            n += 1
            n %= 4

            data = b"\x12\x34\x56\x78\x9A\xBC\xDE\xF0"
            standard_frame = CANFrame(canid=0x7FF, data=data)
            extended_frame = CANFrame(canid=0x12345678 | CAN_EFF_FLAG, data=data)
            remote_frame = CANFrame(canid=0x7FF | CAN_RTR_FLAG)
            error_frame = CANFrame(canid=0x7FF | CAN_ERR_FLAG, data=data)
            frames = [standard_frame, extended_frame, remote_frame, error_frame]

            error = can.sendMessage(frames[n])
            if error == ERROR.ERROR_OK:
                print("TX  {}".format(frames[n]))
            else:
                print("TX failed with error code {}".format(error))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
