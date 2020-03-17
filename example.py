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
            print("RX ID:{} Data: {}".format(iframe.can_id, iframe.data))

        # Send message every second
        if time.ticks_diff(time.ticks_ms(), end_time) >= 0:
            end_time = time.ticks_add(time.ticks_ms(), 1000)

            oframe = CANFrame(can_id=0x666, can_dlc=4)
            oframe.data = b"\xff\xff\xff\xff"
            error = can.sendMessage(oframe)
            if error == ERROR.ERROR_OK:
                print("TX ID:{} Data: {}".format(oframe.can_id, oframe.data))
            else:
                print("TX failed with error code {}".format(error))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
