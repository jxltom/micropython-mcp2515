# micropython-mcp2515

Micropython MCP2515 driver, porting from Arduino MCP2515 CAN interface library

## Getting Started

This MCP2515 driver could be used for any MCU or platform supported by Micropython, such as Pyboard, ESP32, ESP8286, etc.

```python
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
    # Initialization
    can = CAN(SPI(cs=23))

    # Configuration
    if can.reset() != ERROR.ERROR_OK:
        print("Can not reset for MCP2515")
        return
    if can.setBitrate(CAN_SPEED.CAN_500KBPS, CAN_CLOCK.MCP_8MHZ) != ERROR.ERROR_OK:
        print("Can not set bitrate for MCP2515")
        return
    if can.setNormalMode() != ERROR.ERROR_OK:
        print("Can not set normal mode for MCP2515")
        return

    # Prepare frames
    data = b"\x12\x34\x56\x78\x9A\xBC\xDE\xF0"
    sff_frame = CANFrame(canid=0x7FF, data=data)
    sff_none_data_frame = CANFrame(canid=0x7FF)
    err_frame = CANFrame(canid=0x7FF | CAN_ERR_FLAG, data=data)
    eff_frame = CANFrame(canid=0x12345678 | CAN_EFF_FLAG, data=data)
    eff_none_data_frame = CANFrame(canid=0x12345678 | CAN_EFF_FLAG)
    rtr_frame = CANFrame(canid=0x7FF | CAN_RTR_FLAG)
    rtr_with_eid_frame = CANFrame(canid=0x12345678 | CAN_RTR_FLAG | CAN_EFF_FLAG)
    rtr_with_data_frame = CANFrame(canid=0x7FF | CAN_RTR_FLAG, data=data)
    frames = [
        sff_frame,
        sff_none_data_frame,
        err_frame,
        eff_frame,
        eff_none_data_frame,
        rtr_frame,
        rtr_with_eid_frame,
        rtr_with_data_frame,
    ]

    # Read all the time and send message in each second
    end_time, n = time.ticks_add(time.ticks_ms(), 1000), -1
    while True:
        error, iframe = can.readMessage()
        if error == ERROR.ERROR_OK:
            print("RX  {}".format(iframe))

        if time.ticks_diff(time.ticks_ms(), end_time) >= 0:
            end_time = time.ticks_add(time.ticks_ms(), 1000)
            n += 1
            n %= len(frames)

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
```

## Limitation

- Since Micropython is not fast enough, MCP2515 might send duplicated messages when CAN speed is too fast or SPI speed is too low. This is because previous message will still be transmitted repeatedly when MCP2515's SPI CS is not disabled timely.
