# special address description flags for the CAN_ID
CAN_EFF_FLAG = 0x80000000  # EFF/SFF is set in the MSB
CAN_RTR_FLAG = 0x40000000  # remote transmission request
CAN_ERR_FLAG = 0x20000000  # error message frame

# valid bits in CAN ID for frame formats
CAN_SFF_MASK = 0x000007FF  # standard frame format (SFF)
CAN_EFF_MASK = 0x1FFFFFFF  # extended frame format (EFF)
CAN_ERR_MASK = 0x1FFFFFFF  # omit EFF, RTR, ERR flags

#
# Controller Area Network Identifier structure
#
# bit 0-28 : CAN identifier (11/29 bit)
# bit 29   : error message frame flag (0 = data frame, 1 = error message)
# bit 30   : remote transmission request flag (1 = rtr frame)
# bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
#
CAN_SFF_ID_BITS = 11
CAN_EFF_ID_BITS = 29

# CAN payload length and DLC definitions according to ISO 11898-1
CAN_MAX_DLC = 8
CAN_MAX_DLEN = 8

# CAN ID length
CAN_IDLEN = 4


class CANFrame:
    def __init__(self, can_id, can_dlc):
        self.can_id = can_id  # 32 bit CAN_ID + EFF/RTR/ERR flags
        self.can_dlc = can_dlc  # frame payload length in byte (0 .. CAN_MAX_DLEN)
        self._data = None

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, data):
        if data is not None and not len(data) <= CAN_MAX_DLEN:
            raise Exception("The CAN frame data length exceeds the maximum")

        self._data = data
