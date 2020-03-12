import time
from collections import namedtuple

from machine import PIN

from . import (
    CAN_CFGS,
    CAN_CLKOUT,
    CAN_CLOCK,
    CANCTRL_ABAT,
    CANCTRL_CLKEN,
    CANCTRL_CLKPRE,
    CANCTRL_OSM,
    CANCTRL_REQOP,
    CANCTRL_REQOP_MODE,
    CANINTF,
    CANSTAT_ICOD,
    CANSTAT_OPMOD,
    CNF3_SOF,
    DLC_MASK,
    EFLG,
    EFLG_ERRORMASK,
    ERROR,
    INSTRUCTION,
    MASK,
    MCP_DATA,
    MCP_DLC,
    MCP_EID0,
    MCP_EID8,
    MCP_SIDH,
    MCP_SIDL,
    N_RXBUFFERS,
    N_TXBUFFERS,
    REGISTER,
    RTR_MASK,
    RXB0CTRL_BUKT,
    RXF,
    STAT,
    STAT_RXIF_MASK,
    TXB_EXIDE_MASK,
    RXBn,
    RXBnCTRL_RTR,
    RXBnCTRL_RXM_EXT,
    RXBnCTRL_RXM_MASK,
    RXBnCTRL_RXM_STD,
    RXBnCTRL_RXM_STDEXT,
    TXBn,
    TXBnCTRL,
)
from .can import (
    CAN_EFF_FLAG,
    CAN_EFF_MASK,
    CAN_ERR_FLAG,
    CAN_ERR_MASK,
    CAN_MAX_DLEN,
    CAN_RTR_FLAG,
    CAN_SFF_MASK,
    CAN_IDLEN,
    CANFrame,
)
from .spi.spi_esp32 import SPI

TXBnREGS = namedtuple(TXBnREGS, "CTRL SIDH DATA")
RXBnREGS = namedtuple(RXBnREGS, "CTRL SIDH DATA CANINTFRXnIF")


TXB = [
    [REGISTER.MCP_TXB0CTRL, REGISTER.MCP_TXB0SIDH, REGISTER.MCP_TXB0DATA],
    [REGISTER.MCP_TXB1CTRL, REGISTER.MCP_TXB1SIDH, REGISTER.MCP_TXB1DATA],
    [REGISTER.MCP_TXB2CTRL, REGISTER.MCP_TXB2SIDH, REGISTER.MCP_TXB2DATA],
]

RXB = [
    [
        REGISTER.MCP_RXB0CTRL,
        REGISTER.MCP_RXB0SIDH,
        REGISTER.MCP_RXB0DATA,
        REGISTER.CANINTF_RX0IF,
    ],
    [
        REGISTER.MCP_RXB1CTRL,
        REGISTER.MCP_RXB1SIDH,
        REGISTER.MCP_RXB1DATA,
        REGISTER.CANINTF_RX1IF,
    ],
]


class CAN:
    def __init__(self, _CS):
        self.SPI = SPI()

        self.SPICS = Pin(_CS, Pin.OUT)
        self.endSPI()

    def startSPI(self):
        self.SPICS.low()

    def endSPI(self):
        self.SPICS.high()

    def reset(self):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_RESET)
        self.endSPI()

        time.sleep_ms(10)

        zeros = bytearray(14)
        self.setRegisters(REGISTER.MCP_TXB0CTRL, zeros)
        self.setRegisters(REGISTER.MCP_TXB1CTRL, zeros)
        self.setRegisters(REGISTER.MCP_TXB2CTRL, zeros)

        self.setRegister(REGISTER.MCP_RXB0CTRL, 0)
        self.setRegister(REGISTER.MCP_RXB1CTRL, 0)

        self.setRegister(
            REGISTER.MCP_CANINTE,
            CANINTF.CANINTF_RX0IF
            | CANINTF.CANINTF_RX1IF
            | CANINTF.CANINTF_ERRIF
            | CANINTF.CANINTF_MERRF,
        )

        self.modifyRegister(
            REGISTER.MCP_RXB0CTRL,
            RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT,
            RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT,
        )
        self.modifyRegister(
            REGISTER.MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT
        )

        # clear filters and masks
        """
        filters = [RXF.RXF0, RXF.RXF1, RXF.RXF2, RXF.RXF3, RXF.RXF4, RXF.RXF5]
        for i in range(len(filters)):
            result = self.setFilter(filters[i], True, 0);
            if result != ERROR.ERROR_OK:
                return result
        masks = [MASK.MASK0, MASK.MASK1]
        for i in range(len(masks)):
            result = self.setFilterMask(masks[i], True, 0);
            if result != ERROR.ERROR_OK:
                return result
        """

        return ERROR.ERROR_OK

    def readRegister(self, reg):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_READ)
        self.SPI.transfer(reg)
        ret = self.SPI.transfer(0x00)
        self.endSPI()

        return ret

    def readRegisters(self, reg, n):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_READ)
        self.SPI.transfer(reg)
        # mcp2515 has auto-increment of address-pointer
        values = []
        for i in range(n):
            values.append(SPI.transfer(0x00))
        self.endSPI()

        return values

    def setRegister(self, reg, value):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_WRITE)
        self.SPI.transfer(reg)
        self.SPI.transfer(value)
        endSPI()

    def setRegisters(self, reg, values):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_WRITE)
        self.SPI.transfer(reg)
        for i in range(len(values)):
            SPI.transfer(values[i])
        self.endSPI()

    def modifyRegister(self, reg, mask, data):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_BITMOD)
        self.SPI.transfer(reg)
        self.SPI.transfer(mask)
        self.SPI.transfer(data)
        self.endSPI()

    def getStatus(self):
        self.startSPI()
        self.SPI.transfer(INSTRUCTION.INSTRUCTION_READ_STATUS)
        i = SPI.transfer(0x00)
        self.endSPI()

        return i

    def setConfigMode(self):
        return self.setMode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_CONFIG)

    def setListenOnlyMode(self):
        return self.setMode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_LISTENONLY)

    def setSleepMode(self):
        return self.setMode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_SLEEP)

    def setLoopbackMode(self):
        return self.setMode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_LOOPBACK)

    def setNormalMode(self):
        return self.setMode(CANCTRL_REQOP_MODE.CANCTRL_REQOP_NORMAL)

    def setMode(self, mode):
        self.modifyRegister(REGISTER.MCP_CANCTRL, REGISTER.CANCTRL_REQOP, mode)

        endTime = time.ticks_add(time.ticks_ms(), 10)
        modeMatch = False
        while time.ticks_diff(time.ticks_ms(), endTime) < 0:
            newmode = self.readRegister(REGISTER.MCP_CANSTAT)
            newmode &= CANSTAT_OPMOD

            modeMatch = newmode == mode
            if modeMatch:
                break

        return ERROR.ERROR_OK if modeMatch else ERROR.ERROR_FAIL

    def setBitrate(self, canSpeed):
        return self.setBitrate(canSpeed, CAN_CLOCK.MCP_16MHZ)

    def setBitrate(self, canSpeed, canClock):
        error = self.setConfigMode()
        if error != ERROR.ERROR_OK:
            return error

        set_ = 1
        try:
            cfg1, cfg2, cfg3 = CAN_CFGS[canClock][canSpeed]
        except KeyError:
            set_ = 0

        if set_:
            self.setRegister(REGISTER.MCP_CNF1, cfg1)
            self.setRegister(REGISTER.MCP_CNF2, cfg2)
            self.setRegister(REGISTER.MCP_CNF3, cfg3)
            return ERROR.ERROR_OK
        return ERROR.ERROR_FAIL

    def setClkOut(self, divisor):
        if divisor == CAN_CLKOUT.CLKOUT_DISABLE:
            # Turn off CLKEN
            self.modifyRegister(REGISTER.MCP_CANCTRL, CANCTRL_CLKEN, 0x00)

            # Turn on CLKOUT for SOF
            self.modifyRegister(REGISTER.MCP_CNF3, REGISTER.CNF3_SOF, REGISTER.CNF3_SOF)
            return ERROR.ERROR_OK

        # Set the prescaler (CLKPRE)
        self.modifyRegister(REGISTER.MCP_CANCTRL, REGISTER.CANCTRL_CLKPRE, divisor)

        # Turn on CLKEN
        self.modifyRegister(REGISTER.MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN)

        # Turn off CLKOUT for SOF
        self.modifyRegister(REGISTER.MCP_CNF3, CNF3_SOF, 0x00)
        return ERROR.ERROR_OK

    def prepareId(self, ext, id):
        canid = id & 0x0FFFF
        buffer = bytearray(CAN_IDLEN)

        if ext:
            buffer[MCP_EID0] = canid & 0xFF
            buffer[MCP_EID8] = canid >> 8
            canid = id >> 16
            buffer[MCP_SIDL] = canid & 0x03
            buffer[MCP_SIDL] += (canid & 0x1C) << 3
            buffer[MCP_SIDL] |= TXB_EXIDE_MASK
            buffer[MCP_SIDH] = canid >> 5
        else:
            buffer[MCP_SIDH] = canid >> 3
            buffer[MCP_SIDL] = (canid & 0x07) << 5
            buffer[MCP_EID0] = 0
            buffer[MCP_EID8] = 0

        return buffer

    def setFilterMask(self, mask, ext, ulData):
        res = self.setConfigMode()
        if res != ERROR.ERROR_OK:
            return res

        tbufdata = self.prepareId(ext, ulData)

        reg = None
        if mask == MASK.MASK0:
            reg = REGISTER.MCP_RXM0SIDH
        elif mask == MASK.MASK1:
            reg = REGISTER.MCP_RXM1SIDH
        else:
            return ERROR.ERROR_FAIL

        self.setRegisters(reg, tbufdata)

        return ERROR.ERROR_OK

    def setFilter(self, num, ext, ulData):
        res = self.setConfigMode()
        if res != ERROR.ERROR_OK:
            return res

        reg = None
        if num == RXF.RXF0:
            reg = REGISTER.MCP_RXF0SIDH
        elif num == RXF.RXF1:
            reg = REGISTER.MCP_RXF1SIDH
        elif num == RXF.RXF2:
            reg = REGISTER.MCP_RXF2SIDH
        elif num == RXF.RXF3:
            reg = REGISTER.MCP_RXF3SIDH
        elif num == RXF.RXF4:
            reg = REGISTER.MCP_RXF4SIDH
        elif num == RXF.RXF5:
            reg = REGISTER.MCP_RXF5SIDH
        else:
            return ERROR.ERROR_FAIL

        tbufdata = self.prepareId(ext, ulData)
        self.setRegisters(reg, tbufdata)

        return ERROR.ERROR_OK

    def sendMessage(self, txbn, frame):
        if frame.can_dlc > CAN_MAX_DLEN:
            return ERROR.ERROR_FAILTX

        txbuf = TXB[txbn]

        ext = frame.can_id & CAN_EFF_FLAG
        rtr = frame.can_id & CAN_RTR_FLAG
        id_ = frame.can_id & (CAN_EFF_MASK if ext else CAN_SFF_MASK)

        data = prepareId(ext, id_)
        data.extend(bytearray(1 + CAN_MAX_DLEN))

        data[MCP_DLC] = (frame.can_dlc | RTR_MASK) if rtr else frame.can_dlc

        data[MCP_DATA : MCP_DATA + frame.can_dlc] = frame.data

        self.setRegisters(txbuf.SIDH, data)

        modifyRegister(txbuf.CTRL, TXBnCTRL.TXB_TXREQ, TXBnCTRL.TXB_TXREQ)

        return ERROR.ERROR_OK

    def sendMessage(self, frame):
        if frame.can_dlc > CAN_MAX_DLEN:
            return ERROR.ERROR_FAILTX

        txBuffers = [TXBn.TXB0, TXBn.TXB1, TXBn.TXB2]

        for i in range(N_TXBUFFERS):
            txbuf = TXB[txBuffers[i]]
            ctrlval = self.readRegister(txbuf.CTRL)
            if (ctrlval & TXBnCTRL.TXB_TXREQ) == 0:
                return self.sendMessage(txBuffers[i], frame)

        return ERROR.ERROR_FAILTX

    def readMessage(self, rxbn, frame):
        rxb = RXB[rxbn]

        tbufdata = self.readRegisters(rxb.SIDH, 1 + CAN_IDLEN)

        id_ = tbufdata[MCP_SIDH] << 3 + tbufdata[MCP_SIDL] >> 5

        if (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK:
            id_ = (id_ << 2) + (tbufdata[MCP_SIDL] & 0x03)
            id_ = (id_ << 8) + tbufdata[MCP_EID8]
            id_ = (id_ << 8) + tbufdata[MCP_EID0]
            id_ |= CAN_EFF_FLAG

        dlc = tbufdata[MCP_DLC] & DLC_MASK
        if dlc > CAN_MAX_DLEN:
            return ERROR.ERROR_FAIL, None

        ctrl = self.readRegister(rxb.CTRL)
        if ctrl & RXBnCTRL_RTR:
            id_ |= CAN_RTR_FLAG

        frame = CANFrame(can_id=id_, can_dlc=dlc)

        frame.data = readRegisters(rxb.DATA, dlc)

        modifyRegister(REGISTER.MCP_CANINTF, rxb.CANINTF_RXnIF, 0)

        return ERROR.ERROR_OK, frame

    def readMessage(self, frame):
        rc = ERROR.ERROR_NOMSG, None

        stat = self.getStatus()
        if stat & STAT.STAT_RX0IF:
            rc = self.readMessage(RXBn.RXB0, frame)
        elif stat & STAT.STAT_RX1IF:
            rc = self.readMessage(RXBn.RXB1, frame)

        return rc

    def checkReceive(self):
        res = self.getStatus()
        if res & STAT_RXIF_MASK:
            return True
        return False

    def checkError(self):
        eflg = self.getErrorFlags()

        if eflg & EFLG_ERRORMASK:
            return True
        return False

    def getErrorFlags(self):
        return self.readRegister(REGISTER.MCP_EFLG)

    def clearRXnOVRFlags(self):
        self.modifyRegister(REGISTER.MCP_EFLG, EFLG.EFLG_RX0OVR | EFLG.EFLG_RX1OVR, 0)

    def getInterrupts(self):
        return self.readRegister(REGISTER.MCP_CANINTF)

    def clearInterrupts(self):
        self.setRegister(REGISTER.MCP_CANINTF, 0)

    def getInterruptMask(self):
        return self.readRegister(REGISTER.MCP_CANINTE)

    def clearTXInterrupts(self):
        self.modifyRegister(
            REGISTER.MCP_CANINTF,
            CANINTF.CANINTF_TX0IF | CANINTF.CANINTF_TX1IF | CANINTF.CANINTF_TX2IF,
            0,
        )

    def clearRXnOVR(self):
        eflg = self.getErrorFlags()
        if eflg != 0:
            self.clearRXnOVRFlags()
            self.clearInterrupts()
            # modifyRegister(REGISTER.MCP_CANINTF, CANINTF.CANINTF_ERRIF, 0)

    def clearMERR(self):
        # modifyRegister(REGISTER.MCP_EFLG, EFLG.EFLG_RX0OVR | EFLG.EFLG_RX1OVR, 0)
        # clearInterrupts()
        self.modifyRegister(REGISTER.MCP_CANINTF, CANINTF.CANINTF_MERRF, 0)

    def clearERRIF(self):
        # modifyRegister(REGISTER.MCP_EFLG, EFLG.EFLG_RX0OVR | EFLG.EFLG_RX1OVR, 0)
        # clearInterrupts()
        self.modifyRegister(REGISTER.MCP_CANINTF, CANINTF.CANINTF_ERRIF, 0)
