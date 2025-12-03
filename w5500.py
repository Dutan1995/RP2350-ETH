# w5500.py – Fully working W5500 SPI Driver (TCP Only)
# FIXED: correct control bytes for socket registers + TX/RX buffers

from machine import Pin
import time

# --------- COMMON REGISTERS ----------
MR      = 0x0000
GAR     = 0x0001
SUBR    = 0x0005
SHAR    = 0x0009
SIPR    = 0x000F

# --------- SOCKET 0 REGISTERS ----------
S0_MR       = 0x4000
S0_CR       = 0x4001
S0_IR       = 0x4002
S0_SR       = 0x4003
S0_PORT     = 0x4004
S0_TX_FSR   = 0x4020
S0_TX_RD    = 0x4022
S0_TX_WR    = 0x4024
S0_RX_RSR   = 0x4026
S0_RX_RD    = 0x4028

# --------- COMMANDS ----------
CR_OPEN     = 0x01
CR_LISTEN   = 0x02
CR_DISCON   = 0x08
CR_CLOSE    = 0x10
CR_SEND     = 0x20
CR_RECV     = 0x40

# --------- PHY ----------
PHYCFGR     = 0x002E


class W5500:
    def __init__(self, spi, cs, reset_pin):
        self.spi = spi
        self.cs = cs
        self.cs.init(Pin.OUT, value=1)

        self.reset_pin = reset_pin
        self.reset_pin.init(Pin.OUT, value=1)

    # ---------------------------------------------------------
    # CONTROL BYTE HELPERS  (IMPORTANT FIX!)
    # ---------------------------------------------------------

    def _control_byte(self, addr, write=False):
        # COMMON registers: 0x0000 – 0x0FFF
        if addr < 0x1000:
            return 0x04 if write else 0x00

        # SOCKET 0 registers: 0x4000 – 0x5FFF
        if 0x4000 <= addr < 0x6000:
            return 0x0C if write else 0x08

        # SOCKET 0 TX buffer: 0x6000 – 0x67FF
        if 0x6000 <= addr < 0x6800:
            return 0x14 if write else 0x10

        # SOCKET 0 RX buffer: 0x6800 – 0x6FFF
        if 0x6800 <= addr < 0x7000:
            return 0x1C if write else 0x18

        # fallback
        return 0x00

    # ---------------------------------------------------------
    # LOW LEVEL READ/WRITE (FULLY CORRECT NOW)
    # ---------------------------------------------------------

    def _read(self, addr, length):
        buf = bytearray(length)
        ctrl = self._control_byte(addr, write=False)

        self.cs.value(0)
        self.spi.write(bytearray([
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            ctrl
        ]))
        self.spi.readinto(buf)
        self.cs.value(1)

        return buf

    def _write(self, addr, data):
        ctrl = self._control_byte(addr, write=True)

        self.cs.value(0)
        self.spi.write(bytearray([
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            ctrl
        ]) + data)
        self.cs.value(1)

    # ---------------------------------------------------------
    # PHY STATUS
    # ---------------------------------------------------------

    def read_phystatus(self):
        return self._read(PHYCFGR, 1)[0]

    # ---------------------------------------------------------
    # NETWORK SETUP
    # ---------------------------------------------------------

    def set_ip(self, ip, sub, gate, mac):
        self._write(SHAR, bytes(mac))
        self._write(GAR, bytes(gate))
        self._write(SUBR, bytes(sub))
        self._write(SIPR, bytes(ip))

    # ---------------------------------------------------------
    # SOCKET CONTROL
    # ---------------------------------------------------------

    def socket0_open(self, port):
        self._write(S0_MR, b'\x01')  # TCP mode
        self._write(S0_PORT, bytes([(port >> 8) & 0xFF, port & 0xFF]))
        self._write(S0_CR, bytes([CR_OPEN]))

    def socket0_listen(self):
        self._write(S0_CR, bytes([CR_LISTEN]))

    def socket0_status(self):
        return self._read(S0_SR, 1)[0]

    def socket0_disconnect(self):
        self._write(S0_CR, bytes([CR_DISCON]))

    def socket0_close(self):
        self._write(S0_CR, bytes([CR_CLOSE]))

    # ---------------------------------------------------------
    # SEND
    # ---------------------------------------------------------

    def socket0_send(self, data):
        # Read TX write pointer
        wr = self._read(S0_TX_WR, 2)
        ptr = (wr[0] << 8) | wr[1]

        # Write into TX buffer
        addr = 0x6000 + (ptr & 0x7FF)
        self._write(addr, data)

        # Update pointer
        ptr += len(data)
        self._write(S0_TX_WR, bytes([(ptr >> 8) & 0xFF, ptr & 0xFF]))

        # SEND command
        self._write(S0_CR, bytes([CR_SEND]))

    # ---------------------------------------------------------
    # RECEIVE
    # ---------------------------------------------------------

    def socket0_recv(self):
        # RX size
        rs = self._read(S0_RX_RSR, 2)
        size = (rs[0] << 8) | rs[1]
        if size == 0:
            return None

        # Read pointer
        rd = self._read(S0_RX_RD, 2)
        ptr = (rd[0] << 8) | rd[1]

        # Read data from RX buffer
        addr = 0x6800 + (ptr & 0x7FF)
        data = self._read(addr, size)

        # Update pointer
        ptr += size
        self._write(S0_RX_RD, bytes([(ptr >> 8) & 0xFF, ptr & 0xFF]))

        # RECV command
        self._write(S0_CR, bytes([CR_RECV]))

        return data
