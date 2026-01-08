#!/usr/bin/env python3

import os
import pty
import struct
import asyncio
import logging
import argparse
import serial_asyncio
import tty
import termios


logger = logging.getLogger(__name__)

# ================= CONSTANTS =================
class PtyDesc:
    def __init__(self, channel_id, name, plc_tx, plc_rx):
        self.channel_id = channel_id
        self.name = name
        self.plc_tx = plc_tx
        self.plc_rx = plc_rx

ptys = (PtyDesc(1, "BLE",    0x91, (0x91,)),
        PtyDesc(2, "OT",     0x92, (0x92,)),
        PtyDesc(3, "System", 0x90, (0x90,)),
        )

STFRAME_FI = 0xC0

# =============================================

def set_raw(fd):
    """Put a PTY file descriptor into raw mode."""
    attrs = termios.tcgetattr(fd)
    tty.setraw(fd)
    return attrs

def hexdump(data, width=16):
    out = []
    for i in range(0, len(data), width):
        chunk = data[i:i+width]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "."
                             for b in chunk)
        out.append(f"{i:08x}  {hex_part:<{width*3}} |{ascii_part}|")
    return "\n".join(out)

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else crc << 1
            crc &= 0xFFFF
    return crc

def build_frame(ch: int, payload: bytes, plc=None) -> bytes:
    if plc is None or plc == -1:
        plc = payload[0]
        payload = payload[1:]

    hdr = struct.pack("<BBHB", STFRAME_FI, 0x02, len(payload), plc)
    crc = crc16_ccitt(hdr + payload)
    return hdr + payload + struct.pack("<H", crc)

# ================= ST Frame RX =================

class STFrameReceiver:
#
# +------+-----+--------+-----+----------------+-----------+
# | FI   | FF  | Length | PLC | Payload        | CRC       |
# | 1 B  | 1 B | 2 B LE | 1 B | Length bytes   | 2 B LE    |
# +------+-----+--------+-----+----------------+-----------+
#
# FI = 0xC0
# FF (frame format) = 0x02 (CRC-16)
# PLC: protocol layer control
# CRC covers everything except CRC field itself
#
#
    def __init__(self, callback, log_rx, log_channels):
        self.buf = bytearray()
        self.in_frame = False
        self.callback = callback
        self.log_rx = log_rx
        self.log_channels = log_channels
        self.errors = 0

    def feed(self, data: bytes):
        for b in data:
            if not self.in_frame:
                if b == STFRAME_FI:
                    self.in_frame = True
                    self.buf.clear()
                    self.buf.append(b)
            else:
                self.buf.append(b)
                if self._handle_frame(bytes(self.buf)):
                    self.buf.clear()
                    self.in_frame = False

    def _handle_frame(self, raw: bytes) -> bool:
        if len(raw) < 5:
            return False

        try:
            fi, ff, length, plc = struct.unpack("<BBHB", raw[:5])
            total_len = 5 + length + 2
            if len(raw) < total_len:
                return False

            payload = raw[5:5+length]
            rx_crc, = struct.unpack("<H", raw[5+length:5+length+2])

            if crc16_ccitt(raw[:-2]) != rx_crc:
                self.errors += 1
                logger.warning(f"CRC error(errors: {self.errors:d}): expected {crc16_ccitt(raw[:-2]):4x}, received: {rx_crc:4x}")
                logger.warning("RX frame:\n%s", hexdump(raw))
                return True

            if self.log_rx and (not self.log_channels or plc in self.log_channels):
                logger.debug(f"RX frame (errors: {self.errors:d}):\n{hexdump(raw):s}")
                logger.debug("RX payload (plc=0x%02X):\n%s",
                             plc, hexdump(payload))

            self.callback(plc, payload)
        except Exception:
            logger.exception("Frame parse error")

        return True

# ================= MAIN =================

async def main():
    parser = argparse.ArgumentParser(description="UART MUX with logging filters")

    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="UART device (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=921600,
                        help="UART baudrate (default: 921600)")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    parser.add_argument("--log-rx", action="store_true",
                        help="Log RX frames only")
    parser.add_argument("--log-tx", action="store_true",
                        help="Log TX frames only")
    parser.add_argument("--channel", type=lambda x: int(x, 0),
                        action="append",
                        help="Log specific channel (hex or dec), repeatable")
    parser.add_argument("--channels", type=lambda x: int(x, 0),
                        nargs="+",
                        help="Log multiple channels")

    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(levelname)s: %(message)s"
    )

    log_rx = True
    log_tx = True

    if args.log_rx and not args.log_tx:
        log_tx = False
    elif args.log_tx and not args.log_rx:
        log_rx = False

    log_channels = set()
    if args.channel:
        log_channels.update(args.channel)
    if args.channels:
        log_channels.update(args.channels)

    # Create PTYs
    for pty_desc in ptys:
        pty_desc.master, pty_desc.slave = pty.openpty()

    for pty_desc in ptys:
        print (f"{pty_desc.name}: {os.ttyname(pty_desc.slave):s}")

    # Put all PTYs into raw mode (binary safe)
    for pty_desc in ptys:
        set_raw(pty_desc.master)
        set_raw(pty_desc.slave)

    # Build convenience ptys_by_chan_id
    ptys_by_chan_id = {}
    for pty_desc in ptys:
        ptys_by_chan_id[pty_desc.channel_id] = pty_desc

    reader, writer = await serial_asyncio.open_serial_connection(
        url=args.port,
        baudrate=args.baudrate
    )

    def on_frame(plc, payload):
        for pty_desc in ptys:
            if plc in pty_desc.plc_rx:
                data = payload
                if pty_desc.plc_tx == -1:
                    data = bytes([plc]) + data
                os.write(pty_desc.master, data)
                break

    frame_handler = STFrameReceiver(on_frame, log_rx, log_channels)

    async def uart_rx():
        while True:
            data = await reader.read(1024)
            if not data:
                break
            frame_handler.feed(data)

    def pty_reader(fd, ch):
        try:
            data = os.read(fd, 1024)
            if data:
                plc = ptys_by_chan_id[ch].plc_tx
                frame = build_frame(ch, data, plc)
                if log_tx and (not log_channels or ch in log_channels):
                    logger.debug("TX frame (ch=0x%02X):\n%s",
                                 ch, hexdump(frame))

                    logger.debug("TX data (ch=0x%02X):\n%s",
                                 ch, hexdump(data))

                writer.write(frame)
        except OSError:
            pass

    loop = asyncio.get_running_loop()
    for pty_desc in ptys:
        loop.add_reader(pty_desc.master, pty_reader, pty_desc.master, pty_desc.channel_id)

    print("UART MUX running.")
    await uart_rx()

if __name__ == "__main__":
    asyncio.run(main())
