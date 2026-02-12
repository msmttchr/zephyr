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
import sys
import subprocess
import time
import re
from datetime import datetime

logger = logging.getLogger(__name__)

# ================= CONSTANTS =================
class PtyDesc:
    def __init__(self, channel_id, name, plc_tx, plc_rx):
        self.channel_id = channel_id
        self.name = name
        self.plc_tx = plc_tx
        self.plc_rx = plc_rx
        self.master = None
        self.slave = None

ptys = (PtyDesc(1, "BLE",    0x91, (0x91,)),
        PtyDesc(2, "OT",     0x92, (0x92,)),
        PtyDesc(3, "System", 0x90, (0x90,)),
        )

STFRAME_FI = 0xC0
# ================= LOGGING & TERMINAL FIX =================
class RawFormatter(logging.Formatter):
    def format(self, record):
        # Replaces all \n with \r\n to prevent "staircase" effect in cbreak mode
        return super().format(record).replace('\n', '\r\n') + '\r'

logger = logging.getLogger(__name__)
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(RawFormatter("%(levelname)s: %(message)s"))
logger.addHandler(handler)
logger.propagate = False

# Redefine print to always append \r
def raw_print(*args, **kwargs):
    kwargs["end"] = "\r\n"
    print(*args, **kwargs)

# ================= UTILS =================

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

def get_status (proc):
    if proc is None:
        return None

    if proc.returncode is not None:
        pid_status = "dead"
    else:
        from utils import get_process_info
        pid_status = get_process_info(proc.pid)['status']

    return f"{proc.command} {pid_status} on {proc.slave_path} (PID: {proc.pid})"

def print_status(args, bt_proc=None, hci_dev=None, ot_proc=None):
    """Enhanced status message for the MUX configuration."""
    bt_status = get_status(bt_proc)
    ot_status = get_status(ot_proc)
    if hci_dev:
        from utils import get_hci_status
        status = get_hci_status(hci_dev)
        hci_dev_status = f"{status.get('powered')} ({status.get('address')})"
    msg = [
        f"============================================================",
        f"                 UART MULTIPLEXER STATUS",
        f"============================================================",
        f" Date:        {datetime.now()}",
        f" Port:        {args.port}",
        f" Baudrate:    {args.baudrate}",
        f" Flow Ctrl:   {'Hardware (RTS/CTS)' if args.rtscts else 'None'}",
        f" Bluetooth:   {bt_status}" if bt_status else None,
        f" HCI Intf:    {hci_dev}:{hci_dev_status}" if hci_dev else None,
        f" OT daemon:   {ot_status}" if ot_status else None,
        f"------------------------------------------------------------",
        f" {'CHANNEL':<12} | {'PTY DEVICE'}",
        f"------------------------------------------------------------",
    ]
    for pty_desc in ptys:
        msg.append(f" {pty_desc.name:<12} | {os.ttyname(pty_desc.slave)}")

    msg += [
        f"------------------------------------------------------------",
        f" [ Press 'i' to show this status  |  'q' to Exit  | ",
        f"   Ctrl+C to Exit ]",
        f"============================================================",
    ]
    msg = (x for x in msg if x is not None)
    msg = "\r\n".join(msg)
    raw_print(msg)

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
                logger.warning(f"CRC error(errors: {self.errors:d}): expected {crc16_ccitt(raw[:-2]):4x}, received: {rx_crc:4x}\r")
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
    parser.add_argument("--baudrate", type=int, default=2000000,
                        help="UART baudrate (default: 2000000)")
    parser.add_argument("--rtscts", action=argparse.BooleanOptionalAction, default=True,
                        help="Enable/Disable hardware flow control (default: enabled)")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    parser.add_argument("--log-rx", action="store_true",
                        help="Log RX frames only")
    parser.add_argument("--bt-attach", action="store_true", help="Automatically run btattach on the BLE PTY")
    parser.add_argument("--ot-manager", default="None",
                         choices=["None", "ot-daemon", "otbr-agent"],
                         help="Automatically run selected manager on OT PTY")
    parser.add_argument("--backbone", default="eth0",
                         help="Backbone network interface for otbr-agent (default: eth0)")
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

    # Serial Connection
    reader, writer = await serial_asyncio.open_serial_connection(
        url=args.port,
        baudrate=args.baudrate,
        rtscts=args.rtscts,
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
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                frame_handler.feed(data)
        except asyncio.CancelledError:
            # Optional: cleanup UART, flush buffers, log, etc.
            raw_print("UART RX cancelled")

    def pty_reader(fd, ch):
        try:
            data = os.read(fd, 1024)
            if data:
                plc = ptys_by_chan_id[ch].plc_tx
                frame = build_frame(ch, data, plc)
                if log_tx and (not log_channels or ch in log_channels):
                    logger.debug(f"TX frame (ch={ch:02x}, plc={plc:02x}):\r\n{hexdump(frame):s}\r")
                    logger.debug(f"TX data:\r\n{hexdump(data):s}\r")
                writer.write(frame)
        except OSError:
            pass

    # Store loop in outer scope so stdin_handler can stop it
    loop = asyncio.get_running_loop()

    def stdin_handler(stop_event):
        char = sys.stdin.read(1)
        if not char:
            return
        if char.lower() == 'i':
            print_status(args, bt_proc, hci_dev, ot_proc)
        elif char.lower() == 'q':
            raw_print("\r\nExiting on 'q'...")
            stop_event.set()

    # Identify BLE and OT PTY for Bluetooth/OT attachment
    ble_pty = next((p for p in ptys if p.name == "BLE"), None)
    bt_proc = None
    hci_dev = None

    ot_pty = next((p for p in ptys if p.name == "OT"), None)
    ot_proc = None

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Restore old settings as previous subprocess may have changed them
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        stop_event = asyncio.Event()
        tty.setcbreak(sys.stdin.fileno())
        loop.add_reader(sys.stdin, stdin_handler, stop_event)
        for p_desc in ptys:
            loop.add_reader(p_desc.master, pty_reader, p_desc.master, p_desc.channel_id)

        rx_task = asyncio.create_task(uart_rx())

        if args.bt_attach and ble_pty:
            from utils import get_last_hci_interface
            # We must use the slave path for btattach
            slave_path = os.ttyname(ble_pty.slave)
            try:
                params = ["sudo", "btattach", "-B", slave_path, "-S", str(args.baudrate), "-P", "h4"]
                bt_proc = await asyncio.create_subprocess_exec(
                    *params,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )
                bt_proc.command_line = " ".join(params)
                bt_proc.command = params[1]
                bt_proc.slave_path = slave_path
            except Exception as e:
                logger.error(f"btattach failed to start: {e}")

            await asyncio.sleep(1.5)
            hci_dev = await get_last_hci_interface()
        if args.ot_manager != "None" and ot_pty:
            slave_path = os.ttyname(ot_pty.slave)
            daemon = args.ot_manager
            daemon_args = [daemon]
            if daemon == "otbr-agent":
                daemon_args += ["-I", "wpan0", "-B", args.backbone]

            daemon_args.append(f"spinel+hdlc+uart://{slave_path}?uart-baudrate={args.baudrate}&{'uart-flow-control' if args.rtscts else ''}")
            daemon_args = ["sudo"] + daemon_args
            try:
                ot_proc = await asyncio.create_subprocess_exec(
                    *daemon_args,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )
                ot_proc.command_line = " ".join(daemon_args)
                ot_proc.command = daemon_args[1]
                ot_proc.slave_path = slave_path
                await asyncio.sleep(1.5)
            except Exception as e:
                logger.error(f"{daemon} failed to start: {e}")

        print_status(args, bt_proc, hci_dev, ot_proc)
        await stop_event.wait()          # wait for 'q'
        rx_task.cancel()                 # interrupt reader.read()
        await rx_task                    # let it exit cleanly
    except Exception as e:
        print (e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        raw_print("\r\nExiting...")
