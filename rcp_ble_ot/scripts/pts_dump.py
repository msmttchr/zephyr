#!/usr/bin/env python3

import os
import termios
import tty
import argparse
import logging
import struct
import sys

logger = logging.getLogger("ptydump")

def equivalent_bash_cmd(port, width):
    return (
        f"stty -F {port} raw -echo && "
        f"stdbuf -o0 hexdump -C {port}"
    )

def hexdump(data, offset, width):
    for i in range(0, len(data), width):
        chunk = data[i:i+width]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{offset+i:08x}  {hex_part:<{width*3}} |{ascii_part}|")

def main():
    parser = argparse.ArgumentParser(
        description="Live hex dump of a PTY or serial device"
    )
    parser.add_argument(
        "-p", "--port",
        default="/dev/pts/5",
        help="Device path (default: /dev/pts/5)"
    )
    parser.add_argument(
        "-w", "--width",
        type=int,
        default=8,
        help="Bytes per line (default: 8)"
    )
    parser.add_argument(
        "-d", "--delay",
        type=int,
        default=1000,
        help="Expected delay in ms between timestamps (default: 1000)"
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="Do NOT put device into raw mode (raw is default)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="count",
        default=0,
        help="Increase verbosity (-v, -vv)"
    )
    parser.add_argument(
        "--show-cmd",
        action="store_true",
        help="Print equivalent bash command and exit"
    )

    args = parser.parse_args()

    # Configure logging
    if args.verbose == 0:
        loglevel = logging.WARNING
    elif args.verbose == 1:
        loglevel = logging.INFO
    else:
        loglevel = logging.DEBUG

    logging.basicConfig(
        level=loglevel,
        format="%(levelname)s: %(message)s"
    )

    # Show equivalent bash command and exit
    if args.show_cmd:
        print("Equivalent bash command:\n")
        print(equivalent_bash_cmd(args.port, args.width))
        sys.exit(0)

    logger.info("Opening device: %s", args.port)

    fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY)

    old_attrs = None
    if not args.no_raw:
        logger.info("Setting raw mode (default)")
        old_attrs = termios.tcgetattr(fd)
        tty.setraw(fd)
    else:
        logger.info("Raw mode disabled by user")

    print(
        f"Dumping {args.port} (width={args.width}, raw={'no' if args.no_raw else 'yes'})"
    )
    print("Press Ctrl+C to stop\n")

    offset = 0
    last_val = None
    errors = 0
    try:
        while True:
            data = os.read(fd, 1024)
            if not data:
                logger.info("EOF reached")
                break

            logger.debug("Read %d bytes at offset %d", len(data), offset)
            hexdump(data, offset, args.width)
            offset += len(data)
            val, = struct.unpack("<Q", data)
            if last_val is not None:
                delta = val-last_val
                if delta > round(args.delay * 1.01) or delta < round(args.delay * 0.99):
                    errors += 1
                print(f"Timer: {val} ms, delta: {delta} ms, errors: {errors}")
            last_val = val
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        if old_attrs is not None:
            logger.info("Restoring terminal settings")
            termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
        os.close(fd)
        logger.info("Device closed")

if __name__ == "__main__":
    main()
