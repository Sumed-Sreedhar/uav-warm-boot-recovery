#!/usr/bin/env python3
"""Check the USART1 loopback firmware with both Linux serial ports open."""

import argparse
import os
import random
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager


@contextmanager
def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        saved = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            settings = termios.tcgetattr(fd)
            settings[2] &= ~(termios.CRTSCTS | termios.CSTOPB)
            settings[2] |= termios.CLOCAL | termios.CREAD
            settings[4] = settings[5] = baud
            termios.tcsetattr(fd, termios.TCSANOW, settings)
            yield fd
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, saved)
    finally:
        os.close(fd)


def exchange(data_fd, log_fd, expected):
    received = bytearray()
    log = bytearray()
    sent = 0
    deadline = time.monotonic() + 3
    last_receive = None
    while True:
        now = time.monotonic()
        if last_receive is not None and len(received) >= len(expected):
            # Keep reading briefly to catch trailing bytes and delayed diagnostics.
            if now >= last_receive + 0.15:
                break
        if now >= deadline:
            break
        ready, writable, _ = select.select(
            [data_fd, log_fd], [data_fd] if sent < len(expected) else [], [], 0.05
        )
        for fd in ready:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if fd == data_fd:
                received.extend(chunk)
                if chunk:
                    last_receive = time.monotonic()
            else:
                log.extend(chunk)
        if writable:
            try:
                sent += os.write(data_fd, expected[sent:])
            except BlockingIOError:
                pass

    if log:
        print(f"USART2: {log.decode(errors='replace').rstrip()}")
    if sent != len(expected) or received != expected or b"USART1 loopback:" in log:
        mismatch = next(
            (i for i, (want, got) in enumerate(zip(expected, received)) if want != got),
            min(len(expected), len(received)),
        )
        raise RuntimeError(
            f"{len(expected)} expected, {sent} sent, {len(received)} received, "
            f"{mismatch} matching prefix bytes"
        )


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("ftdi", help="USART1 adapter at 921600 baud")
    cli.add_argument("stlink", help="USART2 console at 115200 baud")
    args = cli.parse_args()
    if os.path.samefile(args.ftdi, args.stlink):
        cli.error("Select two different serial devices")

    rng = random.Random(0xF446)
    with (
        open_port(args.stlink, termios.B115200) as log_fd,
        open_port(args.ftdi, termios.B921600) as data_fd,
    ):
        for length in (513, 16384):
            exchange(data_fd, log_fd, rng.randbytes(length))
            print(f"PASS: {length} bytes echoed exactly, no USART1 loopback error", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError) as error:
        sys.exit(f"FAIL: {error}")
