#!/usr/bin/env python3
"""Tiny raw serial logger for the tethered-agent session.

No pyserial. Uses os.open + termios, 115200 8N1 raw.

- reads continuously, timestamps every line (ms since open), appends to the log
- to send a character, write it into "<log>.cmd" (the file is then cleared)
- exits after --seconds, or early if a line contains --stop-on

Usage:
  python3 -I serial_bridge.py --port /dev/cu.usbserial-10 \
      --log results/2026-10-17/raw_step2.log --seconds 40 \
      [--stop-on "PID START"] [--send j]
"""

import argparse
import os
import select
import sys
import termios
import time


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    speed = getattr(termios, "B115200", 115200)
    new = [iflag, oflag, cflag, lflag, speed, speed, cc]
    termios.tcsetattr(fd, termios.TCSANOW, new)
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--stop-on", default=None)
    ap.add_argument("--send", default=None)
    args = ap.parse_args()

    cmd_path = args.log + ".cmd"

    # Never overwrite: pick a free log name if the target exists.
    log_path = args.log
    n = 1
    while os.path.exists(log_path):
        log_path = "%s.%d" % (args.log, n)
        n += 1

    fd = open_port(args.port)
    logf = open(log_path, "a", buffering=1)

    start = time.monotonic()

    def emit(text):
        line = "[+%8.3f s] %s" % (time.monotonic() - start, text)
        logf.write(line + "\n")
        sys.stdout.write(line + "\n")
        sys.stdout.flush()

    emit("=== open %s (log %s) ===" % (args.port, log_path))
    if args.send:
        os.write(fd, args.send.encode())
        emit("=== sent: %r ===" % args.send)

    buf = b""
    stop = False
    while not stop:
        if time.monotonic() - start > args.seconds:
            emit("=== timeout after %.1f s ===" % args.seconds)
            break

        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                data = b""
            buf += data
            while True:
                idxs = [i for i in (buf.find(b"\n"), buf.find(b"\r")) if i != -1]
                if not idxs:
                    break
                i = min(idxs)
                line = buf[:i]
                buf = buf[i + 1:]
                if buf[:1] == b"\n":
                    buf = buf[1:]
                text = line.decode("latin-1").rstrip()
                emit(text)
                if args.stop_on and args.stop_on in text:
                    emit("=== stop-on matched ===")
                    stop = True
                    break

        # out-of-band command file
        if os.path.exists(cmd_path):
            try:
                with open(cmd_path) as f:
                    cmd = f.read()
                os.remove(cmd_path)
            except OSError:
                cmd = ""
            if cmd.strip():
                os.write(fd, cmd.strip().encode())
                emit("=== sent: %r ===" % cmd.strip())

    # flush any trailing partial line
    if buf:
        emit(buf.decode("latin-1").rstrip())

    logf.close()
    os.close(fd)


if __name__ == "__main__":
    main()
