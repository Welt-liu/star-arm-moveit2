#!/usr/bin/env python
# coding: utf-8
"""Relax all cello servos (disable torque) so the arm can be moved by hand.

Usage:
    ros2 run robo_driver relax_servos

The driver node's node_close() currently does NOT release torque on
shutdown, so after killing the driver the arm remains locked in its
last commanded pose.  This tool opens the servo serial bus, broadcasts
disable_torque(0xff), and exits – leaving the arm free to be moved.

Prerequisites:
    * driver node must NOT be running (it holds the serial port)
    * serial port name matches the driver's (default /dev/ttyUSB0)
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

import fashionstar_uart_sdk as uservo

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Disable torque on all cello servos.")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args(argv)

    try:
        uart = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0,
        )
    except serial.SerialException as exc:
        print(f"ERROR: could not open {args.port}: {exc}", file=sys.stderr)
        print("Is the driver still running? Stop it first "
              "(Ctrl-C the driver.launch.py terminal).", file=sys.stderr)
        return 1

    try:
        mgr = uservo.UartServoManager(uart)
    except Exception as exc:
        print(f"ERROR: failed to init UartServoManager: {exc}", file=sys.stderr)
        uart.close()
        return 2

    print(f"Disabling torque on all servos via {args.port} @ {args.baud} baud...")
    try:
        mgr.disable_torque(0xff)
        time.sleep(0.15)   # let the broadcast land before closing the port
    finally:
        uart.close()

    print("Done.  Arm is relaxed – move it by hand, then restart the driver.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
