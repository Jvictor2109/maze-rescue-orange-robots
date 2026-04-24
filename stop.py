from serial_comm import SerialComm
import argparse

serial = SerialComm(
    port="/dev/ttyS0",
    baudrate=115200,
)

serial.send("MC 0 0 0 0")