#!/usr/bin/env python
import os
if os.uname()[4][:3] == 'arm':
    Raspberry = True
    import serial
else:
    Raspberry = False
import time


class SerialCommand(object):
    def __init__(self, device='/dev/ttyUSB0', baudrate=9600, timeout=0):
        if Raspberry:
            parity = serial.PARITY_NONE
            stopbits = serial.STOPBITS_ONE
            bytesize = serial.EIGHTBITS
        else:
            return
        try:
            self.serial_port = serial.Serial(device, baudrate=baudrate,
                                             parity=parity,
                                             stopbits=stopbits,
                                             bytesize=bytesize,
                                             timeout=timeout)
        except serial.serialutil.SerialException as error:
            raise SystemExit(error)

        if not self.serial_port.is_open:
            self.serial_port.open()

    def send(self, command):
        if Raspberry:
            if not self.serial_port.is_open:
                self.serial_port.open()
            self.serial_port.write(bytearray(command, encoding="utf-8"))

    def send_wait_result(self, command):
        if Raspberry:
            self.send(command)
            time.sleep(.3)
            GetResult = False
            while not GetResult:
                time.sleep(.4)
                r = self.get()
                if len(r) > 0:
                    return r

    def get(self):
        if Raspberry:
            if not self.serial_port.is_open:
                self.serial_port.open()
            hello = self.serial_port.readline()
            hello_str = hello.decode('ascii')
            return hello_str

    def close(self):
        if Raspberry:
            self.serial_port.close()

    def __del__(self):
        if Raspberry:
            self.serial_port.close()
