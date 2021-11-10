import serial
from enum import Enum
from pprint import pprint


class SujType(Enum):
    ECM = 0
    SUJ1 = 1
    SUJ2 = 2


serial_device_dict = {
    SujType.ECM: serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.2),
    SujType.SUJ1: serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.2),
    SujType.SUJ2: serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=0.2)
}


def free_all_brakes(suj_type):
    global serial_device_dict
    serial_device_dict[suj_type].write(b"AT+FREEALL\r\n")
    res = serial_device_dict[suj_type].read_until()
    return res


def lock_all_brakes(suj_type):
    global serial_device_dict
    serial_device_dict[suj_type].write(b"AT+LOCKALL\r\n")
    res = serial_device_dict[suj_type].read_until()
    return res


def free_brake(suj_type, joint_num):
    global serial_device_dict

    index = str(joint_num).encode()
    serial_device_dict[suj_type].write(b"AT+FREE=" + index + b"\r\n")
    res = serial_device_dict[suj_type].read_until()
    return res
