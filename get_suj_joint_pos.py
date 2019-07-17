import serial
import numpy as np
import copy

ser = serial.Serial('COM3', baudrate=115200)

POT_Condition = [1, 1, 1, 1, 1, 1,
                 0, 1, 1, 1, 1, 1]
full_range = int('FFFFFF', 16)
pi = 3.1415926
joint_offset_suj1 = [-0.115, -pi, -pi, -pi, -pi, -pi]
joint_offset_suj2 = [-0.109, -pi, -pi, -pi, -pi, -pi]
joint_offset_ecm = [-0.141, -pi, -pi, -pi, -pi, -pi]

def get_suj_joint_reading():
    readings = []
    voltages = np.zeros((12, 1)).tolist()
    ser.write(b"AT+READALL\r\n")
    for i in range(13):
        readings.append(ser.read_until())
        # print(readings)
        if i == 12:
            if readings[i][-4:] == b"OK\r\n":
                print('Reading Success.')
            else:
                ser.reset_input_buffer()
                print("Reading Error.")

    for reading_ in readings[:-1]:
        reading_ = reading_.decode('utf-8')
        POT = int(reading_[-3], 16)
        voltage = int(reading_[0:6], 16) / full_range * 2.5
        voltages[POT] = voltage
    print(voltages)
    return voltages

def get_suj_joint_pos(voltages, suj_type):
    if suj_type == 'SUJ1':
        joint_pos = copy.deepcopy(joint_offset_suj1)
    elif suj_type == 'SUJ2':
        joint_pos = copy.deepcopy(joint_offset_suj2)
    else:
        joint_pos = copy.deepcopy(joint_offset_ecm)
    for joint_ in range(6):
        if joint_ == 0:
            if suj_type == 'SUJ1' or suj_type == 'SUJ2':
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_] \
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) / 2 \
                                    * (0.460422 if suj_type == 'SUJ1' else 0.462567)
            else:
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_] \
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) / 2 * 0.440917
        else:
            joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_] + voltages[joint_+6] * POT_Condition[joint_+6])\
                                / 2 / 2.5 * 2 * pi
    joint_pos[3] *= -1
    if suj_type == 'ECM':
        joint_pos[4] = joint_pos[0:4]
    return joint_pos


def release_brakes():
    ser.write(b"AT+FREEALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        print("All brakes are released successfully.")
    else:
        ser.reset_input_buffer()
        print("All Brakes Release Failed.")


def release_brakes_single(joint_num):
    if joint_num not in [1, 2, 3, 4, 5, 6]:
        print("Error: joint index out of range [1, 2,, 3, 4, 5, 6].")
        return

    joint_index = str(joint_num).encode()
    ser.write(b"AT+FREE=" + joint_index + b"\r\n")
    respond = ser.read_until()

    if respond == b"OK\r\n":
        print("Succeed to release joint brake: %d" % joint_num)
    else:
        ser.reset_input_buffer()
        print("Fail to release joint brake: %d" % joint_num)


def lock_brakes():
    ser.write(b"AT+LOCKALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        print("All brakes are locked successfully.")
    else:
        ser.reset_input_buffer()
        print("All Brakes Lock Failed.")


if __name__ == '__main__':
    reading = get_suj_joint_reading()
    suj_joint = get_suj_joint_pos(reading)
    suj_joint_deg = [180 / pi * i for i in suj_joint]
    suj_joint_deg[0] = suj_joint[0]
    print(suj_joint, '\n', suj_joint_deg)
    ser.close()
#print(reading)
