#!/usr/bin/env python
import serial
import time
import numpy as np
import copy

ser1 = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
                    write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
ser2 = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
                    write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
ser3 = serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
                    write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)

POT_Condition_suj1 = [1, 1, 1, 1, 1, 1,
                        0, 1, 1, 1, 1, 1]                         
POT_Condition_suj2 = [1, 1, 1, 1, 1, 1,
                      1, 1, 1, 1, 1, 1]

POT_Condition_ecm = [1, 1, 1, 1, 1, 1,
                          0, 1, 1, 1, 1, 1]

full_range = int('FFFFFF', 16)
# pi = np.pi

# digital readings to degrees/meters
reading_ratios_suj1 = [0.0, 1.642306388237421e-05, -180, -180, -180, -180]
reading_ratios_suj2 = [0.0, -180, -180, -180, -180, -180]
reading_ratios_ecm = [0.0, -180, -180, -180, -180, -180]


reading_offset_suj1 = [-0.115, -1.372810126705418e+02, -180, -180, -180, -180]
reading_offset_suj2 = [-0.109, -180, -180, -180, -180, -180]
reading_offset_ecm = [-0.141, -180, -180, -180, -180, -180]


def get_suj_joint_reading(serial_port):
    readings = []
    POT_sum = 0
    valid_reading = True
    # voltages = np.zeros((12, 1)).tolist()
    voltages = [0 for i in range(12)]
    reading_list = [0 for i in range(12)]

    serial_port.write(b"AT+ADDRESS\r\n")
    serial_port_address = serial_port.read_until().decode('utf-8')
    address_num = int(serial_port_address[0], 16)
    isGetAddressScuess = serial_port.read_until()
    if isGetAddressScuess == b"OK\r\n":
        print('Port Address Obtained Successfully')
    else:
        serial_port.reset_input_buffer()
        print('Failed to Get Port Address')

    if address_num == 3:
        arm = 'SUJ1'
    elif address_num == 1:
        arm = 'SUJ2'
    else:
        arm = "ECM"

    serial_port.write(b"AT+READALL\r\n")
    for i in range(13):
        readings.append(serial_port.read_until())
        if i == 12:
            if readings[i][-4:] == b"OK\r\n":
                print('Reading Success.')
            else:
                serial_port.reset_input_buffer()
                print("Reading Error.")
    # print(readings)
    for reading_ in readings[:-1]:
        reading_ = reading_.decode('utf-8')
        POT = int(reading_[-3], 16)
        POT_sum += POT
        voltage = float(int(reading_[0:6], 16)) / float(full_range) * 2.5
        read_value = int(reading_[0:6], 16)
        reading_list[POT] = read_value
        voltages[POT] = voltage
    # It is noted that the readings can contain duplicate data from
    # the ADC due to the hardware reason. And the duplicate readings at the first and last position in the 12 readings.
    # ***** Check method 1 *****
    # if readings[0].decode('utf-8')[-3] == readings[11].decode('utf-8').[-3] :
    #   valid_reading = False
    # ***** Check method 2 *****
    # 66 = sum(0 to 11)
    if POT_sum != 66:
        valid_reading = False
    print(arm + ' reading:')
    print(readings)
    return voltages, reading_list, valid_reading, arm


def get_suj_joint_pos(voltages, suj_type):
    if suj_type == 'SUJ1':
        joint_pos = copy.deepcopy(joint_offset_suj1)
        POT_Condition = POT_Condition_suj1_ecm
    elif suj_type == 'SUJ2':
        joint_pos = copy.deepcopy(joint_offset_suj2)
        POT_Condition = POT_Condition_suj2
    else:
        joint_pos = copy.deepcopy(joint_offset_ecm)
        POT_Condition = POT_Condition_suj1_ecm

    for joint_ in range(6):
        if joint_ == 0:
            if suj_type == 'SUJ1':
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_]
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) * 0.460422
            elif suj_type == 'SUJ2':
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_]
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) / 2 * 0.462567

            else:
                joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_]
                                    + voltages[joint_+6] * POT_Condition[joint_+6]) / 2 * 0.440917
        else:
            joint_pos[joint_] += (voltages[joint_] * POT_Condition[joint_] + voltages[joint_+6] * POT_Condition[joint_+6])\
                                / 2 / 2.5 * 2 * pi
    joint_pos[3] *= -1
    if suj_type == 'ECM':
        joint_pos = joint_pos[0:4]
    return joint_pos

# transform digital readings to degrees values 
def dReading2degree(d_reading, suj_type, ratio_list, bias_list, POT_Condition):
    joint_pos_deg = [0 for i in range(6)]
    joint_pos_read = [0 for i in range(6)]
    for joint_ in range(6):
            if (POT_Condition[joint_]+POT_Condition[joint_+6])==2:
                joint_pos_read[joint_] += (d_reading[joint_] * POT_Condition[joint_] + 
                                            d_reading[joint_+6] * POT_Condition[joint_+6])/ 2

            elif (POT_Condition[joint_]+POT_Condition[joint_+6])==1:
                joint_pos_read[joint_] += (d_reading[joint_] * POT_Condition[joint_] + 
                                d_reading[joint_+6] * POT_Condition[joint_+6])
            else:
                raise Exception('POT_condition should either 1 or 2, now is {}'.format(POT_Condition[joint_]+POT_Condition[joint_+6]))

            joint_pos_deg[joint_] = joint_pos_read[joint_] * ratio_list[joint_]\
                        + bias_list[joint_]


    return joint_pos_read, joint_pos_deg


def release_brakes(ser):
    ser.write(b"AT+FREEALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        print("All brakes are released successfully.")
    else:
        ser.reset_input_buffer()
        print("All Brakes Release Failed.")


def release_brakes_single(joint_num, ser):
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

def control_brakes(is_release_brake_list, ser):
    if len(bool_list) == 6:
        for i in range(6):
            if is_release_brake_list[i]:
                release_brakes_single(i+1, ser)
            else:
                lock_brakes_single(i+1, ser)
    else:
        print('length  of control brake list should be 6')

def lock_brakes_single(joint_num, ser):
    if joint_num not in [1, 2, 3, 4, 5, 6]:
        print("Error: joint index out of range [1, 2,, 3, 4, 5, 6].")
        return

    joint_index = str(joint_num).encode()
    ser.write(b"AT+LOCK=" + joint_index + b"\r\n")
    respond = ser.read_until()

    if respond == b"OK\r\n":
        print("Succeed to lock joint brake: %d" % joint_num)
    else:
        ser.reset_input_buffer()
        print("Fail to lock joint brake: %d" % joint_num)

def lock_brakes(ser):
    ser.write(b"AT+LOCKALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        print("All brakes are locked successfully.")
    else:
        ser.reset_input_buffer()
        print("All Brakes Lock Failed.")


# def get_data():
#     suj1_joint = [0 for i in range(6)]
#     suj2_joint = [0 for i in range(6)]
#     ecm_joint = [0 for i in range(4)]
#     allIsValid = False
#     while not(allIsValid):
#         # v_reading1~3 are the voltage readings
#         # d_reading1~3 are the original digital readings
#         [v_reading1, d_reading1, isValid1, arm1] = get_suj_joint_reading(ser1)
#         [v_reading2, d_reading2, isValid2, arm2] = get_suj_joint_reading(ser2)
#         [v_reading3, d_reading3, isValid3, arm3] = get_suj_joint_reading(ser3)
#         allIsValid = (isValid1 and isValid2 and isValid3)

#     armSerialport = {}
#     if arm1 == 'SUJ1':
#         suj1_joint = get_suj_joint_pos(v_reading1, 'SUJ1')
#         armSerialport['SUJ1'] = ser1
#     elif arm1 == 'SUJ2':
#         suj2_joint = get_suj_joint_pos(v_reading1, 'SUJ2')
#         armSerialport['SUJ2'] = ser1
#     else:
#         ecm_joint = get_suj_joint_pos(v_reading1, 'ECM')
#         armSerialport['ECM'] = ser1

#     if arm2 == 'SUJ1':
#         suj1_joint = get_suj_joint_pos(v_reading2, 'SUJ1')
#         armSerialport['SUJ1'] = ser2
#     elif arm2 == 'SUJ2':
#         suj2_joint = get_suj_joint_pos(v_reading2, 'SUJ2')
#         armSerialport['SUJ2'] = ser2
#     else:
#         ecm_joint = get_suj_joint_pos(v_reading2, 'ECM')
#         armSerialport['ECM'] = ser2


#     suj1_joint_deg = [180 / pi * i for i in suj1_joint]
#     suj2_joint_deg = [180 / pi * i for i in suj2_joint]
#     ecm_joint_deg = [180 / pi * i for i in ecm_joint]
#     suj1_joint_deg[0] = suj1_joint[0]
#     suj2_joint_deg[0] = suj2_joint[0]
#     ecm_joint_deg[0] = ecm_joint[0]

#     # ser1.close()
#     # ser2.close()
#     # ser3.close()
#     # return [suj1_joint, suj2_joint, ecm_joint], armSerialport
#     return [suj1_joint_deg, suj2_joint_deg, ecm_joint_deg], armSerialport


def readSerial(ser):
    isValid = False
    while not(isValid):
        # v_reading1~3 are the voltage readings
        # d_reading1~3 are the original digital readings
        [v_reading, d_reading, isValid, arm_str] = get_suj_joint_reading(ser)

    if arm_str == 'SUJ1':
        joint_pos_read, joint_pos_deg = dReading2degree(d_reading,\
                                        arm_str,\
                                        reading_ratios_suj1,\
                                        reading_offset_suj1,\
                                        POT_Condition_suj1)
    if arm_str == 'SUJ2':
        joint_pos_read, joint_pos_deg = dReading2degree(d_reading,\
                                        arm_str,\
                                        reading_ratios_suj2,\
                                        reading_offset_suj2,\
                                        POT_Condition_suj2)
    if arm_str == 'ECM':
        joint_pos_read, joint_pos_deg = dReading2degree(d_reading,\
                                        arm_str,\
                                        reading_ratios_ecm,\
                                        reading_offset_ecm,\
                                        POT_Condition_ecm)

    return joint_pos_read, joint_pos_deg, arm_str

def readAll(ser1, ser2, ser3):
    armSerialportDic = {}
    joint_pos_read_dict = {}
    joint_pos_deg_dict = {}
    joint_pos_read, joint_pos_deg, arm_str = readSerial(ser1)
    armSerialportDic[arm_str] = ser1
    joint_pos_read_dict[arm_str] = joint_pos_read
    joint_pos_deg_dict[arm_str] = joint_pos_deg

    joint_pos_read, joint_pos_deg, arm_str = readSerial(ser2)
    armSerialportDic[arm_str] = ser2
    joint_pos_read_dict[arm_str] = joint_pos_read
    joint_pos_deg_dict[arm_str] = joint_pos_deg

    joint_pos_read, joint_pos_deg, arm_str = readSerial(ser3)
    armSerialportDic[arm_str] = ser3
    joint_pos_read_dict[arm_str] = joint_pos_read
    joint_pos_deg_dict[arm_str] = joint_pos_deg

    print('\n\nSUJ1 --> SUJ2 --> ECM')
    print('\n digital readings SUJ1 --> SUJ2 --> ECM')
    print(joint_pos_read_dict['SUJ1'])
    print(joint_pos_read_dict['SUJ2'])
    print(joint_pos_read_dict['ECM'])
    print('\n\n degree SUJ1 --> SUJ2 --> ECM')  
    print(joint_pos_deg_dict['SUJ1'])
    print(joint_pos_deg_dict['SUJ2'])
    print(joint_pos_deg_dict['ECM'])

    return armSerialportDic, joint_pos_read_dict, joint_pos_deg_dict


if __name__ == '__main__':
    armSerialportDic = {}
    joint_pos_read_dict = {}
    joint_pos_deg_dict = {}
    print('Initial Reading:')
    # serial connection and intial reading
    armSerialportDic, joint_pos_read_dict, joint_pos_deg_dict = readAll(ser1, ser2, ser3)

    


    while True:
        print('\n--->Choose Mode:')
        action_mode = input('1: Release Single Joint \n2: elease All Joints \
                            \n3: Read Joint position \n4: Lock All Joints \n5: Exit\n')
        if action_mode == 1:
            print('Use Input List Form: [ArmIndex, JointIndex] \nSUJ1: 1 \nSUJ2: 2 \nECM: 3')
            print('SUJ1 & SUJ2 Joint Index: 1-6 \nECM JOint INdex: 1-4')
            user_input = input('Example Input: [1, 1] \n')
            arm = user_input[0]
            joint = user_input[1]
            if arm == 1:
                release_brakes_single(joint, armSerialportDic['SUJ1'])
            elif arm == 2:
                release_brakes_single(joint, armSerialportDic['SUJ2'])
            elif arm == 3:
                release_brakes_single(joint, armSerialportDic['ECM'])
            else:
                print('Error: Invalid Arm Index')

        elif action_mode == 2:
            armIndex = input('Input the Arm Index to Release:\nSUJ1:1  SUJ2:2  ECM:3\n')
            if armIndex == 1:
                release_brakes(armSerialportDic['SUJ1'])
            elif armIndex == 2:
                release_brakes(armSerialportDic['SUJ2'])
            elif armIndex == 3:
                release_brakes(armSerialportDic['ECM'])
            else:
                print('Error: Invalid Arm Index')

        elif action_mode == 3:
            armSerialportDic, joint_pos_read_dict, joint_pos_deg_dict = readAll(ser1, ser2, ser3)

        elif action_mode == 4:
            lock_brakes(ser1)
            lock_brakes(ser2)
            lock_brakes(ser3)

        elif action_mode == 5:
            break
        elif action_mode == 6:
            arm_index = input('input arm index:')
            control_list = input('input is_brake_control_list')
            bool_list = [bool(i) for i in control_list]
            print(bool_list)
            if arm_index == 1:
                control_brakes(bool_list, armSerialportDic['SUJ1'])
            elif arm_index == 2:
                control_brakes(bool_list, armSerialportDic['SUJ2'])
            elif arm_index == 3:
                control_brakes(bool_list, armSerialportDic['ECM'])
            else:
                print('Error: Invalid Arm Index')
        else:
            print('Error: Invalid Mode')
            break

    print('Auto Lock All Joints ......')
    lock_brakes(ser1)
    lock_brakes(ser2)
    lock_brakes(ser3)
    ser1.close()
    ser2.close()
    ser3.close()
    print('Lock Successfully')

