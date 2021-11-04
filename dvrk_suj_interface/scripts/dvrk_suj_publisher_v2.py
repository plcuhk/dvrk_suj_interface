#!/usr/bin/env python
from re import S
from rospy.impl.rosout import RosOutHandler
import serial
import time
import numpy as np
import copy
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dvrk_suj_interface.msg import Bool_List

from enum import Enum
from pprint import pprint

class SujType(Enum):
    SUJ2 = 1
    ECM = 2
    SUJ1 = 3


full_range = int('FFFFFF', 16)
pi = np.pi

# global dictioanry
serial_devices_list = {
    serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.2),
    serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.2),
    serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=0.2)
}
# initialize the dict of serial device in runtime
serial_devices_dict = {}

# const global dictionary
pot_condition_dict = {
    SujType.ECM: [1, 1, 1, 1, 1, 1,
                  0, 1, 1, 1, 1, 1],
    SujType.SUJ1: [1, 1, 1, 1, 1, 1,
                   0, 1, 1, 1, 1, 1],
    SujType.SUJ2: [1, 1, 1, 1, 1, 1,
                   1, 1, 1, 1, 1, 1]
}

# digital readings to degrees/meters
reading_ratios_dict = {
    SujType.ECM: [
        0.000651841047138e-04,
        0.164475557136178e-04,
        0.212760655892334e-04,
        -0.213315053290246e-04,
        0e-04,
        0e-04
    ],
    SujType.SUJ1: [
        0.000679361324099e-04,
        0.164475557136178e-04,
        0.218156049790293e-04,
        -0.216901591180325e-04,
        0.218097559143513e-04,
        0.209533505034868e-04
    ],
    SujType.SUJ2: [
        0.000679361324099e-04,
        0.165601641304823e-04,
        0.219571142129651e-04,
        -0.209920380657358e-04,
        0.221362481103644e-04,
        0.218322230350838e-04
    ]
}

reading_offset_dict = {
    SujType.ECM: [
        -0.000324192907571e+02,
        -1.371779503373699e+02,
        -1.807891704964407e+02,
        1.783010819295235e+02,
        0e+02,
        0e+02
    ],
    SujType.SUJ1: [
        -0.000523031757488e+02,
        -1.371779503373699e+02,
        -1.799627805422682e+02,
        1.815830397319179e+02,
        -1.825077641185728e+02,
        -1.743117399183474e+02,
    ],
    SujType.SUJ2: [
        -0.000523031757488e+02,
        -1.378368230437714e+02,
        -1.881354442466350e+02,
        1.744835508514070e+02,
        -1.833311941228270e+02,
        -1.816169750077472e+02
    ]
}

# global brake release dict
is_brake_release_dict = {}
for suj in SujType:
    is_brake_release_dict[suj] = [False] * 6

# TODO: changing to use python mutex instead
Read_Lock = False
Write_Lock = False

def get_suj_joint_reading(serial_port):
    #global Read_Lock
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Write_Lock):
        print("get_suj_joint_reading writelock blocking")
        pass

    rospy.logdebug("get_suj_joint_reading Read_Lock accquired")
    Read_Lock = True

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
        #rospy.loginfo('Port Address Obtained Successfully')
        pass
    else:
        serial_port.reset_input_buffer()
        rospy.logerr('Failed to Get Port Address')

    if address_num == SujType.SUJ1.value:
        suj_type = SujType.SUJ1
    elif address_num == SujType.SUJ2.value:
        suj_type = SujType.SUJ2
    elif address_num == SujType.ECM.value:
        suj_type = SujType.ECM
    else:
        rospy.logerr('Failed to Get Port Address')

    serial_port.write(b"AT+READALL\r\n")
    for i in range(13):
        readings.append(serial_port.read_until())
        if i == 12:
            if readings[i][-4:] == b"OK\r\n":
                #rospy.loginfo('Reading Success.')
                pass
            else:
                serial_port.reset_input_buffer()
                rospy.logerr("Reading Error.")

    # rospy.loginfo(readings)
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
    #rospy.loginfo(suj_type + ' reading:')
    # rospy.loginfo(readings)

    # reset the Read Lock
    rospy.logdebug("get_suj_joint_reading Read_Lock released")
    Read_Lock = False

    return voltages, reading_list, valid_reading, suj_type

def dReading2degree(suj_type, d_reading):
    # retrive the global list
    ratio_list = reading_ratios_dict[suj_type]
    bias_list = reading_offset_dict[suj_type]
    pot_condition = pot_condition_dict[suj_type]

    # init the output list
    joint_pos_deg = [0 for i in range(6)]
    joint_pos_read = [0 for i in range(6)]

    for joint_ in range(6):
        if (pot_condition[joint_]+pot_condition[joint_+6]) == 2:
            joint_pos_read[joint_] += (d_reading[joint_] * pot_condition[joint_] +
                                       d_reading[joint_+6] * pot_condition[joint_+6]) / 2

        elif (pot_condition[joint_]+pot_condition[joint_+6]) == 1:
            joint_pos_read[joint_] += (d_reading[joint_] * pot_condition[joint_] +
                                       d_reading[joint_+6] * pot_condition[joint_+6])
        else:
            raise Exception('POT_condition should either 1 or 2, now is {}'.format(
                pot_condition[joint_]+pot_condition[joint_+6]))

        joint_pos_deg[joint_] = joint_pos_read[joint_] * \
            ratio_list[joint_] + bias_list[joint_]

        joint_pos_deg[joint_] = joint_pos_deg[joint_] * pi / 180

    return joint_pos_read, joint_pos_deg


def release_brakes(ser):
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Read_Lock):
        print("release_brakes readlock blocking")
        pass
    Write_Lock = True
    ser.write(b"AT+FREEALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        rospy.loginfo("All brakes are released successfully.")
    else:
        ser.reset_input_buffer()
        rospy.logerr("All Brakes Release Failed.")
    Write_Lock = False

def release_brakes_single(joint_num, ser):
    global Write_Lock
    global Read_Lock
    
    rospy.logdebug("release_brakes_single Joint no.: %d" % joint_num)
    if joint_num not in [1, 2, 3, 4, 5, 6]:
        rospy.logerr("Error: joint index out of range [1, 2, 3, 4, 5, 6].")
        return

    # block the code when Read_Lock is true
    while(Read_Lock):
        rospy.loginfo("release_brakes_single readlock blocking")
        pass
    Write_Lock = True

    joint_index = str(joint_num).encode()
    ser.write(b"AT+FREE=" + joint_index + b"\r\n")
    respond = ser.read_until()

    if respond == b"OK\r\n":
        rospy.loginfo("Succeed to release joint brake: %d" % joint_num)
    else:
        ser.reset_input_buffer()
        rospy.loginfo("Fail to release joint brake: %d" % joint_num)

    Write_Lock = False

def lock_brakes(ser):
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Read_Lock):
        print("lock_brakes Read_Lock blocking")
        pass
    Write_Lock = True
    ser.write(b"AT+LOCKALL\r\n")
    respond = ser.read_until()
    if respond == b"OK\r\n":
        rospy.loginfo("All brakes are locked successfully.")
    else:
        ser.reset_input_buffer()
        rospy.logerr("All Brakes Lock Failed.")

    Write_Lock = False

def control_brakes(is_release_brake_list, ser):
    rospy.logdebug("control_brakes")
    pprint(is_release_brake_list)

    # validate the input
    if len(is_release_brake_list) != 6:
        rospy.logerr('length  of control brake list should be 6')
        return 

    # lock all the brake first then release the brake respectively
    lock_brakes(ser)

    for i in range(6):
        if is_release_brake_list[i]:
            release_brakes_single(i+1, ser)

def readSerial(ser):
    isValid = False
    while not(isValid):
        # v_reading1~3 are the voltage readings
        # d_reading1~3 are the original digital readings
        [v_reading, d_reading, isValid, suj_type] = get_suj_joint_reading(ser)

    joint_pos_read, joint_pos_deg = dReading2degree(suj_type, d_reading)
    return joint_pos_read, joint_pos_deg, suj_type


def readAll():
    global serial_devices_dict

    joint_pos_read_dict = {}
    joint_pos_deg_dict = {}

    for ser in serial_devices_list:
        joint_pos_read, joint_pos_deg, suj_type = readSerial(ser)

        # init the dict if not yet regiested this serial port
        if suj_type not in serial_devices_dict:
            serial_devices_dict[suj_type] = ser

        joint_pos_read_dict[suj_type] = joint_pos_read
        joint_pos_deg_dict[suj_type] = joint_pos_deg

    return joint_pos_read_dict, joint_pos_deg_dict


def publish_joint_states(joint_states_list, pub):
    msg = JointState
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    msg.position = joint_states_list
    pub.publish(msg)

def _control_brakes_callback(suj_type, cmd_list):
    global is_brake_release_dict
    rospy.logdebug("_control_brakes_callback suj_type: %d" % suj_type.value)

    # validate the serial devices dict is initialized
    if suj_type not in serial_devices_dict:
        rospy.logerr("Error: serial devices dict is not initialized or the suj type not found in the serial devices dict")
        return

    count = 0

    if len(cmd_list) == 6:
        # check if the current brake release list is the same as bool list
        for i in range(6):
            if(cmd_list[i] != is_brake_release_dict[suj_type][i]):
                count = count + 1

        # debug
        pprint("cmd_list {}".format(cmd_list))
        pprint("is_brake_release_dict[suj_type] {}".format(is_brake_release_dict[suj_type]))
        # pprint("count {}".format(count))

        # if action control is required
        if count != 0:
            is_brake_release_dict[suj_type] = cmd_list
            control_brakes(
                is_brake_release_dict[suj_type], serial_devices_dict[suj_type])
    else:
        rospy.logerr(
            "The brake_list command should contains 6 Boolean arguments")

def control_brakes_SUJ1_cb(msg):
    _control_brakes_callback(SujType.SUJ1, msg.isBrakeList)

def control_brakes_SUJ2_cb(msg):
    _control_brakes_callback(SujType.SUJ2, msg.isBrakeList)

def control_brakes_ECM_cb(msg):
    _control_brakes_callback(SujType.ECM, msg.isBrakeList)

if __name__ == '__main__':
    rospy.loginfo('Initiate serial reading objects')
    joint_pos_read_dict, joint_pos_deg_dict = readAll()

    rospy.init_node('dvrk_suj_publisher', log_level=rospy.DEBUG,
                    anonymous=True, disable_signals=True)

    PSM1_suj_pub = rospy.Publisher(
        '/dvrk/SUJ/PSM1/set_position_joint', JointState, queue_size=10)
    PSM2_suj_pub = rospy.Publisher(
        '/dvrk/SUJ/PSM2/set_position_joint', JointState, queue_size=10)
    ECM_suj_pub = rospy.Publisher(
        '/dvrk/SUJ/ECM/set_position_joint', JointState, queue_size=10)

    rospy.Subscriber("/dvrk/SUJ/PSM1/brake_control",
                     Bool_List, control_brakes_SUJ1_cb)
    rospy.Subscriber("/dvrk/SUJ/PSM2/brake_control",
                     Bool_List, control_brakes_SUJ2_cb)
    rospy.Subscriber("/dvrk/SUJ/ECM/brake_control",
                     Bool_List, control_brakes_ECM_cb)

    pub_dict = {}  # ros publisher objects
    pub_dict[SujType.SUJ1] = PSM1_suj_pub
    pub_dict[SujType.SUJ2] = PSM2_suj_pub
    pub_dict[SujType.ECM] = ECM_suj_pub

    rate = rospy.Rate(10)
    is_success_print = True
    while not rospy.is_shutdown():
        try:
            # joint_pos_read_dict, joint_pos_deg_dict = readAll()
            # publish_joint_states(joint_pos_deg_dict[SujType.SUJ1], pub_dict[SujType.SUJ1])
            # publish_joint_states(joint_pos_deg_dict[SujType.SUJ2], pub_dict[SujType.SUJ2])
            # publish_joint_states(joint_pos_deg_dict[SujType.ECM], pub_dict[SujType.ECM])

            rate.sleep()
            if is_success_print:
                rospy.loginfo("dvrk_suj_publisher is running................")
                rospy.loginfo("close the program by ctrl+c")
                is_success_print = False

        except KeyboardInterrupt:
            break

    rospy.loginfo('Auto Lock All Joints ......')
    # global Read_Lock
    # global Write_Lock
    Read_Lock = False
    Write_Lock = False

    for ser in serial_devices_list:
        lock_brakes(ser)
        ser.close()
    
    rospy.loginfo('Lock Successfully')
    rospy.loginfo('exit program')