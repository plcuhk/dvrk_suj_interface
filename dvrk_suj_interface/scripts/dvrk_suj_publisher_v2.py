#!/usr/bin/env python
from pickle import TRUE
from re import S, T
from typing import final
from rospy.impl.rosout import RosOutHandler
import serial
import time
import numpy as np
import copy
import rospy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header
from dvrk_suj_interface.msg import Bool_List

from enum import Enum
from pprint import pprint

# multi processing support
from threading import Lock
import concurrent.futures

# enable this if want to debug mutex message
PRINT_MUTEX_DEBUG_MSG = True
mutex_timeout = 2  # 1s timeout for mutex accquiring
USING_THREADED_SERIAL = True


class SujType(Enum):
    SUJ2 = 1
    ECM = 2
    SUJ1 = 3


full_range = int('FFFFFF', 16)
pi = np.pi

# Serial Devices class that comes with a mutex lock


class SerialDevice(serial.Serial):
    def __init__(self, *args, **kwargs):
        super(SerialDevice, self).__init__(*args, **kwargs)
        self.lock = Lock()


serial_devices_list = [
    SerialDevice('/dev/ttyUSB0', baudrate=115200, timeout=0.2),
    SerialDevice('/dev/ttyUSB1', baudrate=115200, timeout=0.2),
    SerialDevice('/dev/ttyUSB2', baudrate=115200, timeout=0.2)
]

# initialize the dict of serial device in runtime
serial_devices_dict = {}

# global brake release dict
is_brake_release_dict = {}
for suj in SujType:
    is_brake_release_dict[suj] = [False] * 6

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


def get_suj_joint_reading(ser):

    # initialize the return values
    readings = []
    POT_sum = 0
    valid_reading = True
    # voltages = np.zeros((12, 1)).tolist()
    voltages = [0 for i in range(12)]
    reading_list = [0 for i in range(12)]
    suj_type = None

    # accquire the write lock
    if not ser.lock.acquire(timeout=mutex_timeout):
        rospy.logerr("get_suj_joint_reading serial mutex accquiring timeout")
        return voltages, reading_list, valid_reading, suj_type

    try:
        ser.write(b"AT+ADDRESS\r\n")
        serial_port_address = ser.read_until().decode('utf-8')
        address_num = int(serial_port_address[0], 16)
        isGetAddressScuess = ser.read_until()
        if isGetAddressScuess == b"OK\r\n":
            #rospy.loginfo('Port Address Obtained Successfully')
            pass
        else:
            ser.reset_input_buffer()
            rospy.logerr('Failed to Get Port Address')

        if address_num == SujType.SUJ1.value:
            suj_type = SujType.SUJ1
        elif address_num == SujType.SUJ2.value:
            suj_type = SujType.SUJ2
        elif address_num == SujType.ECM.value:
            suj_type = SujType.ECM
        else:
            rospy.logerr('Failed to Get Port Address')

        ser.write(b"AT+READALL\r\n")
        for i in range(13):
            readings.append(ser.read_until())
            if i == 12:
                if readings[i][-4:] == b"OK\r\n":
                    #rospy.loginfo('Reading Success.')
                    pass
                else:
                    ser.reset_input_buffer()
                    rospy.logerr("Reading Error.")
                    return [0, 0, False, 0]

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

    finally:
        ser.lock.release()
        return voltages, reading_list, valid_reading, suj_type


def dReading2degree(suj_type, d_reading):
    # init the output list
    joint_pos_deg = [0 for i in range(6)]
    joint_pos_read = [0 for i in range(6)]

    if suj_type == None:
        rospy.logerr('Suj Type is None')
        return joint_pos_deg, joint_pos_deg

    # retrive the global list
    ratio_list = reading_ratios_dict[suj_type]
    bias_list = reading_offset_dict[suj_type]
    pot_condition = pot_condition_dict[suj_type]

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
    # accquire the serial lock
    if not ser.lock.acquire(timeout=mutex_timeout):
        rospy.logerr("release_brakes serial mutex accquiring timeout")
        return False

    try:
        ret = False
        ser.write(b"AT+FREEALL\r\n")
        respond = ser.read_until()
        if respond == b"OK\r\n":
            rospy.loginfo("All brakes are released successfully.")
            ret = True
        else:
            ser.reset_input_buffer()
            rospy.logerr("All Brakes Release Failed.")
    finally:
        ser.lock.release()
        return ret


def release_brakes_single(joint_num, ser):
    rospy.logdebug("release_brakes_single Joint no.: %d" % joint_num)

    # validate joint number
    if joint_num not in [1, 2, 3, 4, 5, 6]:
        rospy.logerr("Error: joint index out of range [1, 2, 3, 4, 5, 6].")
        return False

    # accquire the serial lock
    if not ser.lock.acquire(timeout=mutex_timeout):
        rospy.logerr("release_brakes serial mutex accquiring timeout")
        return False

    try:
        ret = False
        ser.write(b"AT+FREE=" + str(joint_num).encode() + b"\r\n")
        respond = ser.read_until()

        if respond == b"OK\r\n":
            rospy.loginfo("Succeed to release joint brake: %d" % joint_num)
            ret = True
        else:
            ser.reset_input_buffer()
            rospy.loginfo("Fail to release joint brake: %d" % joint_num)
    finally:
        ser.lock.release()
        return ret


def lock_brakes(ser):
    # accquire the serial lock
    if not ser.lock.acquire(timeout=mutex_timeout):
        rospy.logerr("release_brakes serial mutex accquiring timeout")
        return False
    try:
        ret = False
        ser.write(b"AT+LOCKALL\r\n")
        respond = ser.read_until()

        if respond == b"OK\r\n":
            rospy.loginfo("All brakes are locked successfully.")
            ret = True
        else:
            ser.reset_input_buffer()
            rospy.logerr("All Brakes Lock Failed.")
    finally:
        ser.lock.release()
        return ret


def control_brakes(is_release_brake_list, ser):
    rospy.logdebug("control_brakes")
    pprint(is_release_brake_list)

    # validate the input
    if len(is_release_brake_list) != 6:
        rospy.logerr('length of control brake list should be 6')
        return False

    ret = True
    # lock all the brake first then release the brake respectively
    ret = ret and lock_brakes(ser)

    for i in range(6):
        if is_release_brake_list[i]:
            ret = ret and release_brakes_single(i+1, ser)
    return ret


def readSerial(ser):
    isValid = False
    suj_type = None
    while not(isValid):
        # v_reading1~3 are the voltage readings
        # d_reading1~3 are the original digital readings
        [v_reading, d_reading, isValid, suj_type] = get_suj_joint_reading(ser)

    joint_pos_read, joint_pos_deg = [], []

    if isValid:
        joint_pos_read, joint_pos_deg = dReading2degree(suj_type, d_reading)
    return joint_pos_read, joint_pos_deg, suj_type


def readAll():
    global serial_devices_dict

    joint_pos_read_dict = {}
    joint_pos_deg_dict = {}

    if USING_THREADED_SERIAL:
        # concurrent version
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            future_list = []
            for ser in serial_devices_list:
                future_list.append(executor.submit(readSerial, ser))

            # get back the value
            for ser, fut in zip(serial_devices_list, future_list):
                joint_pos_read, joint_pos_deg, suj_type = fut.result()

                # init the dict if not yet regiested this serial port
                if suj_type not in serial_devices_dict:
                    serial_devices_dict[suj_type] = ser

                joint_pos_read_dict[suj_type] = joint_pos_read
                joint_pos_deg_dict[suj_type] = joint_pos_deg
    else:
        # sequenctial execution of all the serial ports
        for ser in serial_devices_list:
            joint_pos_read, joint_pos_deg, suj_type = readSerial(ser)

            # init the dict if not yet regiested this serial port
            if suj_type not in serial_devices_dict:
                serial_devices_dict[suj_type] = ser

            joint_pos_read_dict[suj_type] = joint_pos_read
            joint_pos_deg_dict[suj_type] = joint_pos_deg

    return joint_pos_read_dict, joint_pos_deg_dict


def publish_joint_states(joint_states_list, pub):
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    msg.position = joint_states_list
    pub.publish(msg)


def control_brakes_list(suj_type, cmd_list):
    global is_brake_release_dict
    rospy.logdebug("control_brakes suj_type: {} {}".format(
        suj_type.value, cmd_list))

    # validate the serial devices dict is initialized
    if suj_type not in serial_devices_dict:
        rospy.logerr(
            "Error: serial devices dict is not initialized or the suj type not found in the serial devices dict")
        return False

    # validate the input
    if len(cmd_list) != 6:
        rospy.logerr(
            "The brake_list command should contains 6 Boolean arguments")
        return False

    # debug
    # pprint("cmd_list {}".format(cmd_list))
    # pprint("is_brake_release_dict[suj_type] {}".format(
    #     is_brake_release_dict[suj_type]))

    # if action control is required
    if cmd_list != is_brake_release_dict[suj_type]:
        # update the brake release status
        is_brake_release_dict[suj_type] = cmd_list
        ret = control_brakes(
            is_brake_release_dict[suj_type], serial_devices_dict[suj_type])
        # TODO: handle cases when not all the brakes are released

        return ret
    else:
        # no action required as the current status is the same as the input
        return True


def control_brakes_callback(msg, suj_type):
    control_brakes_list(suj_type, msg.isBrakeList)


def suj_clutch_buttons_callback(msg, suj_type):
    rospy.logdebug("suj_clutch_buttons_callback suj_type: %d" % suj_type.value)

    if msg.buttons[0] == 1:
        control_brakes_list(suj_type, [True, True, True, True, True, True])
    elif msg.buttons[0] == 0:
        control_brakes_list(
            suj_type, [False, False, False, False, False, False])
    else:
        rospy.logerr("unable to extract clutch buttons information")


if __name__ == '__main__':
    rospy.loginfo('Initiate serial reading objects')
    joint_pos_read_dict, joint_pos_deg_dict = readAll()

    # debug
    pprint(joint_pos_read_dict)
    pprint(joint_pos_deg_dict)
    pprint(serial_devices_dict)

    rospy.init_node('dvrk_suj_publisher', log_level=rospy.DEBUG,
                    anonymous=True, disable_signals=True)

    # setup brake_control callback
    rospy.Subscriber("/SUJ/PSM1/brake_control",
                     Bool_List, control_brakes_callback, callback_args=SujType.SUJ1)
    rospy.Subscriber("/SUJ/PSM2/brake_control",
                     Bool_List, control_brakes_callback, callback_args=SujType.SUJ2)
    rospy.Subscriber("/SUJ/ECM/brake_control",
                     Bool_List, control_brakes_callback, callback_args=SujType.ECM)

    # setup suj_clutch_buttons callback
    rospy.Subscriber("/PSM1/io/suj_clutch",
                     Joy, suj_clutch_buttons_callback, callback_args=SujType.SUJ1)
    rospy.Subscriber("/PSM2/io/suj_clutch",
                     Joy, suj_clutch_buttons_callback, callback_args=SujType.SUJ2)
    rospy.Subscriber("/ECM/io/suj_clutch",
                     Joy, suj_clutch_buttons_callback, callback_args=SujType.ECM)

    # ros publisher objects
    pub_dict = {
        SujType.SUJ1: rospy.Publisher(
            '/SUJ/PSM1/measured_js', JointState,  queue_size=10),
        SujType.SUJ2: rospy.Publisher(
            '/SUJ/PSM2/measured_js', JointState, queue_size=10),
        SujType.ECM: rospy.Publisher(
            '/SUJ/ECM/measured_js', JointState, queue_size=10),
    }

    rate = rospy.Rate(100)
    is_success_print = True
    interrupted = False

    while not rospy.is_shutdown() and not interrupted:
        try:
            joint_pos_read_dict, joint_pos_deg_dict = readAll()
            publish_joint_states(
                joint_pos_deg_dict[SujType.SUJ1], pub_dict[SujType.SUJ1])
            publish_joint_states(
                joint_pos_deg_dict[SujType.SUJ2], pub_dict[SujType.SUJ2])
            publish_joint_states(
                joint_pos_deg_dict[SujType.ECM], pub_dict[SujType.ECM])

            if is_success_print:
                rospy.loginfo("dvrk_suj_publisher is running................")
                rospy.loginfo("close the program by ctrl+c")
                is_success_print = False

            rate.sleep()

        except KeyboardInterrupt:
            # TODO never catch the KeyboardInterrupt
            rospy.loginfo("Got KeyboardInterrupt")
            interrupted = True

        except KeyError:
            rospy.logerr("Got KeyError")
            interrupted = True

    rospy.loginfo('Auto Lock All Joints ......')

    for ser in serial_devices_list:
        if ser.lock.locked():
            ser.lock.release()
        lock_brakes(ser)
        ser.close()

    rospy.loginfo('Lock Successfully')
    rospy.loginfo('exit program')
