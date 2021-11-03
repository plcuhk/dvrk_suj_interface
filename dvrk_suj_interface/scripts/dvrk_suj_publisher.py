#!/usr/bin/env python
from rospy.impl.rosout import RosOutHandler
import serial
import time
import numpy as np
import copy
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dvrk_suj_interface.msg import Bool_List

from pprint import pprint

# ser1 = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
#                     write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
# ser2 = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
#                     write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
# ser3 = serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=0.2, xonxoff=False, rtscts=False,
#                     write_timeout=0.5, dsrdtr=False, inter_byte_timeout=None, exclusive=None)
ser1 = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.2)
ser2 = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.2)
ser3 = serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=0.2)

POT_Condition_suj1 = [1, 1, 1, 1, 1, 1,
                      0, 1, 1, 1, 1, 1]
POT_Condition_suj2 = [1, 1, 1, 1, 1, 1,
                      1, 1, 1, 1, 1, 1]

POT_Condition_ecm = [1, 1, 1, 1, 1, 1,
                     0, 1, 1, 1, 1, 1]

full_range = int('FFFFFF', 16)
pi = np.pi

# digital readings to degrees/meters
reading_ratios_suj1 = [
    0.000679361324099e-04,
    0.164475557136178e-04,
    0.218156049790293e-04,
    -0.216901591180325e-04,
    0.218097559143513e-04,
    0.209533505034868e-04]
reading_ratios_suj2 = [
    0.000679361324099e-04,
    0.165601641304823e-04,
    0.219571142129651e-04,
    -0.209920380657358e-04,
    0.221362481103644e-04,
    0.218322230350838e-04]
reading_ratios_ecm = [
    0.000651841047138e-04,
    0.164475557136178e-04,
    0.212760655892334e-04,
    -0.213315053290246e-04,
    0e-04,
    0e-04]


reading_offset_suj1 = [
    -0.000523031757488e+02,
    -1.371779503373699e+02,
    -1.799627805422682e+02,
    1.815830397319179e+02,
    -1.825077641185728e+02,
    -1.743117399183474e+02,
]

reading_offset_suj2 = [
    -0.000523031757488e+02,
    -1.378368230437714e+02,
    -1.881354442466350e+02,
    1.744835508514070e+02,
    -1.833311941228270e+02,
    -1.816169750077472e+02]

reading_offset_ecm = [
    -0.000324192907571e+02,
    -1.371779503373699e+02,
    -1.807891704964407e+02,
    1.783010819295235e+02,
    0e+02,
    0e+02]


# brake control unit
is_brake_release_list_SUJ1 = [False] * 6
is_brake_release_list_SUJ2 = [False] * 6
is_brake_release_list_ECM = [False] * 6


Read_Lock = False
Write_Lock = False

# global dictionary variables
armSerialportDic = {}  # serial obj
joint_pos_read_dict = {}  # reading data
joint_pos_deg_dict = {}  # deg data
pub_dict = {}  # ros publisher obj
is_brake_release_dict = {}

is_brake_release_dict['SUJ1'] = is_brake_release_list_SUJ1
is_brake_release_dict['SUJ2'] = is_brake_release_list_SUJ2
is_brake_release_dict['ECM'] = is_brake_release_list_ECM


def get_suj_joint_reading(serial_port):
    #global Read_Lock
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Write_Lock):
        pass
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
    #rospy.loginfo(arm + ' reading:')
    # rospy.loginfo(readings)

    # reset the Read Lock
    Read_Lock = False

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
        if (POT_Condition[joint_]+POT_Condition[joint_+6]) == 2:
            joint_pos_read[joint_] += (d_reading[joint_] * POT_Condition[joint_] +
                                       d_reading[joint_+6] * POT_Condition[joint_+6]) / 2

        elif (POT_Condition[joint_]+POT_Condition[joint_+6]) == 1:
            joint_pos_read[joint_] += (d_reading[joint_] * POT_Condition[joint_] +
                                       d_reading[joint_+6] * POT_Condition[joint_+6])
        else:
            raise Exception('POT_condition should either 1 or 2, now is {}'.format(
                POT_Condition[joint_]+POT_Condition[joint_+6]))

        joint_pos_deg[joint_] = joint_pos_read[joint_] * \
            ratio_list[joint_] + bias_list[joint_]

        joint_pos_deg[joint_] = joint_pos_deg[joint_] * pi / 180

    return joint_pos_read, joint_pos_deg


def release_brakes(ser):
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Read_Lock):
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
    rospy.logdebug("release_brakes_single Joint no.: %d" % joint_num)
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Read_Lock):
        pass
    Write_Lock = True

    if joint_num not in [1, 2, 3, 4, 5, 6]:
        rospy.logerr("Error: joint index out of range [1, 2, 3, 4, 5, 6].")
        return

    joint_index = str(joint_num).encode()
    ser.write(b"AT+FREE=" + joint_index + b"\r\n")
    respond = ser.read_until()

    if respond == b"OK\r\n":
        rospy.loginfo("Succeed to release joint brake: %d" % joint_num)
    else:
        ser.reset_input_buffer()
        rospy.loginfo("Fail to release joint brake: %d" % joint_num)
    Write_Lock = False


def control_brakes(is_release_brake_list, ser):
    rospy.logdebug("control_brakes")
    pprint(is_release_brake_list)

    if len(is_release_brake_list) == 6:
        # for i in range(6):
        #     if is_release_brake_list[i]:
        #         release_brakes_single(i+1, ser)
        #     else:
        #         lock_brakes_single(i+1, ser)

        # lock the brake first then release the brake respectively
        lock_brakes(ser)

        for i in range(6):
            if is_release_brake_list[i]:
                release_brakes_single(i+1, ser)
    else:
        rospy.logerr('length  of control brake list should be 6')


def lock_brakes_single(joint_num, ser):
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Read_Lock):
        pass
    Write_Lock = True

    if joint_num not in [1, 2, 3, 4, 5, 6]:
        rospy.logerr("Error: joint index out of range [1, 2,, 3, 4, 5, 6].")
        return

    joint_index = str(joint_num).encode()
    ser.write(b"AT+LOCK=" + joint_index + b"\r\n")
    respond = ser.read_until()

    if respond == b"OK\r\n":
        rospy.loginfo("Succeed to lock joint brake: %d" % joint_num)
    else:
        ser.reset_input_buffer()
        rospy.logerr("Fail to lock joint brake: %d" % joint_num)

    Write_Lock = False


def lock_brakes(ser):
    global Write_Lock
    global Read_Lock
    # block the code when Read_Lock is true
    while(Read_Lock):
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


def readSerial(ser):
    global reading_ratios_suj1
    global reading_ratios_suj2
    global reading_ratios_ecm
    global reading_offset_suj1
    global reading_offset_suj2
    global reading_offset_ecm
    isValid = False
    while not(isValid):
        # v_reading1~3 are the voltage readings
        # d_reading1~3 are the original digital readings
        [v_reading, d_reading, isValid, arm_str] = get_suj_joint_reading(ser)

    if arm_str == 'SUJ1':
        joint_pos_read, joint_pos_deg = dReading2degree(d_reading,
                                                        arm_str,
                                                        reading_ratios_suj1,
                                                        reading_offset_suj1,
                                                        POT_Condition_suj1)
    if arm_str == 'SUJ2':
        joint_pos_read, joint_pos_deg = dReading2degree(d_reading,
                                                        arm_str,
                                                        reading_ratios_suj2,
                                                        reading_offset_suj2,
                                                        POT_Condition_suj2)
    if arm_str == 'ECM':
        joint_pos_read, joint_pos_deg = dReading2degree(d_reading,
                                                        arm_str,
                                                        reading_ratios_ecm,
                                                        reading_offset_ecm,
                                                        POT_Condition_ecm)

    return joint_pos_read, joint_pos_deg, arm_str


def readAll(ser1, ser2, ser3):

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

    #rospy.loginfo('\n\nSUJ1 --> SUJ2 --> ECM')
    #rospy.loginfo('\n digital readings SUJ1 --> SUJ2 --> ECM')
    # rospy.loginfo(joint_pos_read_dict['SUJ1'])
    # rospy.loginfo(joint_pos_read_dict['SUJ2'])
    # rospy.loginfo(joint_pos_read_dict['ECM'])
    #rospy.loginfo('\n\n degree SUJ1 --> SUJ2 --> ECM')
    # rospy.loginfo(joint_pos_deg_dict['SUJ1'])
    # rospy.loginfo(joint_pos_deg_dict['SUJ2'])
    # rospy.loginfo(joint_pos_deg_dict['ECM'])

    return armSerialportDic, joint_pos_read_dict, joint_pos_deg_dict


def publish_joint_states(joint_states_list, pub):
    msg = JointState
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    msg.position = joint_states_list
    pub.publish(msg)


def control_brakes_SUJ1_cb(msg):
    rospy.logdebug("control_brakes_SUJ1_cb")
    global is_brake_release_dict
    global armSerialportDic
    count = 0
    bool_list = msg.isBrakeList
    if len(bool_list) == 6:

        # check if the current brake release list is the same as bool list
        for i in range(6):
            if(bool_list[i] != is_brake_release_dict['SUJ1'][i]):
                count = count + 1

        pprint(bool_list)
        pprint(is_brake_release_dict)
        pprint(count)

        # if action control is required
        if count != 0:
            is_brake_release_dict['SUJ1'] = bool_list
            control_brakes(
                is_brake_release_dict['SUJ1'], armSerialportDic['SUJ1'])
    else:
        rospy.logerr(
            "The brake_list command should contains 6 Boolean arguments")


def control_brakes_SUJ2_cb(msg):
    rospy.logdebug("control_brakes_SUJ2_cb")
    global is_brake_release_list_SUJ2
    global armSerialportDic
    count = 0
    bool_list = msg.isBrakeList
    if len(bool_list) == 6:
        for i in range(6):
            if(bool_list[i] == is_brake_release_dict['SUJ2'][i]):
                count = count + 1
        if count != 0:
            is_brake_release_dict['SUJ2'] = bool_list
            control_brakes(
                is_brake_release_dict['SUJ2'], armSerialportDic['SUJ2'])
    else:
        rospy.logerr(
            "The brake_list command should contains 6 Boolean arguments")


def control_brakes_ECM_cb(msg):
    rospy.logdebug("control_brakes_ECM_cb")
    global is_brake_release_list_ECM
    global armSerialportDic
    count = 0
    bool_list = msg.isBrakeList
    if len(bool_list) == 6:
        for i in range(6):
            if(bool_list[i] == is_brake_release_dict['ECM'][i]):
                count = count + 1
        if count != 0:
            is_brake_release_dict['ECM'] = bool_list
            control_brakes(
                is_brake_release_dict['ECM'], armSerialportDic['ECM'])


if __name__ == '__main__':
    # global armSerialportDic
    # global joint_pos_read_dict
    # global joint_pos_deg_dict
    # global pub_dict
    # global ser1
    # global ser2
    # global ser3

    armSerialportDic, joint_pos_read_dict, joint_pos_deg_dict = readAll(
        ser1, ser2, ser3)
    #rospy.loginfo('Initiate serial reading objects')

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

    pub_dict['SUJ1'] = PSM1_suj_pub
    pub_dict['SUJ2'] = PSM2_suj_pub
    pub_dict['ECM'] = ECM_suj_pub

    rate = rospy.Rate(100)
    is_success_print = True
    while not rospy.is_shutdown():
        try:
            armSerialportDic, joint_pos_read_dict, joint_pos_deg_dict = readAll(
                ser1, ser2, ser3)
            publish_joint_states(joint_pos_deg_dict['SUJ1'], pub_dict['SUJ1'])
            publish_joint_states(joint_pos_deg_dict['SUJ2'], pub_dict['SUJ2'])
            publish_joint_states(joint_pos_deg_dict['ECM'], pub_dict['ECM'])
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
    lock_brakes(armSerialportDic['SUJ1'])
    lock_brakes(armSerialportDic['SUJ2'])
    lock_brakes(armSerialportDic['ECM'])
    armSerialportDic['SUJ1'].close()
    armSerialportDic['SUJ2'].close()
    armSerialportDic['ECM'].close()
    rospy.loginfo('Lock Successfully')
    rospy.loginfo('exit program')
