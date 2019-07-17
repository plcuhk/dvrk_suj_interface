#!/usr/bin/env python
import rospy
from suj_ecm_pos.msg import suj_pos   # pay attention to the moduel name package.msg

def callback(data):
    rospy.loginfo('%s joint position:' % data.name, data.joint_pos )

def pos_subscriber():
    rospy.init_node('suj_ecm_joint_pos_sub', anonymous=True)
    rospy.Subscriber('suj_ecm_joint_pos', suj_pos, callback)

    rospy.spin()
if __name__ == '__main__':
    pos_subscriber()