#!/usr/bin/env python
import rospy
from suj_ecm_pos.msg import suj_pos   # pay attention to the moduel name package.msg
import get_suj_setup_joint_pos

def pos_publisher():
    pub = rospy.Publisher('suj_ecm_joint_pos', suj_pos)
    rospy.init_node('suj_ecm_joint_pos_pub', anonymous=True)
    r = rospy.Rate(10)
    msg = suj_pos()
    msg.name = 'suj1'
    msg.joint_pos = get_suj_setup_joint_pos.get_data()
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        pos_publisher()
    except rospy.ROSInterruptException:
        pass
