#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('/cmd_joints', JointState, queue_size=1)
    rospy.init_node('qbo_stare')
    r = rospy.Rate(10) # 10hz
    msg = JointState()

    # Try to get pramase - use defult otherwise
    pan = rospy.get_param( '~pan', 0.0)
    tilt = rospy.get_param( '~tilt', -0.1)

    msg.name = ['head_pan_joint', 'head_tilt_joint']
    msg.position = [pan, tilt]
    msg.velocity = [0.5, 0.5]
    print 'Publishing '+str(pan)+' '+str(tilt)+' to head-pan and head-tilt'
#    count = 0
    while not rospy.is_shutdown():# and count < 600:
#        count = count + 1
        pub.publish(msg)
        r.sleep()
#    print 'Done'


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
