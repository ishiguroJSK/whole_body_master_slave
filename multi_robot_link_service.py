#!/usr/bin/env python
# -*- coding: utf-8 -*-
from multiprocessing import Value, Array, Process
import time

import os
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from hrpsys_ros_bridge.srv import *
tablis_jnum = 27
jaxon_jnum = 33

def process1(ja):
    os.environ['ROS_MASTER_URI'] = 'http://jaxonred:11311'
    pub = rospy.Publisher('jaxon_service_caller', String, queue_size=10)
    rospy.init_node('jaxon_service_caller', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        svc = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/setJointAnglesWithMask', OpenHRP_SequencePlayerService_setJointAnglesWithMask)

        if ja != [0]*tablis_jnum :
            jaxon_ja = [0]*jaxon_jnum
            jaxon_ja[0:12] = ja[0:12]
            jaxon_ja[14] = ja[12]
            jaxon_ja[18:23] = ja[13:18]
            jaxon_ja[23:25] = [ja[19],ja[18]]
            jaxon_ja[26:31] = ja[20:25]
            jaxon_ja[31:33] = [ja[26],ja[25]]
            if jaxon_ja[0] < np.deg2rad(-30) : jaxon_ja[0] = np.deg2rad(-30)
            if jaxon_ja[6] > np.deg2rad(30)  : jaxon_ja[6] = np.deg2rad(30)
            if jaxon_ja[1] > np.deg2rad(-15) : jaxon_ja[1] = np.deg2rad(-15)
            if jaxon_ja[7] < np.deg2rad(15)  : jaxon_ja[7] = np.deg2rad(15)
            if jaxon_ja[19] > np.deg2rad(-45) : jaxon_ja[19] = np.deg2rad(-45)
            if jaxon_ja[27] < np.deg2rad(45)  : jaxon_ja[27] = np.deg2rad(45)
            print jaxon_ja
            ret = svc(jaxon_ja, [1]*len(jaxon_ja), 0.5)
            
        r.sleep()

def process2(ja):
    os.environ['ROS_MASTER_URI'] = 'http://tablis:11311'
    rospy.init_node('tablis_joint_state_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()

def callback(data):
    global ja
    ja[:] = data.position

if __name__ == '__main__':
    ja = Array('d', tablis_jnum)
    ja 

    process1 = Process(target=process1, args=[ja])
    process2 = Process(target=process2, args=[ja])

    process1.start()
    process2.start()

    process1.join()
    process2.join()

    print("process ended")
