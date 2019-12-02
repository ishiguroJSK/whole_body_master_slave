#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import time
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped

from multiprocessing import Value, Array, Process
import os

prefix = ["lf","rf","lh","rh"]
w_dof = 6 # force3 + torque3

def process1(wl):
    os.environ['ROS_MASTER_URI'] = 'http://jaxonred:11311'
    rospy.init_node('force_sub_from_jaxon', anonymous=True)
    for i in range(len(prefix)):
        rospy.Subscriber( prefix[i]+"sensor", WrenchStamped, callback, callback_args=i)
    rospy.spin()
    
def callback(data, id):
    global wl
    wl[id*w_dof+0] = data.wrench.force.x
    wl[id*w_dof+1] = data.wrench.force.y
    wl[id*w_dof+2] = data.wrench.force.z
    wl[id*w_dof+3] = data.wrench.torque.x
    wl[id*w_dof+4] = data.wrench.torque.y
    wl[id*w_dof+5] = data.wrench.torque.z
        
def process2(wl,ww):
    os.environ['ROS_MASTER_URI'] = 'http://jaxonred:11311'
    rospy.init_node('tf_sub_from_jaxon', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(prefix)):
            try:
                (pos, rot) = listener.lookupTransform("BODY", prefix[i]+"sensor", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                infostr += prefix[i] + "\033[91m waiting... \033[0m\n"
                continue
            else:
                vl = Vector3Stamped()
                vl.header.frame_id = prefix[i]+"sensor"
                vl.vector.x = wl[i*w_dof+0]
                vl.vector.y = wl[i*w_dof+1]
                vl.vector.z = wl[i*w_dof+2]
                vw = listener.transformVector3("BODY", vl)
                ww[i*w_dof+0] = vw.vector.x
                ww[i*w_dof+1] = vw.vector.y
                ww[i*w_dof+2] = vw.vector.z
                vl.vector.x = wl[i*w_dof+3]
                vl.vector.y = wl[i*w_dof+4]
                vl.vector.z = wl[i*w_dof+5]
                vw = listener.transformVector3("BODY", vl)
                ww[i*w_dof+3] = vw.vector.x
                ww[i*w_dof+4] = vw.vector.y
                ww[i*w_dof+5] = vw.vector.z
                infostr += prefix[i] + "\033[92m OK \033[0m"+str(pos)+" "+str(rot)+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Working at "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)

def process3(ww):
    os.environ['ROS_MASTER_URI'] = 'http://tablis:11311'
    rospy.init_node('tf_sub_from_tablis', anonymous=True)
    pubs = [rospy.Publisher("/feedback/"+p+"wrench_wld",WrenchStamped, queue_size=1) for p in prefix]
    rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(pubs)):
            val = WrenchStamped()
            val.header.frame_id = "BASE_LINK"
            val.wrench.force.x  = ww[i*w_dof+0]
            val.wrench.force.y  = ww[i*w_dof+1]
            val.wrench.force.z  = ww[i*w_dof+2]
            val.wrench.torque.x = ww[i*w_dof+3]
            val.wrench.torque.y = ww[i*w_dof+4]
            val.wrench.torque.z = ww[i*w_dof+5]
            pubs[i].publish(val)
            # infostr += prefix[i] + "\033[92m OK \033[0m"+str(pos)+" "+str(rot)+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Working at "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)


        
if __name__ == '__main__':
    wl = Array('d', w_dof * len(prefix)) # local wrench
    ww = Array('d', w_dof * len(prefix)) # world wrench
    process1 = Process(target=process1, args=[wl])
    process2 = Process(target=process2, args=[wl,ww])
    process3 = Process(target=process3, args=[ww])
    process1.start()
    process2.start()
    process3.start()
    ### working ###
    process1.join()
    process2.join()
    process3.join()
    print("process ended")
