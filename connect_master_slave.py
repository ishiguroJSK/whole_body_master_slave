#!/usr/bin/env python  
import math
import os
import roslib
import rospy
import tf
import time
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from multiprocessing import Value, Array, Process

p_tgts = ["com","lf","rf","lh","rh"]
l_tgts = ["BASE_LINK","LLEG_LINK5","RLEG_LINK5","LARM_LINK6","RARM_LINK6"]
w_tgts = ["lf","rf","lh","rh"]
w_dof = 6 # wrench DOF = force3 + torque3
p_dof = 7 # pose DOF = pos3 + quartanion4
master_host = 'http://tablis:11311'
master_origin = "BASE_LINK"
slave_host = 'http://jaxonred:11311'
slave_origin = "BODY"


def sub_tf_from_master(pw):
    os.environ['ROS_MASTER_URI'] = master_host
    os.environ['ROS_IP'] = "192.168.96.113"
    rospy.init_node("sub_tf_from_master", anonymous=True)
    tfl = tf.TransformListener()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(l_tgts)):
            try:
                (pos, rot) = tfl.lookupTransform(master_origin, l_tgts[i], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                infostr += l_tgts[i] + "\033[91m waiting... \033[0m\n"
                continue
            else:
                pw[i*p_dof+0 : i*p_dof+3] = pos[:]
                pw[i*p_dof+3 : i*p_dof+7] = rot[:]
                infostr += l_tgts[i] + "\033[92m OK \033[0m"+str(pos)+" "+str(rot)+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Master->(tf)-> "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)


def pub_wrench_to_master(ww):
    os.environ['ROS_MASTER_URI'] = master_host
    os.environ['ROS_IP'] = "192.168.96.113"
    rospy.init_node("pub_wrench_to_master", anonymous=True)
    pubs = [rospy.Publisher("/feedback_"+w+"sensor",WrenchStamped, queue_size=1) for w in w_tgts]
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(pubs)):
            val = WrenchStamped()
            val.header.frame_id = master_origin
            val.wrench.force.x  = ww[i*w_dof+0]
            val.wrench.force.y  = ww[i*w_dof+1]
            val.wrench.force.z  = ww[i*w_dof+2]
            val.wrench.torque.x = ww[i*w_dof+3]
            val.wrench.torque.y = ww[i*w_dof+4]
            val.wrench.torque.z = ww[i*w_dof+5]
            pubs[i].publish(val)
            infostr += w_tgts[i] + "\033[92m OK \033[0m"+str(ww[i*w_dof:i*w_dof+6])+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Master<-(wrench)<- "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)

        
def pub_pose_to_slave(pw):
    os.environ['ROS_MASTER_URI'] = slave_host
    os.environ['ROS_IP'] = "192.168.96.113"
    rospy.init_node("pub_pose_to_slave", anonymous=True)
    pubs = [rospy.Publisher("/human_tracker_"+p+"_ref", PoseStamped, queue_size=1) for p in p_tgts]
    rate = rospy.Rate(200)
    vals = [PoseStamped() for i in range(0, len(l_tgts))]
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(0,len(vals)):
            vals[i].header.frame_id = "BODY"
            vals[i].header.stamp = rospy.Time.now()
            vals[i].pose.position.x    = pw[i*p_dof+0]
            vals[i].pose.position.y    = pw[i*p_dof+1]
            vals[i].pose.position.z    = pw[i*p_dof+2]
            vals[i].pose.orientation.x = pw[i*p_dof+3]
            vals[i].pose.orientation.y = pw[i*p_dof+4]
            vals[i].pose.orientation.z = pw[i*p_dof+5]
            vals[i].pose.orientation.w = pw[i*p_dof+6]
            pubs[i].publish(vals[i])
            infostr += l_tgts[i] + "\033[92m OK \033[0m"+str(pw[i*w_dof:i*w_dof+6])+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "->(pose)->Slave "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)

def sub_wrench_from_slave(wl):
    os.environ['ROS_MASTER_URI'] = slave_host
    os.environ['ROS_IP'] = "192.168.96.113"
    rospy.init_node("sub_wrench_from_slave", anonymous=True)
    for i in range(len(w_tgts)):
        rospy.Subscriber( w_tgts[i]+"sensor", WrenchStamped, callback, callback_args=i)
    rospy.spin()
    
def callback(data, id):
    global wl
    wl[id*w_dof+0] = data.wrench.force.x
    wl[id*w_dof+1] = data.wrench.force.y
    wl[id*w_dof+2] = data.wrench.force.z
    wl[id*w_dof+3] = data.wrench.torque.x
    wl[id*w_dof+4] = data.wrench.torque.y
    wl[id*w_dof+5] = data.wrench.torque.z
    
def sub_tf_from_slave(wl,ww):
    os.environ['ROS_MASTER_URI'] = slave_host
    os.environ['ROS_IP'] = "192.168.96.113"
    rospy.init_node("sub_tf_from_slave", anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(w_tgts)):
            try:
                (pos, rot) = listener.lookupTransform(slave_origin, w_tgts[i]+"sensor", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                infostr += w_tgts[i] + "\033[91m waiting... \033[0m\n"
                continue
            else:
                vl = Vector3Stamped()
                vl.header.frame_id = w_tgts[i]+"sensor"
                vl.vector.x = wl[i*w_dof+0]
                vl.vector.y = wl[i*w_dof+1]
                vl.vector.z = wl[i*w_dof+2]
                vw = listener.transformVector3(slave_origin, vl)
                ww[i*w_dof+0] = vw.vector.x
                ww[i*w_dof+1] = vw.vector.y
                ww[i*w_dof+2] = vw.vector.z
                vl.vector.x = wl[i*w_dof+3]
                vl.vector.y = wl[i*w_dof+4]
                vl.vector.z = wl[i*w_dof+5]
                vw = listener.transformVector3(slave_origin, vl)
                ww[i*w_dof+3] = vw.vector.x
                ww[i*w_dof+4] = vw.vector.y
                ww[i*w_dof+5] = vw.vector.z
                infostr += w_tgts[i] + "\033[92m OK \033[0m"+str(pos)+" "+str(rot)+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "(tf)<-Slave "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)


if __name__ == '__main__':
    pw = Array('d', p_dof * len(l_tgts)) # world pose
    wl = Array('d', w_dof * len(w_tgts)) # local wrench
    ww = Array('d', w_dof * len(w_tgts)) # world wrench

    processes = []
    processes += [Process(target=pub_pose_to_slave,     args=[pw])]
    processes += [Process(target=sub_tf_from_master,    args=[pw])]
    processes += [Process(target=sub_wrench_from_slave, args=[wl])]
    processes += [Process(target=sub_tf_from_slave,     args=[wl,ww])]
    processes += [Process(target=pub_wrench_to_master,  args=[ww])]

    for p in processes:
        p.start()
    print("all process started")

    ### working ###

    for p in processes:
        p.join()
    print("all process finished")
