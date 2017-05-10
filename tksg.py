#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import signal
import rospy
import tf
import math
from std_msgs.msg import *
from geometry_msgs.msg import *


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PoseStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PoseStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PoseStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PoseStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PoseStamped, queue_size=10)
  pub_zmp = rospy.Publisher('/human_tracker_zmp_ref', PointStamped, queue_size=10)
#   pub_rfw = rospy.Publisher('/human_tracker_rfw_ref', WrenchStamped, queue_size=10)
#   pub_lfw = rospy.Publisher('/human_tracker_lfw_ref', WrenchStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh,pub_zmp]
  
  rospy.init_node('tksg_publisher', anonymous=True)
  r = rospy.Rate(100)
  
  pub_val_com = PoseStamped()
  pub_val_rf = PoseStamped()
  pub_val_lf = PoseStamped()
  pub_val_rh = PoseStamped()
  pub_val_lh = PoseStamped()
  pub_val_zmp = PointStamped()
  pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_zmp]
  
  print "start tksg"
  loop = 0
  while not rospy.is_shutdown():
    
    pub_val_com.pose.position.x = 0;
    pub_val_com.pose.position.y = 0;
    pub_val_com.pose.position.z = 1.05;
    
    pub_val_rf.pose.position.x = 0;
    pub_val_rf.pose.position.y = -0.1;
    pub_val_rf.pose.position.z = 0;
    
    pub_val_lf.pose.position.x = 0;
    pub_val_lf.pose.position.y = 0.1;
    pub_val_lf.pose.position.z = 0;
    
    pub_val_rh.pose.position.x = 0.3;
    pub_val_rh.pose.position.y = -0.3;
    pub_val_rh.pose.position.z = 0.9;
    
    pub_val_lh.pose.position.x = 0.3;
    pub_val_lh.pose.position.y = 0.3;
    pub_val_lh.pose.position.z = 0.9;
    
    pub_val_zmp.point.x = 0.0;
    pub_val_zmp.point.y = 0.0;
    pub_val_zmp.point.z = 0.0;
    
    for i in range(len(pub_list)):
      pub_list[i].publish(pub_val_list[i])
    r.sleep()
    loop += 1
