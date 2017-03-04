#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import threading
import time
import signal
import rospy
import tf
import math
from std_msgs.msg import *
from geometry_msgs.msg import *

pub_val_com = PoseStamped()
pub_val_rf = PoseStamped()
pub_val_lf = PoseStamped()
pub_val_rh = PoseStamped()
pub_val_lh = PoseStamped()
pub_val_rfw = WrenchStamped()
pub_val_lfw = WrenchStamped()
pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw]

key_list = ["com", "rf", "lf", "rh", "lh", "rfw", "lfw"]
label_list = ["X","Y","Z","R","P","Y"]
topic_d = {"com":PoseStamped, "rf":PoseStamped, "lf":PoseStamped, "rh":PoseStamped, "lh":PoseStamped, "rfw":WrenchStamped, "lfw":WrenchStamped}


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PoseStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PoseStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PoseStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PoseStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PoseStamped, queue_size=10)
  pub_rfw = rospy.Publisher('/human_tracker_rfw_ref', WrenchStamped, queue_size=10)
  pub_lfw = rospy.Publisher('/human_tracker_lfw_ref', WrenchStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh,pub_rfw,pub_lfw]
  
  rospy.init_node('humansync_test_publisher', anonymous=True)
  r = rospy.Rate(100)
  
  
  pub_val_com = PoseStamped()
  pub_val_rf = PoseStamped()
  pub_val_lf = PoseStamped()
  pub_val_rh = PoseStamped()
  pub_val_lh = PoseStamped()
  pub_val_rfw = WrenchStamped()
  pub_val_lfw = WrenchStamped()
  pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw]
  
  print "start pub coil COM trajectory"
  loop = 0
  HZ = 0;
  while not rospy.is_shutdown():
    sec = loop/100.0
    rad = 0.1
    HZ = 0.01*sec
    for i in range(len(pub_list)):
      pub_val_list[0].pose.position.y = rad * math.sin(2*math.pi*HZ*sec)
      pub_list[i].publish(pub_val_list[i])
    r.sleep()
    loop += 1
