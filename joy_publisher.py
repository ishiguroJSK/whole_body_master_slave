#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time

import signal

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Joy

pub_val_com = PointStamped()
pub_val_rf = PointStamped()
pub_val_lf = PointStamped()
pub_val_rh = PointStamped()
pub_val_lh = PointStamped()
pub_val_rfw = WrenchStamped()
pub_val_lfw = WrenchStamped()
pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw]


key_list = ["com", "rf", "lf", "rh", "lh", "rfw", "lfw"]
topic_d = {"com":PointStamped, "rf":PointStamped, "lf":PointStamped, "rh":PointStamped, "lh":PointStamped, "rfw":WrenchStamped, "lfw":WrenchStamped}

HZ = 100.0

rf_vel = [0.0, 0.0]
lf_vel = [0.0, 0.0]
commode = "IDLE"

def callback(data):
  global rf_vel, lf_vel, commode
  f_vel = 0.4
  lf_vel[0] = data.axes[1] * f_vel
  lf_vel[1] = data.axes[0] * f_vel
  rf_vel[0] = data.axes[5] * f_vel
  rf_vel[1] = data.axes[2] * f_vel
  
  if data.buttons[4] == 1.0 and data.buttons[5] == 1.0:
    commode = "CENTER"
  elif data.buttons[4] != 1.0 and data.buttons[5] == 1.0:
    commode = "RIGHT"
  elif data.buttons[4] == 1.0 and data.buttons[5] != 1.0:
    commode = "LEFT"
  else:
    commode = "IDLE"
  print "callback"  + commode
  
  
def pubhumanpose():
  global rf_vel, lf_vel, commode, pub_val_list, pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw, pub_list
  init_foot_width = 0.1
  if commode == "CENTER":
    pub_val_lfw.wrench.force.z = pub_val_rfw.wrench.force.z = 600
    pub_val_com.point.x = ( pub_val_rf.point.x + pub_val_lf.point.x ) / 2
    pub_val_com.point.y = ( pub_val_rf.point.y-init_foot_width + pub_val_lf.point.y+init_foot_width ) / 2
  elif commode == "RIGHT":
    pub_val_lf.point.x += lf_vel[0] / HZ
    pub_val_lf.point.y += lf_vel[1] / HZ
    if   pub_val_lf.point.x > pub_val_rf.point.x + 0.15: pub_val_lf.point.x = pub_val_rf.point.x + 0.15
    elif pub_val_lf.point.x < pub_val_rf.point.x - 0.10: pub_val_lf.point.x = pub_val_rf.point.x - 0.10
    if   pub_val_lf.point.y > pub_val_rf.point.y + 0.10: pub_val_lf.point.y = pub_val_rf.point.y + 0.10
    elif pub_val_lf.point.y < pub_val_rf.point.y - 0.05: pub_val_lf.point.y = pub_val_rf.point.y - 0.05
    pub_val_lfw.wrench.force.z = 0
    pub_val_rfw.wrench.force.z = 600
    pub_val_com.point.x = pub_val_rf.point.x
    pub_val_com.point.y = pub_val_rf.point.y-init_foot_width
  elif commode == "LEFT":
    pub_val_rf.point.x += rf_vel[0] / HZ
    pub_val_rf.point.y += rf_vel[1] / HZ
    if   pub_val_rf.point.x > pub_val_lf.point.x + 0.15: pub_val_rf.point.x = pub_val_lf.point.x + 0.15
    elif pub_val_rf.point.x < pub_val_lf.point.x - 0.10: pub_val_rf.point.x = pub_val_lf.point.x - 0.10
    if   pub_val_rf.point.y < pub_val_lf.point.y - 0.10: pub_val_rf.point.y = pub_val_lf.point.y - 0.10
    elif pub_val_rf.point.y > pub_val_lf.point.y + 0.05: pub_val_rf.point.y = pub_val_lf.point.y + 0.05
    pub_val_lfw.wrench.force.z = 600
    pub_val_rfw.wrench.force.z = 0
    pub_val_com.point.x = pub_val_lf.point.x
    pub_val_com.point.y = pub_val_lf.point.y+init_foot_width
  else:
    pub_val_lfw.wrench.force.z = pub_val_rfw.wrench.force.z = 0
    pub_val_com.point.x = ( pub_val_rf.point.x + pub_val_lf.point.x ) / 2
    pub_val_com.point.y = ( pub_val_rf.point.y-init_foot_width + pub_val_lf.point.y+init_foot_width ) / 2
    
  for i in range(len(pub_list)):
    pub_list[i].publish(pub_val_list[i])


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  sub_joy = rospy.Subscriber("/joy", Joy, callback)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PointStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PointStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PointStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PointStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PointStamped, queue_size=10)
  pub_rfw = rospy.Publisher('/human_tracker_rfw_ref', WrenchStamped, queue_size=10)
  pub_lfw = rospy.Publisher('/human_tracker_lfw_ref', WrenchStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh,pub_rfw,pub_lfw]
  
  rospy.init_node('humansync_joy_publisher', anonymous=True)
  r = rospy.Rate(HZ)
  
  print "start ROS pub loop"
  while not rospy.is_shutdown():
    pubhumanpose()
    r.sleep()
