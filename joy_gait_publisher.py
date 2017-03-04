#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import signal
import numpy as np
import math
import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Joy

pub_val_com = PoseStamped()
pub_val_rf = PoseStamped()
pub_val_lf = PoseStamped()
pub_val_rh = PoseStamped()
pub_val_lh = PoseStamped()
pub_val_rfw = WrenchStamped()
pub_val_lfw = WrenchStamped()
pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw]


key_list = ["com", "rf", "lf", "rh", "lh", "rfw", "lfw"]
topic_d = {"com":PoseStamped, "rf":PoseStamped, "lf":PoseStamped, "rh":PoseStamped, "lh":PoseStamped, "rfw":WrenchStamped, "lfw":WrenchStamped}

HZ = 100.0

rf_vel = [0.0, 0.0]
lf_vel = [0.0, 0.0]
rf_rot_vel = 0.0
lf_rot_vel = 0.0
cur_rf_rot = 0.0
cur_lf_rot = 0.0
commode = "IDLE"

def callback(data):
  global rf_vel, lf_vel, commode, rf_rot_vel, lf_rot_vel
  f_vel = 0.4
  f_rot_vel = 10*math.pi/180
  lf_vel[0] = data.axes[1] * f_vel
  lf_vel[1] = data.axes[0] * f_vel
  rf_vel[0] = data.axes[5] * f_vel
  rf_vel[1] = data.axes[2] * f_vel
  rf_rot_vel = data.buttons[0] * f_rot_vel + data.buttons[2] * (-f_rot_vel)
  lf_rot_vel = data.axes[6] * f_rot_vel
  
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
  global pub_val_list, pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_rfw, pub_val_lfw, pub_list, cur_rf_rot, cur_lf_rot
  if commode == "CENTER":
    pub_val_lfw.wrench.force.z = pub_val_rfw.wrench.force.z = 600
    pub_val_com.pose.position.x = ( pub_val_rf.pose.position.x + pub_val_lf.pose.position.x ) / 2
    pub_val_com.pose.position.y = ( pub_val_rf.pose.position.y + pub_val_lf.pose.position.y ) / 2
    quaternion = tf.transformations.quaternion_from_euler(0,0, (cur_rf_rot+cur_rf_rot)/2)
    pub_val_com.pose.orientation.x = quaternion[0]
    pub_val_com.pose.orientation.y = quaternion[1]
    pub_val_com.pose.orientation.z = quaternion[2]
    pub_val_com.pose.orientation.w = quaternion[3]
  elif commode == "RIGHT":
    pub_val_lf.pose.position.x += lf_vel[0] / HZ
    pub_val_lf.pose.position.y += lf_vel[1] / HZ
    cur_lf_rot += lf_rot_vel /HZ
    quaternion = tf.transformations.quaternion_from_euler(0,0, cur_lf_rot)
    pub_val_lf.pose.orientation.x = quaternion[0]
    pub_val_lf.pose.orientation.y = quaternion[1]
    pub_val_lf.pose.orientation.z = quaternion[2]
    pub_val_lf.pose.orientation.w = quaternion[3]
    quaternion = tf.transformations.quaternion_from_euler(0,0, cur_rf_rot)
    pub_val_com.pose.orientation.x = quaternion[0]
    pub_val_com.pose.orientation.y = quaternion[1]
    pub_val_com.pose.orientation.z = quaternion[2]
    pub_val_com.pose.orientation.w = quaternion[3]
    
    r2l_vec = np.array([pub_val_lf.pose.position.x - pub_val_rf.pose.position.x, pub_val_lf.pose.position.y - pub_val_rf.pose.position.y])
    if np.linalg.norm(r2l_vec) > 0.3:
      pub_val_lf.pose.position.x = 0.3 * r2l_vec[0]/np.linalg.norm(r2l_vec) + pub_val_rf.pose.position.x
      pub_val_lf.pose.position.y = 0.3 * r2l_vec[1]/np.linalg.norm(r2l_vec) + pub_val_rf.pose.position.y
    if pub_val_lf.pose.position.y < pub_val_rf.pose.position.y + 0.16:
      pub_val_lf.pose.position.y = pub_val_rf.pose.position.y + 0.16
    pub_val_lfw.wrench.force.z = 0
    pub_val_rfw.wrench.force.z = 600
    pub_val_lf.pose.position.z = 0.03
    pub_val_com.pose.position.x = pub_val_rf.pose.position.x
    pub_val_com.pose.position.y = pub_val_rf.pose.position.y
    
  elif commode == "LEFT":
    pub_val_rf.pose.position.x += rf_vel[0] / HZ
    pub_val_rf.pose.position.y += rf_vel[1] / HZ
    cur_rf_rot += rf_rot_vel /HZ
    quaternion = tf.transformations.quaternion_from_euler(0,0, cur_rf_rot)
    pub_val_rf.pose.orientation.x = quaternion[0]
    pub_val_rf.pose.orientation.y = quaternion[1]
    pub_val_rf.pose.orientation.z = quaternion[2]
    pub_val_rf.pose.orientation.w = quaternion[3]
    quaternion = tf.transformations.quaternion_from_euler(0,0, cur_lf_rot)
    pub_val_com.pose.orientation.x = quaternion[0]
    pub_val_com.pose.orientation.y = quaternion[1]
    pub_val_com.pose.orientation.z = quaternion[2]
    pub_val_com.pose.orientation.w = quaternion[3]
    
    l2r_vec = np.array([pub_val_rf.pose.position.x - pub_val_lf.pose.position.x, pub_val_rf.pose.position.y - pub_val_lf.pose.position.y])
    if np.linalg.norm(l2r_vec) > 0.3:
      pub_val_rf.pose.position.x = 0.3 * l2r_vec[0]/np.linalg.norm(l2r_vec) + pub_val_lf.pose.position.x
      pub_val_rf.pose.position.y = 0.3 * l2r_vec[1]/np.linalg.norm(l2r_vec) + pub_val_lf.pose.position.y
    if pub_val_rf.pose.position.y > pub_val_lf.pose.position.y - 0.16:
      pub_val_rf.pose.position.y = pub_val_lf.pose.position.y - 0.16
    pub_val_lfw.wrench.force.z = 600
    pub_val_rfw.wrench.force.z = 0
    pub_val_rf.pose.position.z = 0.03
    pub_val_com.pose.position.x = pub_val_lf.pose.position.x
    pub_val_com.pose.position.y = pub_val_lf.pose.position.y
    
  else:
    pub_val_lfw.wrench.force.z = pub_val_rfw.wrench.force.z = 0
    pub_val_com.pose.position.x = ( pub_val_rf.pose.position.x + pub_val_lf.pose.position.x ) / 2
    pub_val_com.pose.position.y = ( pub_val_rf.pose.position.y + pub_val_lf.pose.position.y ) / 2
    quaternion = tf.transformations.quaternion_from_euler(0,0, (cur_rf_rot+cur_rf_rot)/2)
    pub_val_com.pose.orientation.x = quaternion[0]
    pub_val_com.pose.orientation.y = quaternion[1]
    pub_val_com.pose.orientation.z = quaternion[2]
    pub_val_com.pose.orientation.w = quaternion[3]
    
  
  pub_val_rh.pose.position.x = pub_val_com.pose.position.x
  pub_val_rh.pose.position.y = pub_val_com.pose.position.y
  pub_val_lh.pose.position.x = pub_val_com.pose.position.x
  pub_val_lh.pose.position.y = pub_val_com.pose.position.y
    
  for i in range(len(pub_list)):
    pub_list[i].publish(pub_val_list[i])


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  sub_joy = rospy.Subscriber("/joy", Joy, callback)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PoseStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PoseStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PoseStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PoseStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PoseStamped, queue_size=10)
  pub_rfw = rospy.Publisher('/human_tracker_rfw_ref', WrenchStamped, queue_size=10)
  pub_lfw = rospy.Publisher('/human_tracker_lfw_ref', WrenchStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh,pub_rfw,pub_lfw]
  
  rospy.init_node('humansync_joy_publisher', anonymous=True)
  r = rospy.Rate(HZ)
  global pub_val_rf, pub_val_lf
  pub_val_rf.pose.position.y = -0.1
  pub_val_lf.pose.position.y = 0.1
  
  print "start ROS pub loop"
  while not rospy.is_shutdown():
    pubhumanpose()
    r.sleep()
