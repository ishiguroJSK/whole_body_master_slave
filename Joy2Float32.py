#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
pub = 0

def cb(msg):
  pub.publish(Float32(msg.axes[2]))

if __name__ == '__main__':
  rospy.init_node('joy2float32_node', anonymous=True)
  rospy.Subscriber("/joy_in", Joy, cb)
  pub = rospy.Publisher("/float32_out", Float32, queue_size=10)
  rospy.spin()
