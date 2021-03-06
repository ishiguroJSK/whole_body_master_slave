#!/bin/bash

(
    echo "start tablis record"
    export ROS_MASTER_URI=http://tablis:11311
    rosbag record -o ~/ishiguro/tablis \
/act_capture_point \
/act_contact_states \
/clock \
/delay_check_packet_inbound \
/delay_check_packet_outbound \
/imu \
/joint_states \
/lfsensor \
/lfsensor_cop \
/lhsensor \
/lhsensor_cop \
/master_com_pose \
/master_delay_ans \
/master_head_pose \
/master_larm_pose \
/master_larm_wrench \
/master_lhand_pose \
/master_lleg_pose \
/master_lleg_wrench \
/master_rarm_pose \
/master_rarm_wrench \
/master_rhand_pose \
/master_rleg_pose \
/master_rleg_wrench \
/motor_states_low/abs_angle \
/motor_states_low/abs_cur_angle_diff \
/motor_states_low/board_vdd \
/motor_states_low/board_vin \
/motor_states_low/comm_normal \
/motor_states_low/cur_angle \
/motor_states_low/dgain \
/motor_states_low/h817_rx_error0 \
/motor_states_low/h817_rx_error1 \
/motor_states_low/motor_current \
/motor_states_low/motor_outer_temp \
/motor_states_low/motor_output \
/motor_states_low/motor_temp \
/motor_states_low/pgain \
/motor_states_low/ref_angle \
/motor_states_low/servo_alarm \
/motor_states_throttle \
/odom \
/off_lfsensor \
/off_lhsensor \
/off_rfsensor \
/off_rhsensor \
/pc_monitor/tablis/eth1/receive \
/pc_monitor/tablis/eth1/transmit \
/ref_capture_point \
/ref_contact_states \
/ref_lfsensor \
/ref_lhsensor \
/ref_rfsensor \
/ref_rhsensor \
/rfsensor \
/rfsensor_cop \
/rhsensor \
/rhsensor_cop \
/robotsound \
/shm_servo_state \
/slave_delay_ans \
/slave_larm_wrench \
/slave_lleg_wrench \
/slave_rarm_wrench \
/slave_rleg_wrench \
/tf \
/tf_static \
/urata_status \
/zmp

)
