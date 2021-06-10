#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import UInt16MultiArray, UInt16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from getch import _Getch
 
MOTOR_COUNT = 1
JOINT_COUNTS = [10]
POSITION_DEL = math.pi/10
VELOCITY_DEL = math.pi/30
CURRENT_DEL = 3

if __name__ == '__main__':

  rospy.init_node('test_bata_controller')
  mode_pub = rospy.Publisher('motor_modes', UInt16MultiArray, queue_size=1)
  traj_pub = rospy.Publisher('cmd_trajectory', JointTrajectory, queue_size=1)
  freq_pub = rospy.Publisher('brake_freq', UInt16, queue_size=1)
  getch = _Getch()

  mode_msg = UInt16MultiArray()
  mode_msg.data = [3]*MOTOR_COUNT
  
  traj_msg = JointTrajectory()
  traj_pt_msg = JointTrajectoryPoint()
  for i in range(MOTOR_COUNT):
    traj_pt_msg.positions.append(0.0)
    traj_pt_msg.velocities.append(0.0)
    traj_pt_msg.accelerations.append(0.0)
    traj_pt_msg.effort.append(0.0)
    for j in range(JOINT_COUNTS[i]):
      traj_pt_msg.positions.append(0.0)
      traj_pt_msg.velocities.append(0.0)
      traj_pt_msg.accelerations.append(0.0)
      traj_pt_msg.effort.append(0.0)
  traj_msg.points.append(traj_pt_msg)

  freq_msg = UInt16()
  freq_msg.data = 255

  sp = 0.0
  motor_idx = 0
  brake_idx = 0  
  while not rospy.is_shutdown():
    cmd = getch()

    if cmd == 'm':
      resp = raw_input('Enter the motor to control: [0-%d]'%(MOTOR_COUNT-1))
      motor_idx = int(resp)
      brake_idx = 0
      sp = 0.0
      print 'Controlling motor %d'%motor_idx
    elif cmd == 'b':
      resp = raw_input('Enter the brake to control: [0-%d]'%(JOINT_COUNTS[motor_idx]-1))
      brake_idx = int(resp)
      sp = 0.0
      print 'Controlling brake %d'%brake_idx
    elif cmd == 'p':
      mode_msg.data[motor_idx] = 3
      mode_pub.publish(mode_msg)
      sp = 0.0

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      traj_msg.points[0].positions[idx] = sp
      print 'Motor %d set to joint mode'%motor_idx
    elif cmd == 'v':
      mode_msg.data[motor_idx] = 1
      mode_pub.publish(mode_msg)
      sp = 0.0

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      traj_msg.points[0].velocities[idx] = sp
      print 'Motor %d set to velocity mode'%motor_idx
    elif cmd == 'c':
      mode_msg.data[motor_idx] = 0
      mode_pub.publish(mode_msg)
      sp = 0.0

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      traj_msg.points[0].effort[idx] = sp
      print 'Motor %d set to current mode'%motor_idx
    elif cmd == '+':
      print 'About to turn on brake %d of motor %d'%(brake_idx, motor_idx)
      raw_input('Press enter to continue')

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      idx += 1 + brake_idx

      traj_msg.points[0].effort[idx] = 1
      traj_pub.publish(traj_msg)
      print 'Turned on brake %d of motor %d'%(brake_idx, motor_idx)
    
    elif cmd == '-':
      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      idx += 1 + brake_idx

      traj_msg.points[0].effort[idx] = 0
      traj_pub.publish(traj_msg)
      print 'Turned off brake %d of motor %d'%(brake_idx, motor_idx)

    elif cmd == 'O':
      print 'About to turn on all of motor %d brakes'%(motor_idx)
      raw_input('Press enter to continue')

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      idx += 1

      for i in range(JOINT_COUNTS[motor_idx]):
        traj_msg.points[0].effort[idx+i] = 1
        traj_pub.publish(traj_msg)
        rospy.sleep(0.01)

    elif cmd == 'A':
      print 'About to turn off all of motor %d brakes'%(motor_idx)
      raw_input('Press enter to continue')

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      idx += 1

      for i in range(JOINT_COUNTS[motor_idx]):
        traj_msg.points[0].effort[idx+i] = 0
        traj_pub.publish(traj_msg)
        rospy.sleep(0.01)

    elif cmd == 'w':

      idx = 0
      mode_str = ''
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1

      if mode_msg.data[motor_idx] == 0:
        sp += CURRENT_DEL
        traj_msg.points[0].effort[idx] = sp
        mode_str = 'current'
      elif mode_msg.data[motor_idx] == 1:
        sp += VELOCITY_DEL
        traj_msg.points[0].velocities[idx] = sp
        mode_str = 'velocity'
      elif mode_msg.data[motor_idx] == 3:
        sp += POSITION_DEL
        traj_msg.points[0].positions[idx] = sp
        mode_str = 'position'
      traj_pub.publish(traj_msg)
      print 'Increased motor %d '%motor_idx + mode_str + ' to %f'%sp
  
    elif cmd == 's':
      idx = 0
      mode_str = ''
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1

      if mode_msg.data[motor_idx] == 0:
        sp -= CURRENT_DEL
        traj_msg.points[0].effort[idx] = sp
        mode_str = 'current'
      elif mode_msg.data[motor_idx] == 1:
        sp -= VELOCITY_DEL
        traj_msg.points[0].velocities[idx] = sp
        mode_str = 'velocity'
      elif mode_msg.data[motor_idx] == 3:
        sp -= POSITION_DEL
        traj_msg.points[0].positions[idx] = sp
        mode_str = 'position'
      traj_pub.publish(traj_msg)
      print 'Decreased motor %d '%motor_idx + mode_str + ' to %f'%sp  

    elif cmd == 'u':
      mode_str = ''
      if mode_msg.data[motor_idx] == 0:
        mode_str = 'current'
      elif mode_msg.data[motor_idx] == 1:
        mode_str = 'velocity'
      elif mode_msg.data[motor_idx] == 3:
        mode_str = 'position'

      resp = raw_input('Enter new ' + mode_str + ' set point:')
      sp = float(resp) 

      idx = 0
      for i in range(motor_idx):
        idx += JOINT_COUNTS[i] + 1
      if mode_msg.data[motor_idx] == 0:
        traj_msg.points[0].effort[idx] = sp
      elif mode_msg.data[motor_idx] == 1:
        traj_msg.points[0].velocities[idx] = sp
      elif mode_msg.data[motor_idx] == 3:
        traj_msg.points[0].positions[idx] = sp 
      traj_pub.publish(traj_msg)

      print 'Set motor %d '%motor_idx + mode_str + ' to %f'%sp

    elif cmd == 'f':
      resp = raw_input('Enter new brake frequency:')
      freq = int(resp)
      if freq < 1 or freq > 255:
        print 'Given frequency is not in range ([1-255])'
      else:
        freq_msg.data = freq
        freq_pub.publish(freq_msg)
        print 'Set brake frequency to %d'%freq

    elif cmd == 'q':
      break
    else:
      print 'Unrecognized cmd' 

  '''
  mode_pub.publish(mode_msg)

  rospy.sleep(1.0)
  
  mode_pub.publish(mode_msg)

  rospy.sleep(1.0)

  print 'Sending position command'

  traj_pub.publish(traj_msg)
  
  print 'Sent position command'
  rospy.sleep(1.0)

  '''
  '''
  print 'Switching to velocity mode'
  mode_msg.data[0] = 1
  mode_pub.publish(mode_msg)

  print 'Sending velocity commands'
  traj_msg.points[0].velocities[0] = 0.3
  while not rospy.is_shutdown():
    traj_pub.publish(traj_msg)
    rospy.sleep(2.0)
    traj_msg.points[0].velocities[0] *= -1
  '''  

