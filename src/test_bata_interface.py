#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState 
from bata_base.msg import BataCmd, BataChainCmd
from getch import _Getch
 
POSITION_DEL = math.pi/10
VELOCITY_DEL = math.pi/30
CURRENT_DEL = 3

if __name__ == '__main__':

  rospy.init_node('test_bata_controller')
  motor_count = rospy.get_param('motor_count')
  joint_counts = rospy.get_param('joint_counts')

  getch = _Getch()

  # Initialize in velocity mode, 0.0 speed, all brakes off  
  cmd_msg = BataCmd()
  for i in range(motor_count):
    chain_cmd_msg = BataChainCmd()
    chain_cmd_msg.enable_brake = [False]*joint_counts[i]
    chain_cmd_msg.motor_mode = 1
    cmd_msg.chain_cmds.append(chain_cmd_msg)

  cmd_pub = rospy.Publisher('cmd_trajectory', BataCmd, queue_size=1)
  cmd_pub.publish(cmd_msg)

  motor_idx = 0
  brake_idx = 0  
  while not rospy.is_shutdown():
    cmd = getch()

    if cmd == 'm':
      resp = input('Enter the motor to control: [0-%d]'%(motor_count-1))
      motor_idx = int(resp)
      brake_idx = 0
      print ('Controlling motor %d'%motor_idx)
    elif cmd == 'b':
      resp = input('Enter the brake to control: [0-%d]'%(joint_counts[motor_idx]-1))
      brake_idx = int(resp)
      sp = 0.0
      print( 'Controlling brake %d of motor %d'%(brake_idx, motor_idx))
    elif cmd == 'p':
      joint_msg = rospy.wait_for_message('joint_states', JointState)
      cmd_msg.chain_cmds[motor_idx].motor_mode = 3
      for i in range(len(joint_msg.name)):
        if joint_msg.name[i] == 'm'+str(motor_idx):
          cmd_msg.chain_cmds[motor_idx].motor_cmd = joint_msg.position[i]
          break
      cmd_pub.publish(cmd_msg)

      print( 'Motor %d set to joint mode'%motor_idx)

    elif cmd == 'v':

      cmd_msg.chain_cmds[motor_idx].motor_mode = 1
      cmd_msg.chain_cmds[motor_idx].motor_cmd = 0.0
      cmd_pub.publish(cmd_msg)

      print ('Motor %d set to velocity mode'%motor_idx)
    elif cmd == 'c':

      cmd_msg.chain_cmds[motor_idx].motor_mode = 0
      cmd_msg.chain_cmds[motor_idx].motor_cmd = 0.0
      cmd_pub.publish(cmd_msg)

      print( 'Motor %d set to current mode'%motor_idx)
    elif cmd == '+':
      print( 'About to turn on brake %d of motor %d'%(brake_idx, motor_idx))
      input('Press enter to continue')

      cmd_msg.chain_cmds[motor_idx].enable_brake[brake_idx] = True
      cmd_pub.publish(cmd_msg)

      print ('Turned on brake %d of motor %d'%(brake_idx, motor_idx))
    
    elif cmd == '-':

      cmd_msg.chain_cmds[motor_idx].enable_brake[brake_idx] = False
      cmd_pub.publish(cmd_msg)

      print ('Turned off brake %d of motor %d'%(brake_idx, motor_idx))

    elif cmd == 'O':
      print ('About to turn on all of motor %d brakes'%(motor_idx))
      input('Press enter to continue')

      for i in range(joint_counts[motor_idx]):
        cmd_msg.chain_cmds[motor_idx].enable_brake[i] = True
      cmd_pub.publish(cmd_msg)        

    elif cmd == 'A':
      print ('About to turn off all of motor %d brakes'%(motor_idx))
      input('Press enter to continue')

      for i in range(joint_counts[motor_idx]):
        cmd_msg.chain_cmds[motor_idx].enable_brake[i] = False
      cmd_pub.publish(cmd_msg)        

    elif cmd == 'w':

      mode_str = ''

      if cmd_msg.chain_cmds[motor_idx].motor_mode == 0:
        cmd_msg.chain_cmds[motor_idx].motor_cmd += CURRENT_DEL        
        mode_str = 'current'
      elif cmd_msg.chain_cmds[motor_idx].motor_mode == 1:
        cmd_msg.chain_cmds[motor_idx].motor_cmd += VELOCITY_DEL
        mode_str = 'velocity'
      elif cmd_msg.chain_cmds[motor_idx].motor_mode == 3:
        cmd_msg.chain_cmds[motor_idx].motor_cmd += POSITION_DEL
        mode_str = 'position'
      cmd_pub.publish(cmd_msg)
      print ('Increased motor %d '%motor_idx + mode_str + ' to %f'%cmd_msg.chain_cmds[motor_idx].motor_cmd)
  
    elif cmd == 's':
      mode_str = ''

      if cmd_msg.chain_cmds[motor_idx].motor_mode == 0:
        cmd_msg.chain_cmds[motor_idx].motor_cmd -= CURRENT_DEL        
        mode_str = 'current'
      elif cmd_msg.chain_cmds[motor_idx].motor_mode == 1:
        cmd_msg.chain_cmds[motor_idx].motor_cmd -= VELOCITY_DEL
        mode_str = 'velocity'
      elif cmd_msg.chain_cmds[motor_idx].motor_mode == 3:
        cmd_msg.chain_cmds[motor_idx].motor_cmd -= POSITION_DEL
        mode_str = 'position'
      cmd_pub.publish(cmd_msg)
      print ('Decreased motor %d '%motor_idx + mode_str + ' to %f'%cmd_msg.chain_cmds[motor_idx].motor_cmd)

    elif cmd == 'u':
      mode_str = ''
      if cmd_msg.chain_cmds[motor_idx].motor_mode == 0:
        mode_str = 'current'
      elif cmd_msg.chain_cmds[motor_idx].motor_mode == 1:
        mode_str = 'velocity'
      elif cmd_msg.chain_cmds[motor_idx].motor_mode == 3:
        mode_str = 'position'

      resp = input('Enter new ' + mode_str + ' set point:')
      cmd_msg.chain_cmds[motor_idx].motor_cmd = float(resp) 
      cmd_pub.publish(cmd_msg)

      print ('Set motor %d '%motor_idx + mode_str + ' to %f'%cmd_msg.chain_cmds[motor_idx].motor_cmd)

    elif cmd == 'q':
      break
    else:
      print( 'Unrecognized cmd' )

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

