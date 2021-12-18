#!/usr/bin/env python

import rospy
import serial
import struct
import math
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray
from bata_base.msg import BataCmd, OpticalSensors

SERIAL_BAUD = 1000000
UPDATE_CMD = 0
INFO_CMD = 1
SENSOR_CMD = 2
MODE_CMD = 3

cur_cmd = None
motor_count = 0
motor_modes = []
joint_counts = []
optical_counts = []

def pos_val_to_rad(val):
  return (val-((4096-0)/2.0))*(2*math.pi/4096)

def pos_rad_to_val(rad):
  return int(rad*(4096/(2*math.pi)) + 4096/2.0)

def vel_val_to_rad_sec(val):
  return val * 0.229 * 2*math.pi / 60.0

def vel_rad_sec_to_val(rad_sec):
  return int(rad_sec / (0.229 * 2*math.pi / 60.0))

def cur_val_to_ma(val):
  return 2.69*val

def cur_ma_to_val(ma):
  return int(ma / 2.69)

def cmd_cb(msg):
  global cur_cmd

  if len(msg.chain_cmds) != len(motor_modes):
    print('Msg does not have the correct amount of chain cmds (expected %d, received %d)'
          %(len(motor_modes), len(msg.chain_cmds)))
    return  

  for i in range(len(motor_modes)):
    if len(msg.chain_cmds[i].enable_brake) != joint_counts[i]:
      print('Chain cmd %d does not have the correct amount of brake commands (expected %d, received %d)'
           %(i, joint_counts[i], len(msg.chain_cmds[i].enable_brake)))
      return             

  cur_cmd = msg

def init_motor_board(ser):
  global motor_count, motor_modes, joint_counts, optical_counts

  # Turn off sensor data
  ser.write(struct.pack("B", SENSOR_CMD))
  ser.write(struct.pack("B", 0))
  rospy.sleep(0.1)
  ser.reset_input_buffer()

  # Get motor/joint info
  ser.write(struct.pack("B", INFO_CMD))

  motor_count = 0
  motor_modes = []
  joint_counts = []
  optical_counts = []

  while ser.in_waiting < 1:
    pass
  motor_count = struct.unpack("B", ser.read())[0]

  while ser.in_waiting < 3*motor_count:
    pass

  for i in range(motor_count):
    motor_modes.append(struct.unpack("B", ser.read())[0])
    joint_counts.append(struct.unpack("B", ser.read())[0])
    optical_counts.append(struct.unpack("B", ser.read())[0])

  # Turn on sensor data
  ser.write(struct.pack("B", SENSOR_CMD))
  ser.write(struct.pack("B", 1))


def main():
  global cur_cmd, motor_modes

  rospy.init_node('bata_interface')
  ser = serial.Serial(port='/dev/ttyACM0', 
                      baudrate=SERIAL_BAUD)

  print( 'Started interface')
  init_motor_board(ser)

  print('Motor count: %d'%motor_count)
  print('Motor modes: ' + str(motor_modes))
  print('Joint counts: '+str(joint_counts))
  print('Optical counts: '+str(optical_counts))  

  # Check that the param server agrees with above values
  if not rospy.has_param('motor_count'):
    print('motor_count parameter not set')
    return
  if not rospy.has_param('joint_counts'):
    print('joint_counts parameter not set')
    return
  if not rospy.has_param('optical_counts'):
    print('optical_counts parameter not set')
    return
  if rospy.get_param('motor_count') != motor_count:
    print('motor_count conflict (HW expects %d, received %d)'
          %(motor_count,rospy.get_param('motor_count')))
    return
  
  for i, joint_count in enumerate(rospy.get_param('joint_counts')):
    if joint_count != joint_counts[i]:
      print('joint_counts[%d] conflict (HW expects %d, received %d)'
            %(i, joint_counts[i], joint_count))
      return
    
  for i, optical_count in enumerate(rospy.get_param('optical_counts')):
    if optical_count != optical_counts[i]:
      print('optical_counts[%d] conflict (HW expects %d, received %d)'
            %(i, optical_counts[i], optical_count))
      return 

  sensor_update_bytes = 4
  cmd_update_bytes = 2*motor_count
  for i in range(motor_count):
    sensor_update_bytes += 6+3*joint_counts[i] + 1 * optical_counts[i]
    cmd_update_bytes += (joint_counts[i]+7)//8

  joint_home_vals = []  
  encoder_msg = JointState()
  for i in range(motor_count):   
    joint_name = 'm'+str(i)+'_name'
    if not rospy.has_param(joint_name):
      encoder_msg.name.append('m'+str(i))
    else:
      encoder_msg.name.append(rospy.get_param(joint_name))    
    
    encoder_msg.position.append(0)
    encoder_msg.velocity.append(0)
    encoder_msg.effort.append(0)
    joint_home_vals.append([])
    for j in range(joint_counts[i]):
      joint_name = 'm'+str(i)+'_j'+str(j)+'_name'
      if not rospy.has_param(joint_name):
        encoder_msg.name.append('m'+str(i)+'_j'+str(j))
      else:
        encoder_msg.name.append(rospy.get_param(joint_name))    
    
      encoder_msg.position.append(0)
      encoder_msg.velocity.append(0)
      encoder_msg.effort.append(0)

      home_val_name = encoder_msg.name[-1]+"_home_val"
      if not rospy.has_param(home_val_name):
        joint_home_vals[i].append(0.0)
        print ('Did not find param '+home_val_name+', setting to 0.0')
      else:
        joint_home_vals[i].append(rospy.get_param(home_val_name))

  optical_msg = OpticalSensors()
  for i in range(motor_count):
    optical_msg.sensors.append(UInt16MultiArray())
    for j in range(optical_counts[i]):
      optical_msg.sensors[i].data.append(0)

  sensor_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  optical_pub = rospy.Publisher('optical_sensors', OpticalSensors, queue_size=1)
  traj_sub = rospy.Subscriber("cmd_trajectory", BataCmd, cmd_cb)

  start_stamp = None
  prev_hw_stamp = None
  encoder_msg.header.stamp = rospy.Time.now()
  while not rospy.is_shutdown():

    if cur_cmd is not None:

      # Update motor modes if necessary
      for i in range(motor_count):
        if motor_modes[i] != cur_cmd.chain_cmds[i].motor_mode:
          motor_modes[i] = cur_cmd.chain_cmds[i].motor_mode
          ser.write(struct.pack("B", MODE_CMD))
          ser.write(struct.pack("B", i))
          ser.write(struct.pack("B", motor_modes[i]))
          
      # Send motor and brake commands
      cmd_idx = 0
      cmd_update = [0]*cmd_update_bytes
      for i in range(motor_count):
        if(motor_modes[i] == 0):
          cur_val = cur_ma_to_val(cur_cmd.chain_cmds[i].motor_cmd)
          cmd_update[cmd_idx] = (cur_val >> 8) & 0x00FF
          cmd_update[cmd_idx + 1] = cur_val & 0x00FF
        elif(motor_modes[i] == 1):
          vel_val = vel_rad_sec_to_val(cur_cmd.chain_cmds[i].motor_cmd)
          cmd_update[cmd_idx] = (vel_val >> 8)  & 0x00FF
          cmd_update[cmd_idx + 1] = vel_val & 0x00FF   
        elif(motor_modes[i] == 3):
          pos_val = pos_rad_to_val(cur_cmd.chain_cmds[i].motor_cmd)
          cmd_update[cmd_idx] = (pos_val >> 8)  & 0x00FF
          cmd_update[cmd_idx + 1] = pos_val & 0x00FF 
        else:
          print( 'Invalid motor mode')
        cmd_idx += 1
        for j in range(joint_counts[i]):
          if(j%8==0):
            cmd_idx += 1
          if cur_cmd.chain_cmds[i].enable_brake[j]:
            cmd_update[cmd_idx] |= (0x1<<((7-(j%8))))
        cmd_idx += 1

      ser.write(struct.pack('B', UPDATE_CMD))
      ser.write(struct.pack('B'*len(cmd_update), *cmd_update))
      #print('Sent '+str(cmd_update))
      cur_cmd = None

    if ser.in_waiting >= sensor_update_bytes:
      # Get time stamp
      time_stamp = struct.unpack(">L", ser.read(4))[0]
      #time_stamp = rospy.Duration.from_sec(time_stamp/1000000.0)
      #if (prev_hw_stamp is None) or (prev_hw_stamp > time_stamp):
      #  start_stamp = rospy.Time.now() - 2*time_stamp
      #
      #encoder_msg.header.stamp = start_stamp + time_stamp
      #prev_hw_stamp = time_stamp
      prev_stamp = encoder_msg.header.stamp.to_sec()
      encoder_msg.header.stamp = rospy.Time.now()
      optical_msg.header.stamp = encoder_msg.header.stamp
      now_stamp = encoder_msg.header.stamp.to_sec()
      status_bad = False

      joint_idx = 0
      for i in range(motor_count):
        encoder_msg.position[joint_idx] = pos_val_to_rad(struct.unpack(">h",ser.read(2))[0])
        encoder_msg.velocity[joint_idx] = vel_val_to_rad_sec(struct.unpack(">h",ser.read(2))[0])
        encoder_msg.effort[joint_idx] = cur_val_to_ma(struct.unpack(">h", ser.read(2))[0])
        joint_idx += 1
        for j in range(joint_counts[i]):

          encoder_val = (2*math.pi/((1<<16)))*struct.unpack(">h", ser.read(2))[0]
          encoder_val -= joint_home_vals[i][j]
          if encoder_val > math.pi:
            encoder_val -= 2*math.pi
          elif encoder_val < -math.pi:
            encoder_val += 2*math.pi

          encoder_msg.velocity[joint_idx] = (encoder_val - encoder_msg.position[joint_idx])/(now_stamp-prev_stamp)
          encoder_msg.position[joint_idx] = encoder_val
          encoder_msg.effort[joint_idx] = struct.unpack("B", ser.read(1))[0]
          
          if encoder_msg.effort[joint_idx] != 0x04:
            status_bad = True

          joint_idx += 1

        for j in range(optical_counts[i]):
          optical_msg.sensors[i].data[j] = struct.unpack("B", ser.read(1))[0]
      
      if status_bad:
        print('Status bad: '+str(encoder_msg))
      else:
        sensor_pub.publish(encoder_msg)
      optical_pub.publish(optical_msg)      

    elif(rospy.Time.now() - encoder_msg.header.stamp > rospy.Duration.from_sec(10)):
      print ('Reseting connection to motor board')
      ser.close()
      rospy.sleep(0.1)
      ser = serial.Serial(port='/dev/ttyACM0', 
                          baudrate=SERIAL_BAUD)
      init_motor_board(ser)
      print ('Reset complete')
      encoder_msg.header.stamp = rospy.Time.now()

  ser.close()
  
if __name__ == '__main__':
  main()
