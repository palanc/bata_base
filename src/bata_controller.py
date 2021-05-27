#!/usr/bin/env python

import rospy
import serial
import struct
import math
import sys
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import UInt16MultiArray, UInt16

SERIAL_BAUD = 1000000
UPDATE_CMD = 0
INFO_CMD = 1
SENSOR_CMD = 2
MODE_CMD = 3
BRAKE_FREQ_CMD = 4

cur_traj = None
motor_modes = []
updated_modes = False
brake_freq = None

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

def traj_cb(msg):
  global cur_traj
  cur_traj = msg

def mode_cb(msg):
  global motor_modes, updated_modes

  if len(msg.data) != len(motor_modes):
    print 'Mode update does not match length of motor array'
  else:
    for i in range(len(motor_modes)):
      motor_modes[i] = msg.data[i]
    updated_modes = True

def brake_freq_cb(msg):
  global brake_freq

  if(msg.data < 1 or msg.data > 255):
    print 'Brake frequency value must be in interval [1-255]'
  else:
    brake_freq = msg.data

def main():
  global cur_traj, motor_modes, updated_modes, brake_freq

  rospy.init_node('bata_controller')
  print 'Node initialized'
  ser = serial.Serial(port='/dev/ttyACM0', 
                      baudrate=SERIAL_BAUD)

  print 'Writing sensor cmd'

  # Turn off sensor data
  ser.write(struct.pack("B", SENSOR_CMD))
  ser.write(struct.pack("B", 0))

  rospy.sleep(0.1)
  ser.reset_input_buffer()

  print('Turned off sensor data')

  ser.write(struct.pack("B", INFO_CMD))
  print 'Waiting for info'
  while ser.in_waiting < 1:
    pass
  motor_count = struct.unpack("B", ser.read())[0]

  while ser.in_waiting < 2*motor_count:
    pass

  joint_counts = []
  for i in range(motor_count):
    motor_modes.append(struct.unpack("B", ser.read())[0])
    joint_counts.append(struct.unpack("B", ser.read())[0])
  
  print('Motor count: %d'%motor_count)
  print('Motor modes: ' + str(motor_modes))
  print('Joint counts: '+str(joint_counts))
  
  sensor_update_bytes = 4
  device_count = motor_count
  cmd_update_bytes = 2*motor_count
  for i in range(motor_count):
    sensor_update_bytes += 6+3*joint_counts[i]
    device_count += joint_counts[i]
    cmd_update_bytes += (joint_counts[i]+7)//8
  print 'cmd_update_bytes = %d'%cmd_update_bytes
  print 'Turning on sensor data'
  # Turn on sensor data
  ser.write(struct.pack("B", SENSOR_CMD))
  ser.write(struct.pack("B", 1))
  
  sensor_msg = JointState()
  for i in range(motor_count):
    sensor_msg.name.append('m'+str(i))
    sensor_msg.position.append(0)
    sensor_msg.velocity.append(0)
    sensor_msg.effort.append(0)
    for j in range(joint_counts[i]):
      sensor_msg.name.append('j_'+str(i)+'_'+str(j))
      sensor_msg.position.append(0)
      sensor_msg.velocity.append(0)
      sensor_msg.effort.append(0)

  sensor_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  traj_sub = rospy.Subscriber("cmd_trajectory", JointTrajectory, traj_cb)
  mode_sub = rospy.Subscriber('motor_modes', UInt16MultiArray, mode_cb)
  freq_sub = rospy.Subscriber('brake_freq', UInt16, brake_freq_cb)

  while not rospy.is_shutdown():

    if cur_traj is not None:
      now = rospy.Time.now()
      while(len(cur_traj.points) > 1 and now >= cur_traj.header.stamp + cur_traj.points[1].time_from_start):
        cur_traj.points.pop(0)
      if now >= cur_traj.header.stamp + cur_traj.points[0].time_from_start:
        # Check command dimensions
        if (len(cur_traj.points[0].positions) != device_count or
            len(cur_traj.points[0].velocities) != device_count or
            len(cur_traj.points[0].effort) != device_count):
          print('Fields of trajectory point do not all have a size of %d'%device_count)
        else:
          cmd_idx = 0
          traj_idx = 0
          cmd_update = [0]*cmd_update_bytes
          for i in range(motor_count):
            if(motor_modes[i] == 0):
              cur_val = cur_ma_to_val(cur_traj.points[0].effort[traj_idx])
              cmd_update[cmd_idx] = (cur_val >> 8) & 0x00FF
              cmd_update[cmd_idx + 1] = cur_val & 0x00FF
            elif(motor_modes[i] == 1):
              vel_val = vel_rad_sec_to_val(cur_traj.points[0].velocities[traj_idx])
              cmd_update[cmd_idx] = (vel_val >> 8)  & 0x00FF
              cmd_update[cmd_idx + 1] = vel_val & 0x00FF   
            elif(motor_modes[i] == 3):
              pos_val = pos_rad_to_val(cur_traj.points[0].positions[traj_idx])
              cmd_update[cmd_idx] = (pos_val >> 8)  & 0x00FF
              cmd_update[cmd_idx + 1] = pos_val & 0x00FF 
            else:
              print 'Invalid motor mode'
            cmd_idx += 1
            traj_idx += 1   
            for j in range(joint_counts[i]):
              if(j%8==0):
                cmd_idx += 1
              if cur_traj.points[0].effort[traj_idx] > sys.float_info.epsilon:
                cmd_update[cmd_idx] |= (0x1<<((7-(j%8))))
              traj_idx += 1
            cmd_idx += 1
          print 'Sent cmd update: ' + str(cmd_update)
          ser.write(struct.pack('B', UPDATE_CMD))
          ser.write(struct.pack('B'*len(cmd_update), *cmd_update))
          
        cur_traj.points.pop(0)
        if len(cur_traj.points) == 0:
          cur_traj = None
    elif updated_modes:
      print 'Updating motor mode'
      for i in range(motor_count):
        ser.write(struct.pack("B", MODE_CMD))
        ser.write(struct.pack("B", i))
        ser.write(struct.pack("B", motor_modes[i]))
      updated_modes = False

    elif brake_freq is not None:
      print 'Updating brake frequency'
      ser.write(struct.pack("B", BRAKE_FREQ_CMD))
      ser.write(struct.pack("B", brake_freq))
      brake_freq = None

    if ser.in_waiting >= sensor_update_bytes:
      # Get time stamp
      time_stamp = struct.unpack(">L", ser.read(4))[0]
      sensor_msg.header.stamp = rospy.Time.from_sec(time_stamp/1000000.0)
      cur_idx = 0
      for i in range(motor_count):
        sensor_msg.position[cur_idx] = pos_val_to_rad(struct.unpack(">h",ser.read(2))[0])
        sensor_msg.velocity[cur_idx] = vel_val_to_rad_sec(struct.unpack(">h",ser.read(2))[0])
        sensor_msg.effort[cur_idx] = cur_val_to_ma(struct.unpack(">h", ser.read(2))[0])
        cur_idx += 1
        for j in range(joint_counts[i]):
          sensor_msg.position[cur_idx] = (2*math.pi/((1<<16)))*struct.unpack(">h", ser.read(2))[0]
          sensor_msg.effort[cur_idx] = struct.unpack("B", ser.read(1))[0]
          cur_idx += 1
      sensor_pub.publish(sensor_msg)

  ser.close()

if __name__ == '__main__':
  main()
