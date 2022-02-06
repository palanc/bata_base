#!/usr/bin/env python

import rospy
import serial
import struct
import math
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray
from bata_base.msg import BataCmd, OpticalSensors
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

SERIAL_BAUD = 1000000
UPDATE_CMD = 1
INFO_CMD = 2
SENSOR_CMD = 3
MODE_CMD = 4
ENCODER_CONFIG_CMD = 6

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
    print('[bata_interface] Waiting for motor_count')
    rospy.sleep(0.1)
  motor_count = struct.unpack("B", ser.read())[0]

  while ser.in_waiting < 3*motor_count:
    print('[bata_interface] Waiting for joint_counts')
    rospy.sleep(0.1)

  for i in range(motor_count):
    motor_modes.append(struct.unpack("B", ser.read())[0])
    joint_counts.append(struct.unpack("B", ser.read())[0])
    optical_counts.append(struct.unpack("B", ser.read())[0])

  # Turn on sensor data
  ser.write(struct.pack("B", SENSOR_CMD))
  ser.write(struct.pack("B", 1))

def configure_encoders(ser, joint_val_negate, joint_home_vals):
  # Turn off sensor data
  ser.write(struct.pack("B", SENSOR_CMD))
  ser.write(struct.pack("B", 0))
  rospy.sleep(0.1)
  ser.reset_input_buffer()
  
  # Get motor/joint info
  ser.write(struct.pack("B", ENCODER_CONFIG_CMD))  
  
  for i in range(motor_count):
    for j in range(joint_counts[i]):
      jh_conv_val = int((((1<<16)/(2*math.pi))*joint_home_vals[i][j])+0.5)
      ser.write(struct.pack("B", joint_val_negate[i][j]))
      ser.write(struct.pack("B", (jh_conv_val >> 8) & 0x0FF))
      ser.write(struct.pack("B", jh_conv_val & 0x0FF))
  
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
  joint_val_negate = []  
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
    joint_val_negate.append([])

    for j in range(joint_counts[i]):
      joint_name = 'm'+str(i)+'_j'+str(j)
      if not rospy.has_param(joint_name+'_name'):
        encoder_msg.name.append(joint_name)
      else:
        encoder_msg.name.append(rospy.get_param(joint_name+'_name'))    
    
      encoder_msg.position.append(0)
      encoder_msg.velocity.append(0)
      encoder_msg.effort.append(0)

      home_val_name = encoder_msg.name[-1]+"_home_val"
      if not rospy.has_param(home_val_name):
        joint_home_vals[i].append(0.0)
        print ('Did not find param '+home_val_name+', setting to 0.0')
      else:
        joint_home_vals[i].append(rospy.get_param(home_val_name))
        
      joint_negate_name = encoder_msg.name[-1]+'_val_negate'
      if not rospy.has_param(joint_negate_name):
        joint_val_negate[i].append(False)
      else:
        joint_val_negate[i].append(rospy.get_param(joint_negate_name))      

  configure_encoders(ser, joint_val_negate, joint_home_vals)

  optical_msg = OpticalSensors()
  optical_points_msg = MarkerArray()
  optical_offsets = []
  marker_id = 0
  for i in range(motor_count):
    optical_msg.sensors.append(UInt16MultiArray())
    prefix = "l_" if i == 0 else "r_"
    optical_offsets.append([])
    for j in range(optical_counts[i]):
      optical_msg.sensors[i].data.append(0)

      optical_marker = Marker()
      optical_marker.header.frame_id = prefix + 'finger_sensor' + str(j)  
      optical_marker.header.stamp = rospy.Time(0)
      optical_marker.ns = 'optical_points'
      optical_marker.id = marker_id
      optical_marker.type = Marker.LINE_STRIP
      optical_marker.action = Marker.ADD
      optical_marker.pose.position.x = 0.0
      optical_marker.pose.position.y = 0.0
      optical_marker.pose.position.z = 0.0
      optical_marker.pose.orientation.x = 0.0
      optical_marker.pose.orientation.y = 0.0
      optical_marker.pose.orientation.z = 0.0
      optical_marker.pose.orientation.w = 1.0
      optical_marker.scale.x = 0.003
      optical_marker.scale.y = 0.003
      optical_marker.scale.z = 0.003
      optical_marker.color.a = 1.0
      optical_marker.color.r = 1.0
      optical_marker.color.g = 0.0
      optical_marker.color.b = 0.0
      optical_marker.points.append(Point())
      optical_marker.points.append(Point())      
      optical_points_msg.markers.append(optical_marker)
      marker_id += 1
      
      offset_name = prefix+"finger_sensor"+str(j)+"_offset"
      if not rospy.has_param(offset_name):
        optical_offsets[i].append(0)
      else:
        optical_offsets[i].append(rospy.get_param(offset_name))
      

  sensor_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  optical_pub = rospy.Publisher('optical_sensors', OpticalSensors, queue_size=1)
  viz_pub = rospy.Publisher('optical_points', MarkerArray, queue_size=1)
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
      #ser.write(struct.pack('B'*len(cmd_update), *cmd_update))
      for i in range(len(cmd_update)):
        ser.write(struct.pack('B', cmd_update[i]))
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
      status_error = False
      status_miscal = False
      optical_error = False
      joint_idx = 0
      optical_idx = 0
      for i in range(motor_count):
        encoder_msg.position[joint_idx] = pos_val_to_rad(struct.unpack(">h",ser.read(2))[0])
        encoder_msg.velocity[joint_idx] = vel_val_to_rad_sec(struct.unpack(">h",ser.read(2))[0])
        encoder_msg.effort[joint_idx] = cur_val_to_ma(struct.unpack(">h", ser.read(2))[0])
        joint_idx += 1
        for j in range(joint_counts[i]):

          encoder_val = (2*math.pi/((1<<16)))*struct.unpack(">h", ser.read(2))[0]
          
          if encoder_val > math.pi:
            encoder_val -= 2*math.pi
          elif encoder_val < -math.pi:
            encoder_val += 2*math.pi

          encoder_msg.velocity[joint_idx] = (encoder_val - encoder_msg.position[joint_idx])/(now_stamp-prev_stamp)
          encoder_msg.position[joint_idx] = encoder_val
          encoder_msg.effort[joint_idx] = struct.unpack("B", ser.read(1))[0]
          
          if encoder_msg.effort[joint_idx] != 0x04:
            if encoder_msg.effort[joint_idx] > 0x06 or encoder_msg.effort[joint_idx] < 0x05:
              status_error = True
            else:
              status_miscal = True

          joint_idx += 1

        for j in range(optical_counts[i]):
          sensor_val = struct.unpack("B", ser.read(1))[0]
          #if sensor_val < 0 or sensor_val > 255:
          #  print(sensor_val)
          optical_msg.sensors[i].data[j] = sensor_val
          if optical_msg.sensors[i].data[j] < 255:
            optical_msg.sensors[i].data[j] += optical_offsets[i][j]
            optical_points_msg.markers[optical_idx].points[1].x = optical_msg.sensors[i].data[j]/1000.0
          else:
            optical_points_msg.markers[optical_idx].points[1].x = 0.0
          optical_idx += 1
          if optical_msg.sensors[i].data[j] < 0 or optical_msg.sensors[i].data[j] > 255:
            optical_error = True
      if status_error or status_miscal:
        print('Status bad: '+str(encoder_msg))
      
      if not status_error:
        sensor_pub.publish(encoder_msg)
      if not optical_error:
        optical_pub.publish(optical_msg) 
      viz_pub.publish(optical_points_msg)     

    elif(rospy.Time.now() - encoder_msg.header.stamp > rospy.Duration.from_sec(0.25)):
      print ('Reseting connection to motor board')
      #ser.reset_input_buffer()
      #ser.reset_output_buffer()
      ser.close()
      rospy.sleep(0.1)
      
      return
      #ser = serial.Serial(port='/dev/ttyACM0', 
      #                    baudrate=SERIAL_BAUD)
      #init_motor_board(ser)
      #ser.reset_input_buffer()
      #ser.reset_output_buffer()      
      print ('Reset complete')
      encoder_msg.header.stamp = rospy.Time.now()

  ser.close()
  
if __name__ == '__main__':
  main()
