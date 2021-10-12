#! /usr/bin/env python

import rospy
import actionlib
from bata_base.msg import BataPathAction, BataPathGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math
import sys
import copy

def set_goal_from_cur_state(motor_count, joint_counts):
  msg = rospy.wait_for_message('joint_states', JointState)
  goal = BataPathGoal()
  goal.path.points.append(JointTrajectoryPoint())

  device_count = 0
  for i in range(motor_count):
    device_count += 1+joint_counts[i]

  state_idx = 0
  if len(msg.name) != device_count:
    print('Expected %d joints, recieved %d'%(device_count, len(msg.name)))
    return goal
 
  for i in range(motor_count):
    if(msg.name[state_idx] != 'm'+str(i)):
      print('Expected joint %d to have name m%d'%(state_idx,i))
      return goal
    state_idx += 1
    for j in range(joint_counts[i]):
      if(msg.name[state_idx] != 'm'+str(i)+'_j'+str(j)):
        print('Expected joint %d to have name m%d_j%d'%(state_idx,i,j))
        return goal
      goal.path.joint_names.append(msg.name[state_idx])
      goal.path.points[0].positions.append(msg.position[state_idx])
      goal.path.points[0].velocities.append(0.0)
      goal.path.points[0].accelerations.append(0.0)
      goal.path.points[0].effort.append(0.0)        
      state_idx += 1

  return goal

def bata_path_client(motor_count, joint_counts):
  client = actionlib.SimpleActionClient('bata_path_exec', BataPathAction)

  print 'Waiting for action server'

  client.wait_for_server()

  # Setup the goal
  print 'Setting up goal'
  goal = set_goal_from_cur_state(motor_count, joint_counts)

  # Zigzag
  '''
  for i in range(len(goal.path.points[0].positions)):
    if i % 2 == 0:
      goal.path.points[i].positions[len(goal.path.points[0].positions)-1-i] = math.pi/6.0
    else:
      goal.path.points[i].positions[len(goal.path.points[0].positions)-1-i] = -math.pi/6.0
    if i < len(goal.path.points[0].positions)-1:
      goal.path.points.append(copy.deepcopy(goal.path.points[i]))
  '''

  # Unzigzag
  '''
  for i in range(len(goal.path.points[0].positions)):
    goal.path.points[i].positions[i] = 0.0
    if i < len(goal.path.points[0].positions)-1:
      goal.path.points.append(copy.deepcopy(goal.path.points[i]))
  '''

  '''
  # Grasping
  # Wrap around object 1
  goal.path.points[0].positions[9] = 0.85
  goal.path.points[0].positions[8] = 0.85
  goal.path.points[0].positions[7] = 0.85
  goal.path.points[0].positions[6] = 0.85
  goal.path.points[0].positions[5] = 0.85
  
  # Move base joint to avoid collision
  goal.path.points.append(copy.deepcopy(goal.path.points[0]))
  goal.path.points[1].positions[0] = 0.4

  # Wrap around object 2
  goal.path.points.append(copy.deepcopy(goal.path.points[1]))
  goal.path.points[2].positions[2] = -0.92
  goal.path.points[2].positions[1] = -0.92

  # Wrap around object 2
  goal.path.points.append(copy.deepcopy(goal.path.points[2]))
  goal.path.points[3].positions[4] = -0.92
  goal.path.points[3].positions[3] = -0.92

  # Push right
  goal.path.points.append(copy.deepcopy(goal.path.points[3]))
  goal.path.points[4].positions[0] = 0.85

  # Push left
  goal.path.points.append(copy.deepcopy(goal.path.points[4]))
  goal.path.points[5].positions[0] = 0.4

  # Push right
  goal.path.points.append(copy.deepcopy(goal.path.points[5]))
  goal.path.points[6].positions[0] = 0.85
  '''

  '''
  # Grasping 2
  # Wrap around object 1
  goal.path.points[0].positions[9] = 0.92
  goal.path.points[0].positions[8] = 0.92
  goal.path.points[0].positions[7] = 0.92
  goal.path.points[0].positions[6] = 0.92

  # Wrap around object 2
  goal.path.points.append(copy.deepcopy(goal.path.points[0]))
  goal.path.points[1].positions[5] = -0.92
  goal.path.points[1].positions[4] = -0.92
  goal.path.points[1].positions[3] = -0.92
  goal.path.points[1].positions[2] = -0.92

  # Push right
  goal.path.points.append(copy.deepcopy(goal.path.points[1]))
  goal.path.points[2].positions[1] = 0.6
  goal.path.points[2].positions[0] = 0.6

  # Push left
  goal.path.points.append(copy.deepcopy(goal.path.points[2]))
  goal.path.points[3].positions[1] = 0.0
  goal.path.points[3].positions[0] = 0.0

  # Push right
  goal.path.points.append(copy.deepcopy(goal.path.points[3]))
  goal.path.points[4].positions[1] = 0.6
  goal.path.points[4].positions[0] = 0.6
  '''

  # Grasping 3
  # Wrap around object 1
  goal.path.points[0].positions[9] = 0.89
  goal.path.points[0].positions[8] = 1.05
  goal.path.points[0].positions[7] = 0.90
  goal.path.points[0].positions[6] = 0.85
  goal.path.points[0].positions[5] = 0.92

  # Move base joint to avoid collision
  goal.path.points.append(copy.deepcopy(goal.path.points[0]))
  goal.path.points[1].positions[0] = 0.4

  # Wrap around object 2
  goal.path.points.append(copy.deepcopy(goal.path.points[1]))
  goal.path.points[2].positions[2] = -0.95
  goal.path.points[2].positions[1] = -0.87

  # Wrap around object 2
  goal.path.points.append(copy.deepcopy(goal.path.points[2]))
  goal.path.points[3].positions[4] = -0.91
  goal.path.points[3].positions[3] = -0.98

  # Push right
  goal.path.points.append(copy.deepcopy(goal.path.points[3]))
  goal.path.points[4].positions[0] = 0.85

  # Push left
  goal.path.points.append(copy.deepcopy(goal.path.points[4]))
  goal.path.points[5].positions[0] = 0.4

  # Push right
  goal.path.points.append(copy.deepcopy(goal.path.points[5]))
  goal.path.points[6].positions[0] = 0.85


  print 'Sending goal'

  client.send_goal(goal)

  print 'Wait for goal'    

  client.wait_for_result()

  print 'Done'

  return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('bata_path_client')
        motor_count = rospy.get_param('motor_count')
        joint_counts = rospy.get_param('joint_counts')
        result = bata_path_client(motor_count, joint_counts)
        print("Result:"+str(result.waypoint_success))
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
