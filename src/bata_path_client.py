#! /usr/bin/env python

import rospy
import actionlib
from bata_base.msg import BataPathAction, BataPathGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math

def set_goal_from_cur_state(motor_count, joint_counts):
  msg = rospy.wait_for_message('joint_states', JointState)
  goal = BataPathGoal()
  goal.path.points.append(JointTrajectoryPoint())

  device_count = 0
  for i in range(motor_count):
    device_count += 1+joint_counts[i]

  state_idx = 0
  if len(msg.name) != device_count_:
    print('Expected %d joints, recieved %d'%(device_count, len(msg.name)))
    return goal
 
  for i in range(motor_count):
    if(msg.name[state_idx] != 'm'+str(i)):
      print('Expected joint %d to have name m%d'%(state_idx,i)
      return goal
    state_idx += 1
    for j in range(joint_counts[i]):
      if(msg.name[state_idx] != 'm'+str(i)+'_j'+str(j)):
        print('Expected joint %d to have name m%d_j%d'%(state_idx,i,j))
        return goal
      goal.path.joint_names.append(state.name[state_idx])
      goal.path.points[0].positions.append(state.position[state_idx])
      goal.path.points[0].velocities.append(0.0)
      goal.path.points[0].accelerations.append(0.0)
      goal.path.points[0].effort.append(0.0)        
      state_idx += 1

  return goal

def bata_path_client(motor_count, joint_counts):
  client = actionlib.SimpleActionClient('bata_path_exec', BataPathAction)
  client.wait_for_server()

  # Setup the goal
  goal = set_goal_from_cur_state(motor_count, joint_counts)
  goal.path.points[0].positions[-1] = math.pi/4.0

  clent.send_goal(goal)
  client.wait_for_result()

  return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('bata_path_client')
        motor_count = rospy.get_param('motor_count')
        joint_counts = rospy.get_param('joint_counts')
        result = bata_path_client(motor_count, joint_counts)
        print("Result:"+str(result.waypoint_success))
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
