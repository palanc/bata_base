#!/usr/bin/env python

import rospy
import actionlib
from bata_base.msg import BataPathFeedback, BataPathResult, BataPathAction
from bata_base.msg import BataCmd, BataChainCmd
from sensor_msgs.msg import JointState
from threading import Lock

import math
import sys

JOINT_DELTA = 5*(math.pi/180.0)
VELOCITY_CMD = 0.2
JOINT_LIMIT = 62.0*(math.pi/180.0)

class BataPathController:

  def __init__(self, motor_count, joint_counts):

    # Set up member variables
    self.motor_count_ = motor_count
    self.joint_counts_ = joint_counts
    self.device_count_ = 0
    for i in range(self.motor_count_):
      self.device_count_ += 1+self.joint_counts_[i]

    self.cmd_pub_ = rospy.Publisher("cmd_trajectory", BataCmd, queue_size=1)

    # Setup comms
    self.robot_state_lock_ = Lock()
    self.cur_robot_state_msg_ = None
    self.robot_state_sub_ = rospy.Subscriber('joint_states', 
                                             JointState,
                                             self.robot_state_cb)
                                            
    self.as_ = actionlib.SimpleActionServer('bata_path_exec', 
                                            BataPathAction,
                                            execute_cb = self.execute_cb,
                                            auto_start = False)
    print 'Starting server'
    self.as_.start()
    print 'Server started'

  def robot_state_cb(self, msg):
    # Only update if the current message isn't being examined right now
    if(self.robot_state_lock_.acquire(False)):
      # Check that message is valid
      state_idx = 0
      msg_ok = True
      if len(msg.name) != self.device_count_:
        print('Expected %d joints, recieved %d'%(self.device_count_, len(msg.name)))
        msg_ok = False
      else:
        for i in range(self.motor_count_):
          if(msg.name[state_idx] != 'm'+str(i)):
            print('Expected joint %d to have name m%d'%(state_idx,i))
            msg_ok = False
            break
          state_idx += 1
          for j in range(self.joint_counts_[i]):
            if(msg.name[state_idx] != 'm'+str(i)+'_j'+str(j)):
              print('Expected joint %d to have name m%d_j%d'%(state_idx,i,j))
              msg_ok = False
              break
            state_idx += 1
      # Update the message
      if msg_ok:
        self.cur_robot_state_msg_ = msg
      self.robot_state_lock_.release()  

  def execute_cb(self, goal):
    # Setup outgoing command
    cmd_msg = BataCmd()
    for i in range(self.motor_count_):
      chain_cmd_msg = BataChainCmd()
      chain_cmd_msg.motor_mode = 1 # Velocity mode
      chain_cmd_msg.motor_cmd = 0.0
      chain_cmd_msg.enable_brake = [False]*joint_counts[i]
      cmd_msg.chain_cmds.append(chain_cmd_msg)

    # Setup action responses
    as_feedback = BataPathFeedback()
    as_result = BataPathResult()
    for i in range(len(goal.path.points)):
      as_feedback.waypoint_success.append(False)

    # Variables for keeping track of robot state and goals
    robot_state = []
    robot_setpoints = []
    robot_joint_names = []
    for i in range(self.motor_count_):
      robot_state.append([0.0]*self.joint_counts_[i])
      robot_setpoints.append([None]*self.joint_counts_[i]) 
      robot_joint_names.append([])
      for j in range(self.joint_counts_[i]):
        robot_joint_names[i].append('m'+str(i)+'_j'+str(j))      

    goal_idx = -1
    while True:
      
      # Check for pre-empt
      if self.as_.is_preempt_requested():
        print('Pre-empted, abort')
        self.as_.set_preempted()
        goal_idx = len(goal.path.points) # Indicates done

      # Get latest robot state
      while self.cur_robot_state_msg_ is None:
        pass
      self.robot_state_lock_.acquire()
      state_idx = 0
      for i in range(self.motor_count_):
        state_idx += 1
        for j in range(self.joint_counts_[i]):
          robot_state[i][j] = self.cur_robot_state_msg_.position[state_idx]
          state_idx += 1

      self.cur_robot_state_msg_ = None
      self.robot_state_lock_.release()

      # Check if any additional joints have reached their goal
      for i in range(self.motor_count_):
        for j in range(self.joint_counts_[i]):
          if(robot_setpoints[i][j] is not None and
             not cmd_msg.chain_cmds[i].enable_brake[j] and
             ((cmd_msg.chain_cmds[i].motor_cmd > sys.float_info.epsilon
               and robot_state[i][j] > robot_setpoints[i][j]) or 
               (cmd_msg.chain_cmds[i].motor_cmd < -sys.float_info.epsilon
               and robot_state[i][j] < robot_setpoints[i][j]))):
            robot_setpoints[i][j] = None
            cmd_msg.chain_cmds[i].enable_brake[j] = True
      print 'Goal idx: ' + str(goal_idx)
      print 'Goals: '+str(robot_setpoints)
      print 'State: '+str(robot_state)

      while True:
        # Check if ready for next goal
        goalComplete = True
        for i in range(self.motor_count_):
          chainGoalComplete = True
          for j in range(self.joint_counts_[i]):
            if robot_setpoints[i][j] is not None:
              chainGoalComplete = False
              goalComplete = False
              break
          if chainGoalComplete:
            cmd_msg.chain_cmds[i].motor_cmd = 0.0
            for j in range(self.joint_counts_[i]):
              cmd_msg.chain_cmds[i].enable_brake[j] = False

        if goalComplete:
          # Publish feedback
          if goal_idx >= 0:
            as_feedback.waypoint_success[goal_idx] = True
            self.as_.publish_feedback(as_feedback)

          goal_idx += 1
          # Check if completely done
          if goal_idx >= len(goal.path.points):
            as_result.waypoint_success = as_feedback.waypoint_success
            self.as_.set_succeeded(as_result)
            break
          else:
            # Check if next goal is valid
            for i in range(len(goal.path.points[goal_idx].positions)):
              if (abs(goal.path.points[goal_idx].positions[i]) > JOINT_LIMIT):
                print('Out of bounds goal: %f'%(goal.path.points[goal_idx].positions[i]))
                as_result.waypoint_success = as_feedback.waypoint_success
                self.as_.set_succeeded(as_result)
                goal_idx = len(goal.path.points) # Indicates done
                break

            if(goal_idx >= len(goal.path.points)):
              break

            goal_match = 0
            for i in range(self.motor_count_):
              for j in range(self.joint_counts_[i]):
                # Figure out which joint goal corresponds to joint (i,j)
                for k in range(len(goal.path.joint_names)):
                  if (robot_joint_names[i][j] == goal.path.joint_names[k]):
                    if(abs(robot_state[i][j] - goal.path.points[goal_idx].positions[k]) > JOINT_DELTA):
                      # Register goal
                      robot_setpoints[i][j] = goal.path.points[goal_idx].positions[k]
                    else:
                      # Joint is already close enough so turn on brake, don't register goal
                      cmd_msg.chain_cmds[i].enable_brake[j] = True
                    goal_match += 1
            # Validate that all joint goals correspond to actual joints
            if goal_match != len(goal.path.joint_names):
              print('Not all goal joint names matched with existing joints, abort')
              as_result.waypoint_success = as_feedback.waypoint_success
              self.as_.set_succeeded(as_result)
              goal_idx = len(goal.path.points) # Indicates done
              break
        else:
          # Found an unfinished goal
          break            
            
      # Get joint votes
      pos_votes = [0] * self.motor_count_
      neg_votes = [0] * self.motor_count_
      for i in range(self.motor_count_):
        for j in range(self.joint_counts_[i]):
          # Only joints that have a goal and are not braked get to vote
          if(robot_setpoints[i][j] is not None and
             not cmd_msg.chain_cmds[i].enable_brake[j]):
            if(robot_setpoints[i][j] > robot_state[i][j]):
              pos_votes[i] += 1
            else:
              neg_votes[i] += 1
      
      # Determine motor directions and brakes
      for i in range(self.motor_count_):
        if abs(cmd_msg.chain_cmds[i].motor_cmd) < sys.float_info.epsilon:
          assert(pos_votes > 0 or neg_votes > 0) # At least one joint should be unbreaked, have a goal
          if pos_votes[i] > neg_votes[i]:
            cmd_msg.chain_cmds[i].motor_cmd = VELOCITY_CMD
          else:
            cmd_msg.chain_cmds[i].motor_cmd = -VELOCITY_CMD
          # Brake any joints that lost the vote
          for j in range(self.joint_counts_[i]):
            if(robot_setpoints[i][j] is not None):
              pos_error = robot_setpoints[i][j]-robot_state[i][j]
              if(cmd_msg.chain_cmds[i].motor_cmd*pos_error < 0.0):
                cmd_msg.chain_cmds[i].enable_brake[j] = True
        elif(pos_votes[i] == 0 and neg_votes[i] == 0):
          # No votes because all joints have either reached goal or are braked
          # Time to switch motor direction
          cmd_msg.chain_cmds[i].motor_cmd *= -1
          # Unbrake any joints that still have an unreached goal
          for j in range(self.joint_counts_[i]):
            if robot_setpoints[i][j] is not None:
              cmd_msg.chain_cmds[i].enable_brake[j] = False
        else:
          # Continue going in the current direction (change nothing)
          pass
            
      if goal_idx >= 0 and goal_idx < len(goal.path.points):
        # Send the updated command
        self.cmd_pub_.publish(cmd_msg)
      else:
        for i in range(motor_count):
          cmd_msg.chain_cmds[i].motor_mode = 1
          cmd_msg.chain_cmds[i].motor_cmd = 0.0
          for j in range(joint_counts[i]):
            cmd_msg.chain_cmds[i].enable_brake[j] = False      
        self.cmd_pub_.publish(cmd_msg)
        return

if __name__ == '__main__':
  rospy.init_node('bata_path_controller')
  motor_count = rospy.get_param('motor_count')
  joint_counts = rospy.get_param('joint_counts')
  server = BataPathController(motor_count, joint_counts)
  rospy.spin()
