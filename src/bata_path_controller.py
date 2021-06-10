#!/usr/bin/env python

import rospy
import actionlib
from bata_base.msg import BataPathFeedback, BataPathResult, BataPathAction
from bata_base.msg import BataCmd, BataChainCmd
from sensor_msgs.msg import JointState
from threading import Lock

import math
import sys

#lock.acquire(), lock.release()

JOINT_DELTA = 5*(math.pi/180.0)
VELOCITY_CMD = 0.2
JOINT_LIMIT = 55.0*(math.pi/180.0)

class BataPathController:

  def __init__(self, motor_count, joint_counts):

    self.motor_count_ = motor_count
    self.joint_counts_ = joint_counts
    self.device_count_ = 0
    self.cmd_msg_ = BataCmd()
    for i in range(motor_count):
      chain_cmd_msg = BataChainCmd()
      chain_cmd_msg.motor_mode = 1
      chain_cmd_msg.motor_cmd = 0.0
      self.device_count_ += 1+self.joint_counts_[i]
      for j in range(joint_counts[i]):
        chain_cmd_msg.enable_brake = False
      self.cmd_msg_.chain_cmds.append(chain_cmd_msg)

    self.cmd_pub_ = rospy.Publisher("cmd_trajectory", BataCmd, queue_size=1)

    self.cmd_pub_.publish(self.cmd_msg_)

    self.robot_state_lock_ = Lock()
    self.cur_robot_state_msg_ = None
    self.robot_state_sub_ = rospy.Subscriber('joint_states', 
                                             JointState,
                                             self.robot_state_cb)
                                            
    self.as_ = actionlib.SimpleActionServer('bata_path_exec', 
                                            BataPathAction,
                                            execute_cb = self.execute_cb,
                                            auto_start = False)
    self.as_.start()

  def robot_state_cb(self, msg):
    if(self.robot_state_lock.acquire(blocking=False)):
      state_idx = 0
      bool msg_ok = True
      if len(msg.name) != self.device_count_:
        print('Expected %d joints, recieved %d'%(self.device_count_, len(msg.name)))
        msg_ok = False
      else:
        for i in range(self.motor_count):
          if(msg.name[state_idx] != 'm'+str(i)):
            print('Expected joint %d to have name m%d'%(state_idx,i)
            msg_ok = False
            break
          state_idx += 1
          for j in range(self.joint_counts[i]):
            if(msg.name[state_idx] != 'm'+str(i)+'_j'+str(j)):
              print('Expected joint %d to have name m%d_j%d'%(state_idx,i,j))
              msg_ok = False
              break
            state_idx += 1

      if msg_ok:
        self.cur_robot_state_msg = msg
      self.robot_state_lock.release()  

  def execute_cb(self, goal):
    as_feedback = BataPathFeedback()
    as_result = BataPathResult()
    for i in range(len(goal.path.points)):
      as_feedback.waypoint_success.append(False)

    goal_idx = -1
    robot_state = []
    robot_setpoints = []
    for i in range(self.motor_count_):
      robot_state.append([0.0]*self.joint_counts_[i])
      robot_setpoints.append([None]*self.joint_counts_[i])       

    robot_joint_names = []
    while self.cur_robot_state_msg_ is None:
      pass
    self.robot_state_lock.acquire()
    state_idx = 0
    for i in range(self.motor_count_):
      state_idx += 1
      robot_joint_names.append([])
      for j in range(self.joint_counts_[i]):
        robot_joint_names[i].append(self.cur_robot_state_msg.name[state_idx])
        state_idx += 1
    self.cur_robot_state_msg_ = None
    self.robot_state_lock.release()

    while True:
      
      # Check for pre-empt
      if self.as_.is_preempt_requested():
        print('Pre-empted, abort')
        self.as_.set_preempted()
        break

      # Get latest robot state
      while self.cur_robot_state_msg_ is None:
        pass
      self.robot_state_lock.acquire()
      state_idx = 0
      for i in range(self.motor_count_):
        state_idx += 1
        for j in range(self.joint_counts_[i]):
          robot_state[i][j] = self.cur_robot_state_msg.position[state_idx]
          state_idx += 1

      self.cur_robot_state_msg_ = None
      self.robot_state_lock.release()

      # Check if any additional joints have reached their goal
      for i in range(self.motor_count_):
        for j in range(self.joint_counts_[i]):
          if(robot_setpoints[i][j] is not None and
             not self.cmd_msg_.chain_cmds[i].enable_brake[j] and
             ((self.cmd_msg_.chain_cmds[i].motor_cmd > sys.float_info.epsilon
               and robot_state[i][j] > robot_setpoints[i][j]) or 
               (self.cmd_msg_.chain_cmds[i].motor_cmd < sys.float_info.epsilon
               and robot_state[i][j] < robot_setpoints[i][j]))):
            robot_setpoints[i][j] = None
            self.cmd_msg_.chain_cmds[i].enable_brake[j] = True

      # Check if ready for next goal
      while True:
        goalComplete = True
        for i in range(self.motor_count_):
          chainGoalComplete = True
          for j in range(self.joint_counts_[i]):
            if robot_setpoints[i][j] is not None:
              chainGoalComplete = False
              goalComplete = False
              break
          if chainGoalComplete:
            self.cmd_msg_.chain_cmds[i].motor_cmd = 0.0
            for j in range(self.joint_counts_[i]):
              self.cmd_msg_.chain_cmds[i].enable_brake[j] = False

        if goalComplete:
          if goal_idx >= 0:
            as_feedback.waypoint_success[goal_idx] = True
            self.as_.publish_feedback(as_feedback)

          goal_idx += 1
          # Check if done
          if goal_idx >= len(goal.path.points):
            self.as_.set_succeeded(as_feedback)
            return
          else:
            # Setup next goal
            for i in range(len(goal.path.points[goal_idx].positions)):
              if (abs(goal.path.points[goal_idx].positions[goal_idx]) > JOINT_LIMIT):
                print('Out of bounds goal: %f'%(goal.path.points[goal_idx].positions[goal_idx]))
                self.as_.set_succeeded(as_feedback)
                return  

            goal_match = 0
            for i in range(self.motor_count_):
              for j in range(self.joint_counts_[i]):
                for k in range(len(goal.path.joint_names)):
                  if (robot_joint_names[i][j] == goal.path.joint_names[k]):
                    if(abs(robot_state[i][j] - goal.path.points[goal_idx].positions[k]) > JOINT_DELTA):
                      robot_setpoints[i][j] = goal.path.points[goal_idx].positions[k]
                    else:
                      self.cmd_msg_.chain_cmds[i].enable_brake[j] = True
                    goal_match += 1
            if goal_match != len(goal.path.joint_names):
              print('Not all goal joint names matched with existing joints, abort')
              self.as_.set_succeeded(as_feedback)
              return
        else:
          # Found an unfinished goal
          break            
            
      # Get joint votes
      pos_votes = [0] * self.motor_count_
      neg_votes = [0] * self.motor_count_
      for i in range(self.motor_count_):
        for j in range(self.joint_counts_[i]):
          if(robot_setpoints[i][j] is not None and
             not self.cmd_msg_.chain_cmds[i].enable_brake[j]):
            if(robot_setpoints[i][j] > robot_state[i][j]):
              pos_votes[i] += 1
            else:
              neg_votes[i] += 1
      
      # Determine motor directions and brakes
      for i in range(self.motor_count_):
        if abs(self.cmd_msg_.chain_cmds[i].motor_cmd) < sys.float_info.epsilon:
          assert(pos_votes > 0 or neg_votes > 0)
          if pos_votes[i] > neg_votes[i]:
            self.cmd_msg_.chain_cmds[i].motor_cmd = VELOCITY_CMD
          else:
            self.cmd_msg_.chain_cmds[i].motor_cmd = -VELOCITY_CMD
          for j in range(self.joint_counts_[i]):
            pos_error = robot_setpoints[i][j]-robot_state[i][j]
            if(self.robot_setpoints[i][j] is not None and 
               self.cmd_msg_.chain_cmds[i].motor_cmd*pos_error < 0.0):
              self.cmd_msg_.chain_cmds[i].enable_brake[j] = True
        elif(pos_votes[i] == 0 and neg_votes[i] == 0):
          self.cmd_msg_.chain_cmds[i].motor_cmd *= -1
          for j in range(self.joint_counts_[i]):
            if robot_setpoints[i][j] is not None:
              self.cmd_msg_.chain_cmds[i].enable_brake[j] = False
        else:
          # Continue going in the current direction (change nothing)
          pass
            
      self.cmd_pub_.publish(self.cmd_msg_)      


