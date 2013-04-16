/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <velocity_controllers/joint_setpoint_controller.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include "pluginlib/class_list_macros.h"
#include <iostream>
#include <math.h>
using namespace std;

namespace velocity_controllers {

JointSetpointController::JointSetpointController(){}
JointSetpointController::~JointSetpointController()
{
  joint_position_command_sub_.shutdown();
  // joint_command_subscriber_.shutdown();
}

bool JointSetpointController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  ROS_INFO_STREAM("JointSetpointController: Starting Up... ");
  // Get Node
  node_ = n;
  
  // Get ROOT and TIP Parameters
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("CartesianSetpointController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("CartesianSetpointController: Root link is " << root_name_);

  if (!node_.getParam("tip_name", tip_name_)){
    ROS_ERROR("CartesianSetpointController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("CartesianSetpointController: Tip link is " << tip_name_);

  // Check hardware interface is valid
  assert(robot);
  robot_ = robot;

  // Get the robot description file
  if (!node_.getParam("/adjutant/device/merlin/robot_description", robot_desc_string_)){
    ROS_ERROR("JointSetpointController: No robot description found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("JointSetpointController: Robot Description is " << robot_desc_string_);

  // Load URDF for robot
  urdf::Model urdf;
  if (!urdf.initParam("/adjutant/device/merlin/robot_description")){
    ROS_ERROR("JointSetpointController: No URDF file found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // Initialize a KDL Tree
  if (!kdl_parser::treeFromString(robot_desc_string_, kdl_tree_)){
    ROS_ERROR("JointSetpointController: Failed to construct KDL Tree");
    return false;
  }
  // Extract KDL Chain from Tree
  if(!kdl_tree_.getChain(root_name_,tip_name_,kdl_chain_)){
    ROS_ERROR("JointSetpointController: Failed to extract KDL Chain from KDL Tree");
    return false;
  }
  ROS_INFO_STREAM("JointSetpointController: KDL found Chain of "<<kdl_chain_.getNrOfJoints()<< " joints");

  // Number of joints
  num_joints_ = kdl_chain_.getNrOfJoints();

  // Get all joint and link names from chain
  chain_joint_names_.resize(kdl_chain_.getNrOfJoints());
  chain_link_names_.resize(kdl_chain_.getNrOfJoints());
  for (unsigned i=0; i<num_joints_; i++){
    chain_joint_names_[i] = kdl_chain_.getSegment(i).getJoint().getName(); 
    chain_link_names_[i] = kdl_chain_.getSegment(i).getName(); 
  }
  ROS_INFO_STREAM("JointSetpointController: Found Links in Chain = "
            << chain_link_names_[0] <<"  "
            << chain_link_names_[1] <<"  "
            << chain_link_names_[2] <<"  "
            << chain_link_names_[3] <<"  "
            << chain_link_names_[4] <<"  "
            << chain_link_names_[5]);
  ROS_INFO_STREAM("JointSetpointController: Found Joints in Chain = "
            << chain_joint_names_[0] <<"  "
            << chain_joint_names_[1] <<"  "
            << chain_joint_names_[2] <<"  "
            << chain_joint_names_[3] <<"  "
            << chain_joint_names_[4] <<"  "
            << chain_joint_names_[5]);

  // Get all joint names from the hardware interface
  hw_joint_names_ = robot->getJointNames();
  ROS_INFO_STREAM("JointSetpointController: Found Joints in Hardware Interface = "
              << hw_joint_names_[0] <<"  "
              << hw_joint_names_[1] <<"  "
              << hw_joint_names_[2] <<"  "
              << hw_joint_names_[3] <<"  "
              << hw_joint_names_[4] <<"  "
              << hw_joint_names_[5]);
  if(hw_joint_names_.size() != num_joints_)
    ROS_WARN_STREAM("JointSetpointController: Number of Joints in kinematic chain does not agree with robot hardware interface");

  // Initialize Joint Values
  joint_positions_.resize(num_joints_);
  joint_positions_desired_.resize(num_joints_);
    for(unsigned int i = 0;i<num_joints_;i++)
      joint_positions_desired_(i) = 0;
  joint_velocities_.resize(num_joints_);
  joint_accelerations_.resize(num_joints_);
  joint_velocities_desired_.resize(num_joints_);
  joint_velocities_command_.resize(num_joints_);
  joint_positions_upper_limits_.resize(num_joints_);
  joint_positions_lower_limits_.resize(num_joints_);

  // Create PID Controller
  control_toolbox::Pid pid_controller;
  for (unsigned int i = 0; i < num_joints_; i++){
   if (!pid_controller.init(ros::NodeHandle(node_,"pid_gains/" + chain_joint_names_[i]))){
      ROS_WARN_STREAM("JointSetpointController: No PID Gains found on parameter server for joint "<<chain_joint_names_[i]);
      return false;
    }else{
      pid_controller_.push_back(pid_controller);
      double p,i_val,d,i_max,i_min;
      pid_controller.getGains(p, i_val, d, i_max, i_min);
     ROS_INFO_STREAM("JointSetpointController: PID Gains for joint " <<chain_joint_names_[i]<<" --> p: "<<p<<", i: "<<i_val<<", d: "<<d);
    }
    
  }
 
  // Get URDF for joints
  for (unsigned i=0; i<num_joints_; i++){
    joint_urdf_.push_back( urdf.getJoint(chain_joint_names_[i]) );
    if(!joint_urdf_[i]){
      ROS_ERROR("Could not find joint %s in urdf", chain_joint_names_[i].c_str());
      return false;
    }
  }

  // Get Joint Limits
  for (unsigned i=0; i<num_joints_; i++){
    // Velocity Limit
    joint_velocity_limits_.push_back(joint_urdf_[i]->limits->velocity);
    // Upper Position Limit
    joint_upper_position_limits_.push_back(joint_urdf_[i]->limits->upper);
    joint_positions_upper_limits_(i) = joint_urdf_[i]->limits->upper;
    // Lower Position Limit
    joint_lower_position_limits_.push_back(joint_urdf_[i]->limits->lower);
    joint_positions_lower_limits_(i) = joint_urdf_[i]->limits->lower;
  }
  // Get acceleration limits from parameter server
  joint_acceleration_limits_.resize(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i){
    if (!node_.getParam("acceleration_limits/" + chain_joint_names_[i], joint_acceleration_limits_[i])){
      ROS_WARN_STREAM("JointSetpointController: No acceleration limit found on parameter server for joint "<<chain_joint_names_[i]<<", setting to default (0.0)");
      joint_acceleration_limits_[i] = 0.0;
    }
  }
  ROS_WARN_STREAM("JointSetpointController: Acceleration limits = "
            << joint_acceleration_limits_[0] <<"  "
            << joint_acceleration_limits_[1] <<"  "
            << joint_acceleration_limits_[2] <<"  "
            << joint_acceleration_limits_[3] <<"  "
            << joint_acceleration_limits_[4] <<"  "
            << joint_acceleration_limits_[5]);

  // Get all joint handles
  for (unsigned i=0; i<chain_joint_names_.size(); i++){
    joint_handles_.push_back( robot->getJointHandle(chain_joint_names_[i]) );
  }

  // Subscribe to pose commands
  joint_position_command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("input/command/joint_pos", 1, &JointSetpointController::commandCB_joint, this);
  return true;
}

void JointSetpointController::starting(const ros::Time& time)
{
  getRobotInfo();  
  last_time_ = time;
  loop_count_ = 0;

  // reset pid controllers
  for (unsigned int i=0; i<num_joints_; i++)
    pid_controller_[i].reset();

  ROS_INFO_STREAM("JointSetpointController: Initial Joint Position = "
              << joint_positions_(0) <<"  "
              << joint_positions_(1) <<"  "
              << joint_positions_(2) <<"  "
              << joint_positions_(3) <<"  "
              << joint_positions_(4) <<"  "
              << joint_positions_(5)); 
  ROS_INFO_STREAM("JointSetpointController: Initial Joint Velocity = "
              << joint_velocities_(0) <<"  "
              << joint_velocities_(1) <<"  "
              << joint_velocities_(2) <<"  "
              << joint_velocities_(3) <<"  "
              << joint_velocities_(4) <<"  "
              << joint_velocities_(5)); 
}

bool JointSetpointController::getRobotInfo()
{
  // Get the joint positions and velocities
  for (unsigned i=0; i<num_joints_; i++){
      joint_positions_(i) = joint_handles_[i].getPosition();
      joint_velocities_(i) = joint_handles_[i].getVelocity();
  }
  return true;
}

void JointSetpointController::commandCB_joint(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("JointSetpointController: Pose Command Recieved");
  for (unsigned int i = 0; i < num_joints_; ++i){
    joint_positions_desired_(i) = msg->data[i];
  }
}

void JointSetpointController::update(const ros::Time& time, const ros::Duration& period)
{
  // Get time
  ros::Duration dt = period;
  last_time_ = time;
  // Get last pose
  getRobotInfo();

  // Calculate the position error
  KDL::JntArray joint_positions_error;
  KDL::Subtract(joint_positions_,joint_positions_desired_,joint_positions_error);

  // For each joint, calculate the required velocity to move to new position
  for(unsigned int i=0;i<num_joints_;i++){
    // Perform PID Update of Velocity
    joint_velocities_command_(i) = pid_controller_[i].updatePid(joint_positions_error(i), dt);
    // Calculate Instantaneous Acceleration
    joint_accelerations_(i) = joint_velocities_command_(i) - joint_velocities_(i);
  }

  // Check acceleration with acceleration limits, and override if necessary
  for(unsigned int i=0;i<num_joints_;i++){
    if(fabs(joint_accelerations_(i)) > joint_acceleration_limits_[i]){
      if(joint_accelerations_(i) > 0)
        joint_velocities_command_(i) = joint_velocities_(i) + joint_acceleration_limits_[i];
      else
        joint_velocities_command_(i) = joint_velocities_(i) - joint_acceleration_limits_[i];
    }
  }
  for(unsigned int i=0;i<num_joints_;i++){
    // Get Joint Handle
    hardware_interface::JointHandle joint = joint_handles_[i];
    // Get current joint's velocity limits
    double vel_limit = joint_velocity_limits_[i];
    // Check command velocity agains limits
    if(joint_velocities_command_(i) > vel_limit){
      joint_velocities_command_(i) = vel_limit;
    }else if(joint_velocities_command_(i) < -vel_limit){
      joint_velocities_command_(i) = -vel_limit;
    }
    // Set velocity command to current joint
    joint.setCommand(joint_velocities_command_(i));
  }
}

void JointSetpointController::stopping(const ros::Time& time)
{
  ROS_INFO_STREAM("Shutting Down Controller and Commanding Zero Velocity.");
  // Set all velocities to zero.
  for(unsigned int i=0;i<num_joints_;i++){
    hardware_interface::JointHandle joint = joint_handles_[i];
    joint.setCommand(0.0);
  }
  ROS_INFO_STREAM("Controller Stopped Successfully.");
}

}// namespace

PLUGINLIB_DECLARE_CLASS(velocity_controllers, JointSetpointController, velocity_controllers::JointSetpointController, controller_interface::ControllerBase)

