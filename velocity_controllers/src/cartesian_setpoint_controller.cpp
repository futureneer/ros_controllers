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

#include <velocity_controllers/cartesian_position_controller.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include "pluginlib/class_list_macros.h"
#include "tf_conversions/tf_kdl.h"
#include <iostream>
#include <math.h>
using namespace std;

namespace velocity_controllers {

CartesianSetpointController::CartesianSetpointController(){}
CartesianSetpointController::~CartesianSetpointController()
{
  cartesian_command_subscriber_.shutdown();
  // joint_command_subscriber_.shutdown();
}

bool CartesianSetpointController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  ROS_INFO_STREAM("CartesianSetpointController: Starting Up... ");
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
  if (!node_.getParam("/robot_description", robot_desc_string_)){
    ROS_ERROR("CartesianSetpointController: No robot description found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("CartesianSetpointController: Robot Description is " << robot_desc_string_);

  // Load URDF for robot
  urdf::Model urdf;
  if (!urdf.initParam("/robot_description")){
    ROS_ERROR("CartesianSetpointController: No URDF file found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // Initialize a KDL Tree
  if (!kdl_parser::treeFromString(robot_desc_string_, kdl_tree_)){
    ROS_ERROR("CartesianSetpointController: Failed to construct KDL Tree");
    return false;
  }
  // Extract KDL Chain from Tree
  if(!kdl_tree_.getChain(root_name_,tip_name_,kdl_chain_)){
    ROS_ERROR("CartesianSetpointController: Failed to extract KDL Chain from KDL Tree");
    return false;
  }
  ROS_INFO_STREAM("CartesianSetpointController: KDL found Chain of "<<kdl_chain_.getNrOfJoints()<< " joints");

  // Number of joints
  num_joints_ = kdl_chain_.getNrOfJoints();

  // Get all joint and link names from chain
  chain_joint_names_.resize(kdl_chain_.getNrOfJoints());
  chain_link_names_.resize(kdl_chain_.getNrOfJoints());
  for (unsigned i=0; i<num_joints_; i++){
    chain_joint_names_[i] = kdl_chain_.getSegment(i).getJoint().getName(); 
    chain_link_names_[i] = kdl_chain_.getSegment(i).getName(); 
  }
  ROS_INFO_STREAM("CartesianSetpointController: Found Links in Chain = "
            << chain_link_names_[0] <<"  "
            << chain_link_names_[1] <<"  "
            << chain_link_names_[2] <<"  "
            << chain_link_names_[3] <<"  "
            << chain_link_names_[4] <<"  "
            << chain_link_names_[5]);
  ROS_INFO_STREAM("CartesianSetpointController: Found Joints in Chain = "
            << chain_joint_names_[0] <<"  "
            << chain_joint_names_[1] <<"  "
            << chain_joint_names_[2] <<"  "
            << chain_joint_names_[3] <<"  "
            << chain_joint_names_[4] <<"  "
            << chain_joint_names_[5]);

  // Get all joint names from the hardware interface
  hw_joint_names_ = robot->getJointNames();
  ROS_INFO_STREAM("CartesianSetpointController: Found Joints in Hardware Interface = "
              << hw_joint_names_[0] <<"  "
              << hw_joint_names_[1] <<"  "
              << hw_joint_names_[2] <<"  "
              << hw_joint_names_[3] <<"  "
              << hw_joint_names_[4] <<"  "
              << hw_joint_names_[5]);
  if(hw_joint_names_.size() != num_joints_)
    ROS_WARN_STREAM("CartesianSetpointController: Number of Joints in kinematic chain does not agree with robot hardware interface");

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
  if (!pid_controller.init(ros::NodeHandle(node_,"pid_gains"))) return false;
  for (unsigned int i = 0; i < num_joints_; i++)
    pid_controller_.push_back(pid_controller);
 
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
  for (int i = 0; i < num_joints_; ++i){
    if (!node_.getParam("acceleration_limits/" + chain_joint_names_[i], joint_acceleration_limits_[i])){
      ROS_WARN_STREAM("CartesianSetpointController: No acceleration limit found on parameter server for joint "<<chain_joint_names_[i]<<", setting to default (0.0)");
      joint_acceleration_limits_[i] = 0.0;
    }
  }
  ROS_WARN_STREAM("CartesianSetpointController: Acceleration limits = "
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

  // Create Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  ik_limit_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_positions_upper_limits_,joint_positions_lower_limits_,*fk_solver_.get(), *ik_vel_solver_.get()));
  ik_solver_.reset(new KDL::ChainIkSolverPos_NR(kdl_chain_,*fk_solver_.get(), *ik_vel_solver_.get()));

  // Subscribe to pose commands
  cartesian_command_subscriber_ = n.subscribe<geometry_msgs::PoseStamped>("cartesian_pose_command", 1, &CartesianSetpointController::commandCB_cartesian, this);
  // joint_command_subscriber_ = n.subscribe<std_msgs::Float64MultiArray>("joint_position_command", 1, &CartesianSetpointController::commandCB_joint, this);
  return true;
}

void CartesianSetpointController::starting(const ros::Time& time)
{
  joint_command_lock_ = false;
  pose_desired_ = getPose();  
  last_time_ = time;
  loop_count_ = 0;

  // reset pid controllers
  for (unsigned int i=0; i<num_joints_; i++)
    pid_controller_[i].reset();

  ROS_INFO_STREAM("CartesianSetpointController: Initial Cartesian Position = "
              << pose_desired_.p.x() <<"  "
              << pose_desired_.p.y() <<"  "
              << pose_desired_.p.z());

  double roll, pitch, yaw;
  pose_desired_.M.GetRPY(roll,pitch,yaw);
  ROS_INFO_STREAM("CartesianSetpointController: Initial Cartesian Rotation = roll: " << roll <<" pitch: " << pitch <<" yaw: " << yaw);

  ROS_INFO_STREAM("CartesianSetpointController: Initial Position = "
              << joint_positions_(0) <<"  "
              << joint_positions_(1) <<"  "
              << joint_positions_(2) <<"  "
              << joint_positions_(3) <<"  "
              << joint_positions_(4) <<"  "
              << joint_positions_(5)); 
  ROS_INFO_STREAM("CartesianSetpointController: Initial Velocity = "
              << joint_velocities_(0) <<"  "
              << joint_velocities_(1) <<"  "
              << joint_velocities_(2) <<"  "
              << joint_velocities_(3) <<"  "
              << joint_velocities_(4) <<"  "
              << joint_velocities_(5)); 
}

KDL::Frame CartesianSetpointController::getPose()
{
  // Get the joint positions and velocities
  for (unsigned i=0; i<num_joints_; i++){
      joint_positions_(i) = joint_handles_[i].getPosition();
      joint_velocities_(i) = joint_handles_[i].getVelocity();
  }

  // get cartesian pose
  KDL::Frame result;
  fk_solver_->JntToCart(joint_positions_, result);

  return result;
}

void CartesianSetpointController::commandCB_cartesian(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  ROS_WARN_STREAM("CartesianSetpointController: Pose Command Recieved");
  // convert message to transform
  tf::Stamped<tf::Pose> pose_stamped;
  poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  // tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  tf::PoseTFToKDL(pose_stamped, pose_desired_);
  joint_command_lock_ = true;
}

// void CartesianSetpointController::commandCB_joint(const std_msgs::Float64MultiArrayConstPtr& msg)
// {
//   ROS_WARN_STREAM("CartesianSetpointController: Joint Command Recieved");
//   std::vector<double> command;
//   command = msg->data;
//   for (unsigned i=0; i<num_joints_; i++){
//     joint_positions_desired_(i) = command[i];
//   }
// }

void CartesianSetpointController::update(const ros::Time& time, const ros::Duration& period)
{
  // Get time
  ros::Duration dt = period;
  last_time_ = time;
  // Get last pose
  pose_measured_ = getPose();


  if(joint_command_lock_ == true){
    ROS_DEBUG_STREAM("CartesianSetpointController: Desired Cartesian Position = "
                << pose_desired_.p.x() <<"  "
                << pose_desired_.p.y() <<"  "
                << pose_desired_.p.z());

    // Calculate desired joint positions from desired cartesian pos
    int e = ik_solver_->CartToJnt(joint_positions_,pose_desired_,joint_positions_desired_);
    joint_command_lock_ = false;
  }

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

  ROS_WARN_STREAM("CartesianSetpointController: Desired Position = "
              << joint_positions_desired_(0) <<"  "
              << joint_positions_desired_(1) <<"  "
              << joint_positions_desired_(2) <<"  "
              << joint_positions_desired_(3) <<"  "
              << joint_positions_desired_(4) <<"  "
              << joint_positions_desired_(5)); 

  ROS_WARN_STREAM("CartesianSetpointController: Current Position = "
              << joint_positions_(0) <<"  "
              << joint_positions_(1) <<"  "
              << joint_positions_(2) <<"  "
              << joint_positions_(3) <<"  "
              << joint_positions_(4) <<"  "
              << joint_positions_(5)); 

  // Check acceleration with acceleration limits, and override if necessary
  for(unsigned int i=0;i<num_joints_;i++){
    if(fabs(joint_accelerations_(i)) > joint_acceleration_limits_[i]){
      if(joint_accelerations_(i) > 0)
        joint_velocities_command_(i) = joint_velocities_(i) + joint_acceleration_limits_[i];
      else
        joint_velocities_command_(i) = joint_velocities_(i) - joint_acceleration_limits_[i];
    }
  }

  // Assign velocity to each joint from command
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

void CartesianSetpointController::stopping(const ros::Time& time)
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

PLUGINLIB_DECLARE_CLASS(velocity_controllers, CartesianSetpointController, velocity_controllers::CartesianSetpointController, controller_interface::ControllerBase)

