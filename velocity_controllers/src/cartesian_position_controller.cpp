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
using namespace std;

namespace velocity_controllers {

CartesianPositionController::CartesianPositionController()
{}

CartesianPositionController::~CartesianPositionController()
{
  sub_command_.shutdown();
  // command_filter_.reset();
}

bool CartesianPositionController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  ROS_INFO_STREAM("CartesianPositionController: Starting Up... ");
  // Get Node
  node_ = n;
  // Get ROOT and TIP Parameters
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("CartesianPositionController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("CartesianPositionController: Root link is " << root_name_);

  if (!node_.getParam("tip_name", tip_name_)){
    ROS_ERROR("CartesianPositionController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("CartesianPositionController: Tip link is " << tip_name_);

  // Check hardware interface is valid
  assert(robot);
  robot_ = robot;

  // Get the robot description file
  if (!node_.getParam("/robot_description", robot_desc_string_)){
    ROS_ERROR("CartesianPositionController: No robot description found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO_STREAM("CartesianPositionController: Robot Description is " << robot_desc_string_);

  // Load URDF for robot
  urdf::Model urdf;
  if (!urdf.initParam("/robot_description")){
    ROS_ERROR("CartesianPositionController: No URDF file found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // Initialize a KDL Tree
  if (!kdl_parser::treeFromString(robot_desc_string_, kdl_tree_)){
    ROS_ERROR("CartesianPositionController: Failed to construct KDL Tree");
    return false;
  }
  // Extract KDL Chain from Tree
  if(!kdl_tree_.getChain(root_name_,tip_name_,kdl_chain_)){
    ROS_ERROR("CartesianPositionController: Failed to extract KDL Chain from KDL Tree");
    return false;
  }
  ROS_INFO_STREAM("CartesianPositionController: KDL Chain found "<<kdl_chain_.getNrOfJoints()<< " joints");

  // Get all joint states from the hardware interface
  joint_names_ = robot->getJointNames();
  num_joints_ = joint_names_.size();
  for (unsigned i=0; i<num_joints_; i++)
    ROS_DEBUG("Got joint %s", joint_names_[i].c_str());

  // Initialize Joint Values
  joint_positions_.resize(num_joints_);
  joint_positions_desired_.resize(num_joints_);
  joint_veolcities_.resize(num_joints_);
  joint_veolcities_desired_.resize(num_joints_);
  joint_veolcities_command_.resize(num_joints_);
  joint_positions_upper_limits_.resize(num_joints_);
  joint_positions_lower_limits_.resize(num_joints_);

  // Create PID Controller
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(ros::NodeHandle(node_,"pid_gains"))) return false;
  for (unsigned int i = 0; i < num_joints_; i++)
    pid_controller_.push_back(pid_controller);
 
  // Get URDF for joints
  for (unsigned i=0; i<num_joints_; i++){
    joint_urdf_.push_back( urdf.getJoint(joint_names_[i]) );
    if(!joint_urdf_[i]){
      ROS_ERROR("Could not find joint %s in urdf", joint_names_[i].c_str());
      return false;
    }
  }
  // Get Joint Limits
  for (unsigned i=0; i<num_joints_; i++){
    // Velocity Limit
    joint_vel_limits_.push_back(joint_urdf_[i]->limits->velocity);
    // Upper Position Limit
    joint_upper_position_limits_.push_back(joint_urdf_[i]->limits->upper);
    joint_positions_upper_limits_(i) = joint_urdf_[i]->limits->upper;
    // Lower Position Limit
    joint_lower_position_limits_.push_back(joint_urdf_[i]->limits->lower);
    joint_positions_lower_limits_(i) = joint_urdf_[i]->limits->lower;
  }

  // Get all joint handles
  for (unsigned i=0; i<joint_names_.size(); i++){
    joint_handles_.push_back( robot->getJointHandle(joint_names_[i]) );
  }

  // Create Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_positions_upper_limits_,joint_positions_lower_limits_,*fk_solver_.get(), *ik_vel_solver_.get()));

  // Subscribe to pose commands
  sub_command_ = n.subscribe<geometry_msgs::PoseStamped>("command", 1, &CartesianPositionController::commandCB, this);
  // sub_command_.subscribe(node_, "command", 10);
  // command_filter_.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(
  //                         sub_command_, tf_, root_name_, 10, node_));
  // command_filter_->registerCallback(boost::bind(&CartesianPositionController::command, this, _1));



  return true;
}

void CartesianPositionController::starting(const ros::Time& time)
{
  pose_desired_ = getPose();  
  last_time_ = time;
  loop_count_ = 0;

  // reset pid controllers
  for (unsigned int i=0; i<num_joints_; i++)
    pid_controller_[i].reset();


  ROS_INFO_STREAM("CartesianPositionController: Initial Cartesian Position = "
              << pose_desired_.p.x() <<"  "
              << pose_desired_.p.y() <<"  "
              << pose_desired_.p.z());

  ROS_INFO_STREAM("CartesianPositionController: Initial Position = "
              << joint_positions_(0) <<"  "
              << joint_positions_(1) <<"  "
              << joint_positions_(2) <<"  "
              << joint_positions_(3) <<"  "
              << joint_positions_(4) <<"  "
              << joint_positions_(5)); 
  ROS_INFO_STREAM("CartesianPositionController: Initial Velocity = "
              << joint_veolcities_(0) <<"  "
              << joint_veolcities_(1) <<"  "
              << joint_veolcities_(2) <<"  "
              << joint_veolcities_(3) <<"  "
              << joint_veolcities_(4) <<"  "
              << joint_veolcities_(5)); 
}

KDL::Frame CartesianPositionController::getPose()
{
  // Get the joint positions and velocities
  for (unsigned i=0; i<num_joints_; i++){
      joint_positions_(i) = joint_handles_[i].getPosition();
      joint_veolcities_(i) = joint_handles_[i].getVelocity();
  }

  // get cartesian pose
  KDL::Frame result;
  fk_solver_->JntToCart(joint_positions_, result);

  return result;
}

void CartesianPositionController::commandCB(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  // ROS_WARN_STREAM("CartesianPositionController: Recieving Command");
  // convert message to transform
  tf::Stamped<tf::Pose> pose_stamped;
  poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  // tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  tf::PoseTFToKDL(pose_stamped, pose_desired_);
}

void CartesianPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // Get time
  ros::Duration dt = period;
  last_time_ = time;
  // Get last pose
  pose_measured_ = getPose();
  
  // ROS_INFO_STREAM("CartesianPositionController: Desired Cartesian Position = "
  //             << pose_desired_.p.x() <<"  "
  //             << pose_desired_.p.y() <<"  "
  //             << pose_desired_.p.z());

  // Calculate desired joint positions from desired cartesian pos
  int e = ik_solver_->CartToJnt(joint_positions_,pose_desired_,joint_positions_desired_);
  // ROS_DEBUG_STREAM("CartesianPositionController: IK Returned "<<e);

  ROS_DEBUG_STREAM("CartesianPositionController: Current Position = "
              << joint_positions_(0) <<"  "
              << joint_positions_(1) <<"  "
              << joint_positions_(2) <<"  "
              << joint_positions_(3) <<"  "
              << joint_positions_(4) <<"  "
              << joint_positions_(5));

  ROS_DEBUG_STREAM("CartesianPositionController: Desired Joint Position = "
              << joint_positions_desired_(0) <<"  "
              << joint_positions_desired_(1) <<"  "
              << joint_positions_desired_(2) <<"  "
              << joint_positions_desired_(3) <<"  "
              << joint_positions_desired_(4) <<"  "
              << joint_positions_desired_(5));

  // Calculate the position error
  KDL::JntArray joint_positions_error;
  KDL::Subtract(joint_positions_,joint_positions_desired_,joint_positions_error);

  ROS_DEBUG_STREAM("CartesianPositionController: Position Error = "
              << joint_positions_error(0) <<"  "
              << joint_positions_error(1) <<"  "
              << joint_positions_error(2) <<"  "
              << joint_positions_error(3) <<"  "
              << joint_positions_error(4) <<"  "
              << joint_positions_error(5) <<" - dt: " <<dt);

  // For each joint, calculate the required velocity to move to new position
  for(unsigned int i=0;i<num_joints_;i++){
    joint_veolcities_command_(i) = pid_controller_[i].updatePid(joint_positions_error(i), dt);
  }

  // Assign velocity to each joint from command
  for(unsigned int i=0;i<num_joints_;i++){
    // Get Joint Handle
    hardware_interface::JointHandle joint = joint_handles_[i];
    // Get current joint's velocity limits
    double vel_limit = joint_vel_limits_[i];
    // Check command velocity agains limits
    if(joint_veolcities_command_(i) > vel_limit){
      ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<joint_veolcities_command_(i));
      joint_veolcities_command_(i) = vel_limit;
    }else if(joint_veolcities_command_(i) < -vel_limit){
      ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<joint_veolcities_command_(i));
      joint_veolcities_command_(i) = -vel_limit;
    }
    // Set velocity command to current joint
    joint.setCommand(joint_veolcities_command_(i));
  }
}

void CartesianPositionController::stopping(const ros::Time& time)
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

PLUGINLIB_DECLARE_CLASS(velocity_controllers, CartesianPositionController, velocity_controllers::CartesianPositionController, controller_interface::ControllerBase)

