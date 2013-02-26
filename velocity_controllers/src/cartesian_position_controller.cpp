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

namespace velocity_controllers {

CartesianPositionController::CartesianPositionController()
{}

CartesianPositionController::~CartesianPositionController()
{
  // sub_command_.shutdown();
  command_filter_.reset();
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

  // Create Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  ik_solver_.reset(new KDL::ChainIkSolverPos_NR(kdl_chain_, *fk_solver_.get(), *ik_vel_solver_.get()));
  // Initialize Joint Values
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_vel_.resize(kdl_chain_.getNrOfJoints());

  // Subscribe to pose commands
  sub_command_.subscribe(node_, "command", 10);
  command_filter_.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(
                          sub_command_, tf_, root_name_, 10, node_));
  command_filter_->registerCallback(boost::bind(&CartesianPositionController::command, this, _1));

  // // Get all joint states from the hardware interface
  // joint_names_ = robot->getJointNames();
  // num_joints_ = joint_names_.size();
  // for (unsigned i=0; i<num_joints_; i++)
  //   ROS_DEBUG("Got joint %s", joint_names_[i].c_str());
  // // Load URDF for robot
  // urdf::Model urdf;
  // if (!urdf.initParam("robot_description")){
  //   ROS_ERROR("Failed to parse urdf file");
  //   return false;
  // }
  // // Get URDF for joints
  // for (unsigned i=0; i<num_joints_; i++){
  //   joint_urdf_.push_back( urdf.getJoint(joint_names_[i]) );
  //   if(!joint_urdf_[i]){
  //     ROS_ERROR("Could not find joint %s in urdf", joint_names_[i].c_str());
  //     return false;
  //   }
  // }
  // // Get Joint Limits
  // for (unsigned i=0; i<num_joints_; i++){
  //   // Velocity Limit
  //   joint_vel_limits_.push_back(joint_urdf_[i]->limits->velocity);
  //   // Upper Position Limit
  //   joint_upper_position_limits_.push_back(joint_urdf_[i]->limits->upper);
  //   // Lower Position Limit
  //   joint_lower_position_limits_.push_back(joint_urdf_[i]->limits->lower);
  // }
  // // Get all joint handles
  // for (unsigned i=0; i<joint_names_.size(); i++){
  //   joint_handles_.push_back( robot->getJointHandle(joint_names_[i]) );
  // }
  // // Resize commands to be the proper size
  // command_.resize(num_joints_);
  // // Initialize command subscriber
  // // sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &CartesianPositionController::commandCB, this);
  return true;
}

void CartesianPositionController::command(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  // convert message to transform
  tf::Stamped<tf::Pose> pose_stamped;
  poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  tf::PoseTFToKDL(pose_stamped, pose_desi_);
}

void CartesianPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // // Assign velocity to each joint from command
  // for(unsigned int i=0;i<num_joints_;i++){
  //   double command_vel = 0;
  //   double current_joint_cmd = command_[i];
  //   hardware_interface::JointHandle joint = joint_handles_[i];
  //   // Get current joint's velocity limits
  //   double vel_limit = joint_vel_limits_[i];
  //   // Check command velocity agains limits
  //   if(current_joint_cmd > vel_limit){
  //     command_vel = vel_limit;
  //     ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<current_joint_cmd);
  //   }else if(current_joint_cmd < -vel_limit){
  //     command_vel = -vel_limit;
  //     ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<current_joint_cmd);
  //   }else{
  //     command_vel = current_joint_cmd;
  //   }
  //   // Set velocity command to current joint
  //   joint.setCommand(command_vel);
  // }
}

void CartesianPositionController::stopping(const ros::Time& time)
{
  // ROS_INFO_STREAM("Shutting Down Controller and Commanding Zero Velocity.");
  // // Set all velocities to zero.
  // for(unsigned int i=0;i<num_joints_;i++){
  //   double command_vel = 0;
  //   hardware_interface::JointHandle joint = joint_handles_[i];
  //   joint.setCommand(command_vel);
  // }
  // ROS_INFO_STREAM("Controller Stopped Successfully.");
  
}

// void CartesianPositionController::commandCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//   // command_ = msg->data;
// }

}// namespace

PLUGINLIB_DECLARE_CLASS(velocity_controllers, CartesianPositionController, velocity_controllers::CartesianPositionController, controller_interface::ControllerBase)

