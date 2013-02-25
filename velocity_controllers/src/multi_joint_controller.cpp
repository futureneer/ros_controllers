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

#include <velocity_controllers/multi_joint_controller.h>
#include <pluginlib/class_list_macros.h>

namespace velocity_controllers {

MultiJointController::MultiJointController()
{}

MultiJointController::~MultiJointController()
{
  sub_command_.shutdown();
}

bool MultiJointController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  // Get all joint states from the hardware interface
  joint_names_ = robot->getJointNames();
  num_joints_ = joint_names_.size();

  for (unsigned i=0; i<num_joints_; i++)
    ROS_DEBUG("Got joint %s", joint_names_[i].c_str());

  // Load URDF for robot
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")){
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  
  // Get URDF for joints
  for (unsigned i=0; i<num_joints_; i++){
    joint_urdf_.push_back( urdf.getJoint(joint_names_[i]) );
    if(!joint_urdf_[i]){
      ROS_ERROR("Could not find joint %s in urdf", joint_names_[i].c_str());
      return false;
    }
  }

  // Get all joint handles
  for (unsigned i=0; i<joint_names_.size(); i++){
    joints_.push_back( robot->getJointHandle(joint_names_[i]) );
  }

  // Resize commands to be the proper size
  command_.resize(num_joints_);
  
  // Old code for publishing joint state
  // controller_state_publisher_.reset(
  //   new realtime_tools::RealtimePublisher<controllers_msgs::JointControllerState>(n, "state", 1));
  
  sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &MultiJointController::commandCB, this);
  
  return true;
}

void MultiJointController::update(const ros::Time& time, const ros::Duration& period)
{

  // Assign velocity to each joint from command
  for(unsigned int i=0;i<num_joints_;i++){
    double command_vel = 0;
    double current_joint_cmd = command_[i];
    hardware_interface::JointHandle joint = joints_[i];
    // Get current joint's velocity limits
    double vel_limit = joint_urdf_[i]->limits->velocity;
    // Check command velocity agains limits
    if(current_joint_cmd > vel_limit){
      command_vel = vel_limit;
      ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<current_joint_cmd);
    }else if(current_joint_cmd < -vel_limit){
      command_vel = -vel_limit;
      ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<current_joint_cmd);
    }else{
      command_vel = current_joint_cmd;
    }
    // Set velocity command to current joint
    joint.setCommand(command_vel);
  }

  // Old code for publishing joint state
  // // Publish joint state
  // if(controller_state_publisher_ && controller_state_publisher_->trylock())
  // {
  //   controller_state_publisher_->msg_.header.stamp = time;
  //   controller_state_publisher_->msg_.set_point = command_vel;
  //   controller_state_publisher_->msg_.process_value = joint_.getPosition();
  //   controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
  //   controller_state_publisher_->msg_.error = 0;
  //   controller_state_publisher_->msg_.time_step = period.toSec();
  //   controller_state_publisher_->msg_.command = 0;
  //   // Publish State
  //   controller_state_publisher_->unlockAndPublish();
  // }
}

void MultiJointController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  command_ = msg->data;
}

}// namespace

PLUGINLIB_DECLARE_CLASS(velocity_controllers, MultiJointController, velocity_controllers::MultiJointController, controller_interface::ControllerBase)

