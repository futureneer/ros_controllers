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

#ifndef VELOCITY_CONTROLLERS_JOINT_SETPOINT_CONTROLLER_H
#define VELOCITY_CONTROLLERS_JOINT_SETPOINT_CONTROLLER_H

/**
   @class velocity_controllers:JointSetpointController
   @brief Cartesian Position Controller (velocity)

   This class calculates the IK of the robot using the cartesian pose command and commands joint velocity.  It can also command joint velocity by joint_position_commands

   @section ROS interface

   @param type Must be "JointSetpointController"

   Subscribes to:
   - @b cartesian_pose_command (geometry_msgs::PoseStamped) : The cartesian pose to command
   - @b joint_position_command (std_msgs::Float64MultiArray) : A vector of joint positions to command
*/

// BOOST
#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
// ROS
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
// Hardware
#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
// Controllers
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
// State Publishing   
#include <realtime_tools/realtime_publisher.h>
// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/chain.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/frames.hpp>
// Messaging
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace velocity_controllers
{

class JointSetpointController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  JointSetpointController();
  ~JointSetpointController();

  bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);  
  void commandCB_joint(const std_msgs::Float64MultiArray::ConstPtr& msg);

  // Input to the controller
  KDL::Frame pose_desired_, pose_measured_;
  KDL::Twist twist_ff_;
  // State output
  KDL::Twist twist_error_;

private:
  // Member Functions
  bool getRobotInfo();

  // Members  
  ros::NodeHandle node_;
  std::string controller_name_, root_name_, tip_name_;
  ros::Time last_time_;

  // Robot Structure
  hardware_interface::VelocityJointInterface *robot_;

  // PID Controller
  std::vector<control_toolbox::Pid> pid_controller_;

  // KDL and Kinematics
  std::string robot_desc_string_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray joint_positions_;
  KDL::JntArray joint_positions_desired_;
  KDL::JntArray joint_positions_upper_limits_;
  KDL::JntArray joint_positions_lower_limits_;
  KDL::JntArray joint_velocities_;
  KDL::JntArray joint_accelerations_;
  KDL::JntArray joint_velocities_dt_;
  KDL::JntArray joint_velocities_desired_;
  KDL::JntArray joint_velocities_command_;
  KDL::JntArray joint_velocities_overshoot_;
  KDL::Jacobian jacobian_;

  // Paths
  KDL::Path_Line* current_path_;
  bool on_path_;
  double setpoint_limit_;
  double setpoint_increment_;

  // URDF and Joint Information
  unsigned int num_joints_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_acceleration_limits_;
  std::vector<double> joint_upper_position_limits_;
  std::vector<double> joint_lower_position_limits_;
  std::vector<std::string> hw_joint_names_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> chain_link_names_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector< boost::shared_ptr<const urdf::Joint> > joint_urdf_;
  std::vector<double> command_;

  // reatltime publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_error_publisher_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_publisher_;
  unsigned int loop_count_;

  // TF and Subscribers
  tf::TransformListener tf_;
  ros::Subscriber joint_position_command_sub_;
};

}

#endif
