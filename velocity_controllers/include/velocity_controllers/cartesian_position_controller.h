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

#ifndef VELOCITY_CONTROLLERS_CARTESIAN_POSITION_CONTROLLER_H
#define VELOCITY_CONTROLLERS_CARTESIAN_POSITION_CONTROLLER_H

/**
   @class velocity_controllers:CartesianPositionController
   @brief Joint Velocity Controller (torque or force)

   This class passes the commanded effort down to the joint

   @section ROS interface

   @param type Must be "CartesianPositionController"
   @param joint Name of the joint to control.

   Subscribes to:
   - @b command (std_msgs::Float64) : The joint effort to apply
*/

// BOOST
#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
// ROS
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
// Hardware
#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
// Controllers
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
// State Publishing   
#include <realtime_tools/realtime_publisher.h>
// KDL
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
// Messaging
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
// #include <pr2_controller_interface/controller.h>
// #include <pr2_mechanism_model/chain.h>
// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace velocity_controllers
{

class CartesianPositionController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  CartesianPositionController();
  ~CartesianPositionController();

  bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time){}
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);  

  // Input to the controller
  KDL::Frame pose_desi_, pose_meas_;
  KDL::Twist twist_ff_;
  // State output
  KDL::Twist twist_error_;

private:
  // Member Functions
  void commandCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  KDL::Frame getPose();

  // Members  
  ros::NodeHandle node_;
  std::string controller_name_, root_name_, tip_name_;
  ros::Time last_time_;

  // Robot Structure
  hardware_interface::VelocityJointInterface *robot_;
  // pr2_mechanism_model::RobotState *robot_state_;
  // pr2_mechanism_model::Chain chain_;

  // PID Controller
  std::vector<control_toolbox::Pid> pid_controller_;

  // KDL and Kinematics
  KDL::Chain kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray jnt_pos_;
  KDL::JntArray jnt_eff_;
  KDL::Jacobian jacobian_;

  // URDF and Joint Information
  unsigned int num_joints_;
  std::vector<double> joint_vel_limits_;
  std::vector<double> joint_upper_position_limits_;
  std::vector<double> joint_lower_position_limits_;
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector< boost::shared_ptr<const urdf::Joint> > joint_urdf_;

  std::vector<double> command_;

  // reatltime publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_error_publisher_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_publisher_;
  unsigned int loop_count_;

  // TF and Subscribers
  tf::TransformListener tf_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_command_;
  boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> > command_filter_;
};

}

#endif
