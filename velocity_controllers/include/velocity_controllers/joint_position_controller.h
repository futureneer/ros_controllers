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

#ifndef VELOCITY_CONTROLLERS_JOINT_POSITION_CONTROLLER_H
#define VELOCITY_CONTROLLERS_JOINT_POSITION_CONTROLLER_H

/**
   @class velocity_controllers::JointPositionController
   @brief Joint Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "velocity_controllers::JointPositionController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint position to achieve.

   Publishes:

   - @b state (controllers_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controllers_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <controllers_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>


namespace velocity_controllers
{

class JointPositionController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:

  JointPositionController();
  ~JointPositionController();

  bool init(hardware_interface::VelocityJointInterface *robot, const std::string &joint_name,const control_toolbox::Pid &pid);
  bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double cmd);

  void starting(const ros::Time& time);
  
  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);
  /*!
   * \brief Sets Gains
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  /*!
   * \brief Sets Gains
   */
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  /*!
   * \brief Get the name of the associated joint
   */
  std::string getJointName();
  
  // Members
  hardware_interface::JointHandle joint_;
  boost::shared_ptr<const urdf::Joint> joint_urdf_;
  realtime_tools::RealtimeBuffer<double> command_;             /**< Last commanded position. */

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */
  boost::scoped_ptr<realtime_tools::RealtimePublisher<controllers_msgs::JointControllerState> > controller_state_publisher_ ;
  ros::Subscriber sub_command_;
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);
};

} // namespace

#endif
