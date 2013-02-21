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

#include <velocity_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace velocity_controllers {

JointVelocityController::JointVelocityController()
: command_(0)
{}

JointVelocityController::~JointVelocityController()
{
  sub_command_.shutdown();
}

bool JointVelocityController::init(hardware_interface::VelocityJointInterface *robot, const std::string &joint_name)
{
  joint_ = robot->getJointHandle(joint_name);

  // Load URDF for robot
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")){
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  
  // Get URDF for joint
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_){
    ROS_ERROR("Could not find joint %s in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

bool JointVelocityController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (parameter: %s/joint)", n.getNamespace().c_str());
    return false;
  }

  joint_ = robot->getJointHandle(joint_name);
  
  // Load URDF for robot
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")){
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  
  // Get URDF for joint
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_){
    ROS_ERROR("Could not find joint %s in urdf", joint_name.c_str());
    return false;
  }

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<controllers_msgs::JointControllerState>(n, "state", 1));
  
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointVelocityController::commandCB, this);
  
  return true;
}

void JointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  double command_vel = 0;
  double vel_limit = joint_urdf_->limits->velocity;

  if(command_ > vel_limit){
    command_vel = vel_limit;
    ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<command_);
  }else if(command_ < -vel_limit){
    command_vel = -vel_limit;
    ROS_DEBUG_STREAM("Velocity Limit Exceeded: "<<command_);
  }else{
    command_vel = command_;
  }

  // Set joint velocity command
  joint_.setCommand(command_vel);

  // Publish joint state
  if(controller_state_publisher_ && controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.set_point = command_vel;
    controller_state_publisher_->msg_.process_value = joint_.getPosition();
    controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
    controller_state_publisher_->msg_.error = 0;
    controller_state_publisher_->msg_.time_step = period.toSec();
    controller_state_publisher_->msg_.command = 0;
    // Publish State
    controller_state_publisher_->unlockAndPublish();
  }
}

void JointVelocityController::commandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

}// namespace

PLUGINLIB_DECLARE_CLASS(velocity_controllers, JointVelocityController, velocity_controllers::JointVelocityController, controller_interface::ControllerBase)

