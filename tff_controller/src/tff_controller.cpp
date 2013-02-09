/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Wim Meeussen
 */

#include <algorithm>

#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <pluginlib/class_list_macros.h>

#include "tff_controller/tff_controller.h"


PLUGINLIB_DECLARE_CLASS(tff_controller, TFFController, tff_controller::TFFController, controller_interface::ControllerBase)

using namespace KDL;
using namespace ros;

namespace tff_controller {

  TFFController::TFFController() :
    jnt_to_twist_solver_(NULL),
    jnt_to_jac_solver_(NULL),
    cmd_mode_(6),
    cmd_value_(6),
    twist_to_wrench_(6),
    state_position_publisher_(NULL)
  {}


  TFFController::~TFFController()
  {}

  bool TFFController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    nh_ = n;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    if (!ros::param::search(n.getNamespace(),"robot_description", robot_description)){
      ROS_ERROR_STREAM("TFFController: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
      return false;
    }
    if (!nh_.getParam("root_name", root_name)){
      ROS_ERROR_STREAM("TFFController: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
      return false;
    }
    if (!nh_.getParam("tip_name", tip_name)){
      ROS_ERROR_STREAM("TFFController: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
      return false;
    }

    // Construct an URDF model from the xml string
    urdf::Model urdf_model;
    urdf_model.initString(robot_description);

    // Get a KDL tree from the robot URDF
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
      ROS_ERROR("Failed to construct kdl tree from URDF model");
      return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree.getSegments();
      KDL::SegmentMap::iterator it;

      for( it=segment_map.begin();
          it != segment_map.end();
          it++ )
      {
        ROS_ERROR_STREAM( "    "<<(*it).first);
      }

      return false;
    }

    // Get joint handles for all of the joints in the chain
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin();
        it != kdl_chain_.segments.end();
        ++it)
    {
      joint_handles_.push_back(robot->getJointHandle(it->getJoint().getName()));
    }

    // create solvers
    jnt_to_twist_solver_.reset(new ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
    jnt_posvel_.resize(kdl_chain_.getNrOfJoints());
    jnt_eff_.resize(kdl_chain_.getNrOfJoints());
    jacobian_.resize(kdl_chain_.getNrOfJoints());

    // twist to wrench
    double trans, rot;

    nh_.param("twist_to_wrench_trans", trans, 0.0);
    for (unsigned int i=0; i<3; i++)
      twist_to_wrench_[i] = trans;

    nh_.param("twist_to_wrench_rot", rot, 0.0);
    for (unsigned int i=3; i<6; i++)
      twist_to_wrench_[i] = rot;

    // Construct pid controllers
    control_toolbox::Pid pid_controller;
    if (!pid_controller.init(NodeHandle(nh_, "vel_trans"))) return false;
    for (unsigned int i=0; i<3; i++)
      vel_pid_controller_.push_back(pid_controller);

    if (!pid_controller.init(NodeHandle(nh_, "vel_rot"))) return false;
    for (unsigned int i=0; i<3; i++)
      vel_pid_controller_.push_back(pid_controller);

    if (!pid_controller.init(NodeHandle(nh_, "pos_trans"))) return false;
    for (unsigned int i=0; i<3; i++)
      pos_pid_controller_.push_back(pid_controller);

    if (!pid_controller.init(NodeHandle(nh_, "pos_rot"))) return false;
    for (unsigned int i=0; i<3; i++)
      pos_pid_controller_.push_back(pid_controller);

    // subscribe to tff commands
    sub_command_ = nh_.subscribe<tff_controller::TaskFrameFormalism>("command", 1,
                                                                     &TFFController::command, this);
    // realtime publisher for control state
    state_position_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh_, "state/position", 1));

    return true;
  }

  void TFFController::starting(const ros::Time &time)
  {
    // set initial modes and values
    for (unsigned int i=0; i<6; i++){
      cmd_mode_[i] = tff_controller::TaskFrameFormalism::FORCE;
      cmd_value_[i] = 0;
    }

    // reset pid controllers
    for (unsigned int i=0; i<6; i++){
      vel_pid_controller_[i].reset();
      pos_pid_controller_[i].reset();
    }

    // set initial position, twist
    FrameVel frame_twist;

    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      jnt_posvel_.q(i) = joint_handles_[i].getPosition();
      jnt_posvel_.qdot(i) = joint_handles_[i].getVelocity();
    }

    jnt_to_twist_solver_->JntToCart(jnt_posvel_, frame_twist);
    pose_meas_old_ = frame_twist.value();
    position_ = Twist::Zero();

    // set desired wrench to 0
    wrench_desi_ = Wrench::Zero();

    loop_count_ = 0;
  }


  void TFFController::update(const ros::Time& time, const ros::Duration& dt)
  {
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      jnt_posvel_.q(i) = joint_handles_[i].getPosition();
      jnt_posvel_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // get the chain jacobian
    jnt_to_jac_solver_->JntToJac(jnt_posvel_.q, jacobian_);

    // get cartesian twist and pose
    FrameVel frame_twist;
    jnt_to_twist_solver_->JntToCart(jnt_posvel_, frame_twist);
    pose_meas_ = frame_twist.value();
    twist_meas_ = pose_meas_.M.Inverse() * (frame_twist.deriv());

    // calculate the distance traveled along the axes of the tf
    position_ += pose_meas_.M.Inverse() * diff(pose_meas_old_, pose_meas_);
    pose_meas_old_ = pose_meas_;

    // calculate desired wrench
    wrench_desi_ = Wrench::Zero();
    for (unsigned int i=0; i<6; i++){
      switch(cmd_mode_[i]) {
        case tff_controller::TaskFrameFormalism::FORCE:
          wrench_desi_[i] = cmd_value_[i];
          break;
        case tff_controller::TaskFrameFormalism::VELOCITY:
          wrench_desi_[i] = twist_to_wrench_[i] * (cmd_value_[i] + vel_pid_controller_[i].computeCommand(cmd_value_[i] - twist_meas_[i], dt));
          break;
        case tff_controller::TaskFrameFormalism::POSITION:
          wrench_desi_[i] = twist_to_wrench_[i] * (pos_pid_controller_[i].computeCommand(cmd_value_[i] - position_[i], dt));
          break;
        default:
          // Something is wrong; complain, unset commands, and return
          static const std::string dimensions("xyz");

          ROS_ERROR_STREAM("TFF mode "<<cmd_mode_[i]
                           <<" in "<<((i<3)?("translation"):("rotation"))
                           <<" dimension "<<dimensions.at(i%3)
                           <<" is not valid. Please see the TaskFrameformalism"
                           <<" message for acceptable controller modes.");

          // Set all the joint efforts to 0.0
          for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); j++){
            joint_handles_[j].setCommand(0.0);
          }

          return;
      };
    }

    // Convert wrench to base reference frame
    wrench_desi_ = (pose_meas_.M * wrench_desi_);

    // Convert the wrench into joint efforts
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
      jnt_eff_(i) = 0;
      for (unsigned int j=0; j<6; j++) {
        jnt_eff_(i) += (jacobian_(j,i) * wrench_desi_(j));
      }

      // Set the joint effort
      joint_handles_[i].setCommand(jnt_eff_(i));
    }

    // publish state
    if (++loop_count_ % 100 == 0){
      if (state_position_publisher_){
        if (state_position_publisher_->trylock()){
          state_position_publisher_->msg_.linear.x = position_.vel(0);
          state_position_publisher_->msg_.linear.y = position_.vel(1);
          state_position_publisher_->msg_.linear.z = position_.vel(2);
          state_position_publisher_->msg_.angular.x = position_.rot(0);
          state_position_publisher_->msg_.angular.y = position_.rot(1);
          state_position_publisher_->msg_.angular.z = position_.rot(2);
          state_position_publisher_->unlockAndPublish();
        }
      }
    }
  }


  void TFFController::command(const tff_controller::TaskFrameFormalismConstPtr& tff_msg)
  {
    cmd_mode_[0] = trunc(tff_msg->mode.linear.x);
    cmd_mode_[1] = trunc(tff_msg->mode.linear.y);
    cmd_mode_[2] = trunc(tff_msg->mode.linear.z);
    cmd_mode_[3] = trunc(tff_msg->mode.angular.x);
    cmd_mode_[4] = trunc(tff_msg->mode.angular.y);
    cmd_mode_[5] = trunc(tff_msg->mode.angular.z);

    cmd_value_[0] = tff_msg->value.linear.x;
    cmd_value_[1] = tff_msg->value.linear.y;
    cmd_value_[2] = tff_msg->value.linear.z;
    cmd_value_[3] =  tff_msg->value.angular.x;
    cmd_value_[4] =  tff_msg->value.angular.y;
    cmd_value_[5] =  tff_msg->value.angular.z;
  }

}; // namespace
