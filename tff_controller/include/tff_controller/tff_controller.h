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

#ifndef CARTESIAN_TFF_CONTROLLER_H
#define CARTESIAN_TFF_CONTROLLER_H

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>

//#include <pr2_mechanism_model/chain.h>
#include <tf/transform_datatypes.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <geometry_msgs/Twist.h>
#include <tff_controller/TaskFrameFormalism.h>

namespace tff_controller {

  class TFFController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  public:
    TFFController();
    ~TFFController();

    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

    void command(const tff_controller::TaskFrameFormalismConstPtr& tff_msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_command_;

    // pid controllers
    std::vector<control_toolbox::Pid> vel_pid_controller_, pos_pid_controller_;

    // kdl stuff for kinematics
    KDL::Chain kdl_chain_;
    boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    KDL::JntArrayVel jnt_posvel_;
    KDL::JntArray jnt_pos_, jnt_eff_;
    KDL::Jacobian jacobian_;

    // command for tff
    std::vector<int> mode_;
    std::vector<double> value_, twist_to_wrench_;

    // output of the controller
    std::vector<hardware_interface::JointHandle> joint_handles_;
    KDL::Wrench wrench_desi_;

    KDL::Twist position_, twist_meas_;
    KDL::Frame pose_meas_, pose_meas_old_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_position_publisher_;
    unsigned int loop_count_;
  };


} // namespace


#endif
