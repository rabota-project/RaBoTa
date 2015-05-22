/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for simulation
 */

#include <rabota_hardware_interface/rabota_hardware_interface.h>

namespace ros_control_rabota
{

RabotaHardwareInterface::RabotaHardwareInterface(ros::NodeHandle& nh)
: nh_(nh)
, joint_mode_(1) // POSITION
{
	// Initialize shared memory and interfaces
	init();
	// Create the controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	ROS_INFO("here 1");

	// Get period and create timer
	nh_.param("hardware_interface/loop_hz", loop_hz_, 0.1);
	ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
	ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

	ROS_INFO("here 2");

	non_realtime_loop_ = nh_.createTimer(update_freq, &RabotaHardwareInterface::update, this);

	ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");

	ROS_INFO("here 3");

}

RabotaHardwareInterface::~RabotaHardwareInterface()
{
}

void RabotaHardwareInterface::arduinoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
//	ROS_INFO("hello world \n I heard: %f" , msg->position[0]);

	joint_position_[0] = msg->position[0];
	joint_velocity_[0] = msg->velocity[0];
	joint_effort_[0]   = msg->effort[0];

	joint_position_[1] = msg->position[1];
	joint_velocity_[1] = msg->velocity[1];
	joint_effort_[1]   = msg->effort[1];

	joint_position_[2] = msg->position[2];
	joint_velocity_[2] = msg->velocity[2];
	joint_effort_[2]   = msg->effort[2];

//	joint_velocity_temp_[1] = msg->velocity[1];
//	joint_effort_temp_[1]   = msg->effort[1];


//	ROS_INFO("on arduino callback I heard: [j0 j1 ] = [ %f  %f  ]",joint_position_[0],joint_position_[1]);

}

void RabotaHardwareInterface::init()
{
	command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("arduino/motor_command", 10);
	joints_angle_sub_ = nh_.subscribe("arduino/motor_states", 10, &RabotaHardwareInterface::arduinoCallback , this);

	// Get joint names
	nh_.getParam("hardware_interface/joints", joint_names_);
	if (joint_names_.size() == 0)
	{
		ROS_FATAL_STREAM_NAMED("init","Not joints found on parameter server for controller, did you load the proper yaml file?");
	}
	num_joints_ = joint_names_.size();

	// Resize vectors
	joint_position_.resize(num_joints_);
	joint_velocity_.resize(num_joints_);
	joint_effort_.resize(num_joints_);
	joint_position_command_.resize(num_joints_);
	joint_velocity_command_.resize(num_joints_);
	joint_effort_command_.resize(num_joints_);

	joint_position_command_[0] = 0;
	joint_position_command_[1] = 0;
	joint_position_command_[2] = 0;


//	joint_position_temp_.resize(num_joints_);
//	joint_velocity_temp_.resize(num_joints_);
//	joint_effort_temp_.resize(num_joints_);
//	joint_position_command_temp_.resize(num_joints_);
//	joint_velocity_command_temp_.resize(num_joints_);
//	joint_effort_command_temp_.resize(num_joints_);

//	nh_.getParam("joint1_position_controller/pid",gains);

	// Initialize controller
	for (int i = 0; i < num_joints_; ++i)
	{
		ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

		// Create joint state interface
		joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

		// Create position joint interface
		position_joint_interface_.registerHandle(hardware_interface::JointHandle(
				joint_state_interface_.getHandle(joint_names_[i]),&joint_position_command_[i]));

		// Create velocity joint interface
		//velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(
		//    joint_state_interface_.getHandle(joint_names_[i]),&joint_velocity_command_[i]));

		// Create effort joint interface
		//effort_joint_interface_.registerHandle(hardware_interface::JointHandle(
		//    joint_state_interface_.getHandle(joint_names_[i]),&joint_effort_command_[i]));

	}
	registerInterface(&joint_state_interface_); // From RobotHW base class.
	registerInterface(&position_joint_interface_); // From RobotHW base class.
	registerInterface(&velocity_joint_interface_); // From RobotHW base class.
	registerInterface(&effort_joint_interface_); // From RobotHW base class.
}

void RabotaHardwareInterface::update(const ros::TimerEvent& e)
{
	elapsed_time_ = ros::Duration(e.current_real - e.last_real);

	// Input
	read();

	// Control
	controller_manager_->update(ros::Time::now(), elapsed_time_);

	// Output
	write(elapsed_time_);

	//ROS_INFO("here %d",count++);

}

void RabotaHardwareInterface::read()
{
	// Read the joint states from your hardware here
//	joint_position_[0] = joint_position_temp_[0];
//	joint_velocity_[0] = joint_velocity_temp_[0];
//	joint_effort_[0]   = joint_effort_temp_[0];
//
//	joint_position_[1] = joint_position_temp_[1];
//	joint_velocity_[1] = joint_velocity_temp_[1];
//	joint_effort_[1]   = joint_effort_temp_[1];
//
//	joint_position_[2] = joint_position_temp_[2];
//	joint_velocity_[2] = joint_velocity_temp_[2];
//	joint_effort_[2]   = joint_effort_temp_[2];
}

void RabotaHardwareInterface::write(ros::Duration elapsed_time)
{
	std_msgs::Float64MultiArray msg;
	msg.data.resize(3);
//	msg.data.push_back(joint_position_command_[0]);
//	msg.data.push_back(joint_position_command_[1]);
	msg.data[0] = joint_position_command_[0];
	msg.data[1] = joint_position_command_[1];
	msg.data[2] = joint_position_command_[2];


//	p_error_ = joint_position_command_[0] - joint_position_[0];
//	msg.data[0] = 270.0*p_error_ * POSITION_STEP_FACTOR / loop_hz_;
//
//	p_error_ = joint_position_command_[1] - joint_position_[1];
//	msg.data[1] = p_error_ * POSITION_STEP_FACTOR / loop_hz_;
////	ROS_INFO("[ j1   j2 ] = [ %f   %f ]",msg.data[0],msg.data[1]);
//
//	if (msg.data[0]>=255.0) msg.data[0] = 255;
//	else if (msg.data[0]<-255) msg.data[0] = -255;
//
//	if (msg.data[1]>=255.0) msg.data[1] = 255;
//	else if (msg.data[1]<-255) msg.data[1] = -255;


	command_pub_.publish(msg);

	// Send commands in different modes


	// Move all the states to the commanded set points slowly
	//	for (std::size_t i = 0; i < num_joints_; ++i)
	//	{
	//		switch (joint_mode_)
	//		{
	//		case 1: //hardware_interface::MODE_POSITION:
	//			// Position
	//			p_error_ = joint_position_command_[i] - joint_position_[i];
	//			// scale the rate it takes to achieve position by a factor that is invariant to the feedback loop
	//			joint_position_[i] += p_error_ * POSITION_STEP_FACTOR / loop_hz_;
	//			break;
	//
	//		case 2: //hardware_interface::MODE_VELOCITY:
	//			// Position
	//			joint_position_[i] += joint_velocity_[i] * elapsed_time.toSec();
	//
	//			// Velocity
	//			v_error_ = joint_velocity_command_[i] - joint_velocity_[i];
	//			// scale the rate it takes to achieve velocity by a factor that is invariant to the feedback loop
	//			joint_velocity_[i] += v_error_ * VELOCITY_STEP_FACTOR / loop_hz_;
	//			break;
	//
	//		case 3: //hardware_interface::MODE_EFFORT:
	//			ROS_ERROR_STREAM_NAMED("write","Effort not implemented yet");
	//			break;
	//		}
	//	}
}


} // namespace
