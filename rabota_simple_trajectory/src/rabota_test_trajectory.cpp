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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Generate a random trajectory to test the ros_control controller
 */

// ROS
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace rabota_test
{

static const double MAX_JOINT_VALUE = M_PI; // continuous
static const double MIN_JOINT_VALUE = -M_PI; // continuous
static const double SEC_PER_TRAJ_POINT = 4; // time between points
static const std::size_t TRAJ_POINTS = 10; // number of points to generate

class TestTrajectory
{
public:

	/**
	 * \brief Constructor
	 * \param verbose - run in debug mode
	 */
	TestTrajectory(bool verbose)
: verbose_(verbose)
{
		ros::NodeHandle nh_private("~");
		std::string action_topic = "/rabota/position_trajectory_controller/follow_joint_trajectory";
		//    nh_private.getParam("action_topic", action_topic);
		if (action_topic.empty())
		{
			ROS_FATAL_STREAM_NAMED("constructor","Not follow joint trajectory action topic found on the parameter server");
		}
		ROS_INFO_STREAM_NAMED("constructor","Connecting to action " << action_topic);

		// create the action client
		// true causes the client to spin its own thread
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client(action_topic, true);

		ROS_INFO("Waiting for action server to start.");
		// wait for the action server to start
		action_client.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");

		// send a goal to the action
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = createTrajectory();
		std::cout << "Trajectry:\n" << goal.trajectory << std::endl;
		action_client.sendGoal(goal);

		// Wait for the action to return
		double wait_extra_padding = 2; // time to wait longer than trajectory itself
		bool finished_before_timeout = action_client.waitForResult(ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = action_client.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
			ROS_INFO("Action did not finish before the time out.");

		ROS_INFO_STREAM_NAMED("test_trajectory","TestTrajectory Finished");
}

	/**
	 * \brief Destructor
	 */
	~TestTrajectory()
	{
	}

	/**
	 * \brief Create random trajectory
	 */
	trajectory_msgs::JointTrajectory createTrajectory()
	{
		std::vector<std::string> joint_names;

		// Get joint names
		nh_.getParam("position_trajectory_controller/joints", joint_names);
		
		if (joint_names.size() == 0)
		{
			ROS_FATAL_STREAM_NAMED("init","Not joints found on parameter server for controller, did you load the proper yaml file?");
		}

		// Create header
		trajectory_msgs::JointTrajectory trajectory;
		trajectory.header.stamp = ros::Time::now();
		trajectory.joint_names = joint_names;


		trajectory.points.resize(4);
		//point 1
		trajectory.points[0].positions.resize(joint_names.size());
		trajectory.points[0].positions[0] = 0; // meter;
		trajectory.points[0].positions[1] = 0; // rad;
		trajectory.points[0].positions[2] = 0; // rad;

		trajectory.points[0].velocities.resize(joint_names.size());
		trajectory.points[0].velocities[0] = 0; // meter;
		trajectory.points[0].velocities[1] = 0; // rad;
		trajectory.points[0].velocities[2] = 0; // rad;
		trajectory.points[0].time_from_start = ros::Duration(SEC_PER_TRAJ_POINT);

		//point 2
		trajectory.points[1].positions.resize(joint_names.size());
		trajectory.points[1].positions[0] = 18; // meter;
		trajectory.points[1].positions[1] = 18;   // rad;
		trajectory.points[1].positions[2] = -10;  // rad;

		trajectory.points[1].velocities.resize(joint_names.size());
		trajectory.points[1].velocities[0] = 0; // meter;
		trajectory.points[1].velocities[1] = 0; // rad;
		trajectory.points[1].velocities[2] = 0; // rad;
		trajectory.points[1].time_from_start = ros::Duration(2*SEC_PER_TRAJ_POINT);

		//point 3
		trajectory.points[2].positions.resize(joint_names.size());
		trajectory.points[2].positions[0] = -10; // meter;
		trajectory.points[2].positions[1] = -5;   // rad;
		trajectory.points[2].positions[2] = 10;  // rad;

		trajectory.points[2].velocities.resize(joint_names.size());
		trajectory.points[2].velocities[0] = 0; // meter;
		trajectory.points[2].velocities[1] = 0; // rad;
		trajectory.points[2].velocities[2] = 0; // rad;
		trajectory.points[2].time_from_start = ros::Duration(3*SEC_PER_TRAJ_POINT);

		//point 4
		trajectory.points[3].positions.resize(joint_names.size());
		trajectory.points[3].positions[0] = 0; // meter;
		trajectory.points[3].positions[1] = 0;   // rad;
		trajectory.points[3].positions[2] = 0;  // rad;

		trajectory.points[3].velocities.resize(joint_names.size());
		trajectory.points[3].velocities[0] = 0; // meter;
		trajectory.points[3].velocities[1] = 0; // rad;
		trajectory.points[3].velocities[2] = 0; // rad;
		trajectory.points[3].time_from_start = ros::Duration(4*SEC_PER_TRAJ_POINT);

		// Create trajectory with x points
//		trajectory.points.resize(TRAJ_POINTS);
//		for (std::size_t i = 0; i < TRAJ_POINTS; ++i)
//		{
//			trajectory.points[i].positions.resize(joint_names.size());
//			// for each joint
//			for (std::size_t j = 0; j < joint_names.size(); ++j)
//			{
//				trajectory.points[i].positions[j] = dRand(MIN_JOINT_VALUE, MAX_JOINT_VALUE);
//				trajectory.points[i].time_from_start = ros::Duration(i * SEC_PER_TRAJ_POINT);
//			}
//		}

		return trajectory;
	}

	/** \brief Get random number */
	double dRand(double dMin, double dMax)
	{
		double d = (double)rand() / RAND_MAX;
		return dMin + d * (dMax - dMin);
	}

private:

	// A shared node handle
	ros::NodeHandle nh_;

	// Show more visual and console output, with general slower run time.
	bool verbose_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TestTrajectory> TestTrajectoryPtr;
typedef boost::shared_ptr<const TestTrajectory> TestTrajectoryConstPtr;

} // end namespace

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_trajectory");
	ROS_INFO_STREAM_NAMED("main", "Starting TestTrajectory...");

	// Allow the action server to recieve and send ros messages
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Check for verbose flag
	bool verbose = false;
	if (argc > 1)
	{
		for (std::size_t i = 0; i < argc; ++i)
		{
			if (strcmp(argv[i], "--verbose") == 0)
			{
				ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
				verbose = true;
				continue;
			}
		}
	}

	rabota_test::TestTrajectory server(verbose);

	ROS_INFO_STREAM_NAMED("main", "Shutting down.");
	ros::shutdown();

	return 0;
}
