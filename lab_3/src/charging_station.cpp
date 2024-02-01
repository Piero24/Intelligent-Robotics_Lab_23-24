#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include <exercise_3/chargingAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "robot_charging");
	actionlib::SimpleActionClient<exercise_3::chargingAction> ac("CHARGING___", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	
	ROS_INFO("Action server started, sending goal.");

	exercise_3::chargingGoal goal;
	goal.max = 100;
	std_msgs::Header header;
	int seq = 0;
	header.seq = seq;
	header.stamp = ros::Time::now();
	header.frame_id = "1";
	goal.h = header;
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));
	
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	ROS_INFO("Action did not finish before the time out.");
	return 0;
}
