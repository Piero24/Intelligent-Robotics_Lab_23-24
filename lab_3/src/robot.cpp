#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <exercise_3/chargingAction.h>
class chargingAction
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<exercise_3::chargingAction> as_;
	std::string action_name_;
	exercise_3::chargingFeedback feedback_;
	exercise_3::chargingResult result_;
public:
	chargingAction(std::string name) : as_(nh_, name, boost::bind(&chargingAction::executeCB, this, _1), false), action_name_(name)
	{
		as_.start();
	}
	
	~chargingAction(void){}
	void executeCB(const exercise_3::chargingGoalConstPtr &goal)
	{
		ros::Rate r(1);
		result_.reached = false;
		feedback_.level = 5;
		std_msgs::Header header;
		int seq = 0;
		header.seq = seq;
		header.stamp = ros::Time::now();
		header.frame_id = "0";
		feedback_.h = header;
		
		for (int i = 1; i <= goal->max; i++)
		{
			if (as_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				break;
			}
			feedback_.level++;
			seq++;
			as_.publishFeedback(feedback_);
			r.sleep();
			ROS_INFO("Feedback Level: %d", feedback_.level);
			
			if (feedback_.level == 100)
			{
				result_.reached = true;
				std_msgs::Header header1;
				header1.seq = seq;
				header1.stamp = ros::Time::now();
				header1.frame_id = "1";
				result_.h = header1;
				ROS_INFO("%s: Succeeded (the battery is fully recharged)", action_name_.c_str());
				as_.setSucceeded(result_);
				break;
			}
		}	
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "CHARGING___");
	chargingAction charging("CHARGING___");
	ros::spin();
	return 0;
}
