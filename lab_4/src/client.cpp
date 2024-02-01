#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

void clustering(std::vector<std::vector<float>> points, std::vector<std::vector<float>>& centers,std::vector<std::vector<float>>& centers1,int k,int maxIter, std::vector<int>& label, std::vector<int>& label1){
	float tollerance = 0.05;//Tollerance of the neighborhood we want to 'cluster'
	int counter = 1;
	bool allInRange = true;
	int loopCounter = 0;
	std::vector<std::vector<float>> foundPoints;
	std::vector<std::vector<float>> foundPoints1;
	while(counter<k)
	{
		for(int i = 1; i<points.size();i++)
		{
			//ROS_INFO("FIRST LOOP");
			for(int j = 0; j< centers.size();j++)
			{
				//ROS_INFO("SECOND LOOP");
				if(((points[i][0] >= centers[j][0]-tollerance && points[i][0] <= centers[j][0]+tollerance) || (points[i][1] >= centers[j][1]-tollerance && points[i][1] <= centers[j][1]+tollerance))&& counter <k)
				{
					//ROS_INFO("CONDITION NOT SATISFIED");
					foundPoints.push_back(points[i]);
					label[i] = j;
					//ROS_INFO("%d i %d j", i,j);
					allInRange = false;
				}
			}
			if(allInRange && counter < k)
			{
				centers[counter]=points[i];
				//ROS_INFO("iteration %d counter %d",i,counter);
				counter++;
			}
			else if(counter == k)
				break;
			else
				allInRange = true;
		}
		if(counter >= 5)
		{
			loopCounter++;
			tollerance -=0.005;
		}
		if(loopCounter > maxIter)
			break;
	}
	
	tollerance = 0.05;
	counter=1;
	while(counter<k)
	{
		for(int i = points.size()-1; i>0;i--)
		{
			//ROS_INFO("FIRST LOOP");
			for(int j = centers1.size()-1; j >= 0;j--)
			{
				//ROS_INFO("SECOND LOOP");
				if(((points[i][0] >= centers1[j][0]-tollerance && points[i][0] <= centers1[j][0]+tollerance) || (points[i][1] >= centers1[j][1]-tollerance && points[i][1] <= centers1[j][1]+tollerance))&& counter <k)
				{
					//ROS_INFO("CONDITION NOT SATISFIED");
					foundPoints1.push_back(points[i]);
					label1[i] = j;
					//ROS_INFO("%d i %d j", i,j);
					allInRange = false;
				}
			}
			
			if(allInRange && counter < k)
			{
				centers1[counter]=points[i];
				//ROS_INFO("iteration %d counter %d",i,counter);
				counter++;
			}
			else if(counter == k)
				break;
			else
				allInRange = true;
		}
		if(counter >= 5)
		{
			loopCounter++;
			tollerance -=0.005;
		}
		if(loopCounter > maxIter)
			break;
	}
	/*ROS_INFO("bottom");
	for(int i=0; i<points.size();i++)
		ROS_INFO("%f,%f label [%d] = label1 [%d]",points[i][0],points[i][1],label[i],label1[points.size()-1-i]);*/
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("Received Message. Laser rays number: %lu", msg->ranges.size());

	float range = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	std::vector<std::vector<float>> points;
	std::vector<int> labels;
	std::vector<std::vector<float>> centers(6);
	std::vector<std::vector<float>> centers1(6);
	ROS_INFO("min: %f; max: %f; increment: %f", msg->angle_min, msg->angle_max, msg->angle_increment);
	for(int i = 0; i < range;i++)
	{
		if(msg->ranges[i] != 0 && msg->ranges[i] <=20 )
		{
			std::vector<float> temp_points(2);
			temp_points[0]=msg->ranges[i]*cos(i*msg->angle_increment);
			temp_points[1]=msg->ranges[i]*sin(i*msg->angle_increment);
			points.push_back(temp_points);
			labels.push_back(i);
		}
	}
	std::vector<int> label(points.size());
	std::vector<int> label1(points.size());
	for(int i =0; i<centers.size();i++)
		{centers[i] = points[0];
		centers1[i] = points[0];}
	//ROS_INFO("BEFORE CLUSTERING");
	clustering(points,centers,centers1,6,5,label,label1);
	//ROS_INFO("AFTER CLUSTERING");
	for(int i=0; i<points.size();i++)
		ROS_INFO("point %d is %f,%f [%d] [%d]",labels[i],points[i][0],points[i][1],label[i],label1[points.size()-1-i]);
	for(int i = 0; i<centers.size();i++)
		{ROS_INFO("Center %d-th: [%f,%f]",i,centers[i][0],centers[i][1]);
		ROS_INFO("Center %d-th: [%f,%f]",i,centers1[centers.size()-1-i][0],centers1[centers.size()-1-i][1]);}
	//TODO:
	
	//Clustering from the centers we have found so far
	//Merge the two closest clusters in order to detect the person
	//Find the centroids of the new clusters in order to determine as precise as possible the new person's position
	//Check if it works for the whole bag
	//ros::shutdown();//Just cause i don't want to have a spam of points for now...
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "client");
	ros::NodeHandle n;
	std::string topic_name = "/scan";
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>(topic_name, 10, callback);
	ros::spin();
	return 0;
}
