#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

void clustering(std::vector<std::vector<float>>& points,std::vector<int>& labels){							//This method does the clustering of our
	int currentIteration = 0;																				//points. It takes as parameters the set
	const int radius = 1;																					//of points and a vector of labels.
	labels[0] = 1;																							//We decided to cluster the point accord-
	int currentLabel = 1;																					//ing to their Euclidean distance. We 
	float distance;																							//chose to set the radius to 1, i.e., the
	std::vector<std::vector<float>> centers;																//distance between points can be at 
	centers.push_back(points[0]);																			//maximum 1 in order to be in the same 
	bool found = false;																						//cluster.
	while(true){																							//This cycle goes on till all the points
		for(int i=1;i<points.size();i++){																	//are clustered. It computes the Eucledian
			distance += (points[i][0]-centers[currentLabel-1][0])*(points[i][0]-centers[currentLabel-1][0]);//distance for each point and checks if 
			distance += (points[i][1]-centers[currentLabel-1][1])*(points[i][1]-centers[currentLabel-1][1]);//it is in the range we specified.
			distance = sqrt(distance);																		//
			if(distance <= radius){																			//
				labels[i] = currentLabel;																	//
			}																								//
			distance=0;																						//
		}																									//
		for(int i=0;i<labels.size();i++){																	//After we clustered the points, we check
			if(labels[i]==0){																				//the next unclustered point as the new
				centers.push_back(points[i]);																//center and we keep iterating.
				currentLabel++;																				//
				found = true;																				//
				break;																						//
			}																								//
		}																									//
		if(!found)																							//If all the points have been clustered,
			break;																							//stop the cycle.
		found=false;																						//
	}																										//
}																											//
																											//
std::vector<std::vector<float>> getCenters(std::vector<std::vector<float>> points, std::vector<int> labels){//This function computes the center point
	std::vector<std::vector<float>> centers;																//for each person. It takes as input the
	int numElements = 0;																					//points and the labels we gave to them.
	int currentLabel = 1;																					//
	int maxLabel = 1;																						//
	for(int i =0;i<labels.size();i++){																		//We find the maximum label available in
		if(labels[i]>=maxLabel)																				//order to know when to stop computing 
			maxLabel=labels[i];																				//the centers.
	}																										//
	std::vector<float> temp_vector(2);																		//
	float temp_distance_x=0;																				//
	float temp_distance_y=0;																				//In this cycle we calculate the median 
	while(currentLabel-1<maxLabel){																			//point per person.
		for(int i=0;i<labels.size();i++){																	//
			if(labels[i]==currentLabel){																	//
				temp_distance_x += points[i][0];															//
				temp_distance_y += points[i][1];															//
				numElements++;																				//
			}																								//
		}																									//
		temp_distance_x /= numElements;																		//Here we save the point calculated and 
		temp_distance_y = temp_distance_y/numElements;														//move onto the next person.
		temp_vector[0]=temp_distance_x;																		//
		temp_vector[1]=temp_distance_y;																		//
		centers.push_back(temp_vector);																		//
		temp_distance_y = 0;																				//
		temp_distance_x = 0;																				//
		numElements = 0;																					//
		currentLabel++;																						//We return the vector of centers we have
	}																										//just computed.
	return centers;																							//
}																											//
																											//
void msgCallback(const sensor_msgs::LaserScanPtr& ls){														//This is the messageCallback function
	float range = (ls->angle_max - ls->angle_min)/ls->angle_increment;										//which is called in the subscriber.
	std::vector<std::vector<float>> points;																	//We found the parameters we need and
	std::vector<int> labels;																				//calculated the cartesian coordiantes
	std::vector<std::vector<float>> centers;																//starting from the polar ones given by 
	//ROS_INFO("min: %f; max: %f; increment: %f", ls->angle_min,ls->angle_max,ls->angle_increment);			//the bag file.
	//ROS_INFO("range max: %f; range min: %f",ls->range_max,ls->range_min);									//
	for(int i = 0; i < range;i++){																			//We decided to discard all the points 
		if(ls->ranges[i] != 0 && ls->ranges[i] <=ls->range_max ){											//that where zero or more than the maximum
			std::vector<float> temp_points(2);																//range in order to simplify the 
			temp_points[0]=ls->ranges[i]*cos(i*ls->angle_increment);										//computation.
			temp_points[1]=ls->ranges[i]*sin(i*ls->angle_increment);										//
			points.push_back(temp_points);																	//
			labels.push_back(i);																			//
		}																									//
	}																										//Once we have the points, we set the
	for(int i =0; i<centers.size();i++)																		//first one as the first center, and
		centers[i] = points[0];																				//we call the cluster method.
	std::vector<int> newLabels(points.size());																//
	clustering(points,newLabels);																			//
	std::vector<std::vector<float>> finalPoints = getCenters(points,newLabels);								//Once we have clustered the points, we 
	//for(int i = 0; i<newLabels.size();i++)																//get the centers and print the position 
		//ROS_INFO("The point %d [%f,%f] has label %d",i,points[i][0],points[i][1],newLabels[i]);			//of the people.
	for(int i=0;i<finalPoints.size();i++)																	//
		ROS_INFO("The person %d is in position [%f,%f]",i+1,finalPoints[i][0],finalPoints[i][1]);			//
}																											//
																											//
int main(int argc, char **argv){																			//
	ros::init(argc,argv, "laser_scanner");																	//
	ros::NodeHandle n;																						//We subscribe to the /scan topic in order
	ros::Subscriber sub = n.subscribe("/scan",1000,msgCallback);											//to get the data we need.
	ros::spin();																							//
	return 0;																								//
}																											//
