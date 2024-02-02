/**
 * @file exercise_4.cpp
 * @brief This file contains the ROS node for exercise 4 of the assignment.
 * It subscribes to the topic /scan and publishes the position of the people
 * found in the scan.
 */
#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include "polar_point.h"


/**
 * @brief Finds the feet of the people in the scan.
 * @param m The scan message.
 * @return A vector of points representing the feet of the people.
 * @details This method finds the feet of the people in the scan.
 * It does so by finding the points where the distance is not infinite.
 * It then groups the points in pairs and calculates the average point
 * of each pair.
 */
std::vector<PolarPoint> find_feet(const sensor_msgs::LaserScan& m){
                           
	std::vector<PolarPoint> feet;          // feet collection to return
    const double inf = std::numeric_limits<double>::infinity(); // infinity

    int index = 0;                          // index of the current point
    bool in_foot = false;                   // whether the current point is in a foot
    std::vector<PolarPoint> foot_profile;   // points in the current foot

    // for each point in the scan
    for(auto dist : m.ranges) {

        // if the point is not infinite and we are not in a foot we start a new foot
        if(!in_foot && dist != inf) 
            in_foot = true;

        // if the point is infinite and we are in a foot we end the current foot
        if(in_foot && dist == inf) {

            in_foot = false;

            // add the average point of the foot to the feet collection
            feet.push_back(PolarPoint::getMedianPoint(foot_profile));

            double x,y;
            feet.back().convertToCartesian(x,y);
            
            foot_profile.clear();
        }

        // if we are in a foot we add the point to the foot profile
        if(in_foot && dist != inf) {
            foot_profile.push_back(PolarPoint( m.angle_min + (index * m.angle_increment),dist));
        }

        index++;
    }

    return feet;
}

/**
 * @brief Finds the people in the scan.
 * @param feet The feet of the people.
 * @return A vector of points representing the people.
 * @details This method finds the people in the scan.
 * It does so by grouping the feet in pairs and calculating the average point
 * of each pair.
 */
std::vector<std::pair<double,double>> find_people(std::vector<PolarPoint>& feet) {
    std::vector<std::pair<double,double>> people;         // people collection to return
    std::vector<PolarPoint> person_feet;   // feet of the current person

    std::vector<std::pair<PolarPoint, PolarPoint>> pairs; // pairs of feet

    while (feet.size() > 1) {
        pairs.push_back(PolarPoint::getClosestPoints(feet));
    }

    for(auto& pair : pairs) {
        double x1,x2,y1,y2;
        pair.first.convertToCartesian(x1,y1);
        pair.second.convertToCartesian(x2,y2);
        
        people.push_back(PolarPoint::getMiddlePoint(pair.first, pair.second));
    }

    return people;
}



/**
 * @brief Callback function for the /scan topic.
 * @param m The scan message.
 * @details This function is called when a message is received on the /scan topic.
 * It finds the feet of the people in the scan and prints them.
 */
void messageCallback(const sensor_msgs::LaserScan& m){

    // find the feet and the people
	std::vector<PolarPoint> feet = find_feet(m);

    // find the people
    std::vector<std::pair<double,double>> people = find_people(feet);

    // print the people found in cartesian coordinates
    for(int i=0; i<people.size(); i++) {
        ROS_INFO("Person #%d: (%f, %f)", i, people[i].first, people[i].second);
    }
    std::cout<<std::endl;

}

/**
 * @brief Main function of the node.
 * @param argc Number of arguments.
 * @param argv Array of arguments.
 * @return 0 if the program exits correctly.
 * @details This is the main function of the node.
 * It initializes the node, subscribes to the topic /scan and
 * calls the messageCallback function when a message is received.
 */
int main(int argc, char **argv){
	ros::init(argc, argv, "Exercise_4");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan", 1000, messageCallback);
	ros::spin();
	return 0;
}




