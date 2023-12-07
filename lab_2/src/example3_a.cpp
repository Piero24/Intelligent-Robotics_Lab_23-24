#include "ros/ros.h"
#include "lab_2/ex2_srv.h"
#include <cstdlib>
#include <map>
#include <string>
#include <iostream>
#include <random>

bool add(lab_2::ex2_srv::Request &req, lab_2::ex2_srv::Response &res) {

    static std::map<int, std::string> myMap = {
        { 1, "Robot Vision Lab" },
        { 2, "SSL Lab" },
        { 3, "Neurorobotics Lab" },
        { 4, "IAS-Lab" },
        { 5, "Autonomous Robotics Lab" }
    };

    static int numReq = 0;
    static int batteryRobot = 100;

    int lowerLimit = batteryRobot - static_cast<int>(0.25 * batteryRobot);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(lowerLimit, batteryRobot);

    batteryRobot = dis(gen);

    if (batteryRobot < 10) {
        ROS_INFO("Battery is low, return to the nearest station for recharge.");
        batteryRobot = 100;
    }

    auto it = myMap.begin();
    int firstKey = it->first;
    std::string firstValue = it->second;

    ROS_INFO("request from station with ID: [%d]", req.id);

    res.msg.A = firstKey;
    res.msg.B = firstValue;
    res.msg.C = batteryRobot;

    ROS_INFO("request: A=%d, B=%s C=%d", res.msg.A, res.msg.B.c_str(), res.msg.C);
    numReq++;

    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "example3_a");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("message", add);
    ROS_INFO("Ready to add 3 ints.");
    ros::spin();
    return 0;

}