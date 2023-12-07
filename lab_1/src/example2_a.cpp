#include "ros/ros.h"
#include "lab_1/msg1.h"
#include <sstream>
#include <map>
#include <string>


int main(int argc, char **argv) {

    ros::init(argc, argv, "example2_a");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<lab_1::msg1>("message", 1000);
    ros::Rate loop_rate(5); // Set the loop rate to 5Hz

    std::map<int, std::string> myMap = {
        { 1, "Robot Vision Lab" },
        { 2, "SSL Lab" },
        { 3, "Neurorobotics Lab" },
        { 4, "IAS-Lab" },
        { 5, "Autonomous Robotics Lab" }
    };

    int batteryLevel = 100;

    ROS_INFO("Start cleaning...");
    ros::Duration(1).sleep();

    while (ros::ok()) {

        int totRooms = myMap.size();
        printf("totRooms: %d\n", totRooms);

        lab_1::msg1 msg;
        
        if (!(myMap.empty())) {

            double batteryPerRoom = batteryLevel / static_cast<double>(totRooms);
            batteryLevel -= static_cast<int>(batteryPerRoom);

            msg.A = myMap.begin()->first;
            msg.B = myMap.begin()->second;
            msg.C = batteryLevel;
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

            myMap.erase(myMap.begin());

        } else {
            msg.A = 0;
            msg.B = "0";
            msg.C = batteryLevel;
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            break;
        }

        ros::Duration(1).sleep();
    }

    return 0;
}