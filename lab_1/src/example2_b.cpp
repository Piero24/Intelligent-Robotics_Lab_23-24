#include "ros/ros.h"
#include "lab_1/msg1.h"

void messageCallback(const lab_1::msg1::ConstPtr& msg) {

    if (msg->A == 0 && msg->B == "0") {
        ROS_INFO("Cleaning complited successfully! Return to base!");

    } else {
        ROS_INFO("Room with ID: [%d] named: [%s] cleaned! Battery at [%d]", msg->A, msg->B.c_str(), msg->C);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example2_b");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("message", 1000, messageCallback);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}