#include "ros/ros.h"
#include "lab_2/ex2_srv.h"
#include <string>
#include <cstdlib>

int main(int argc, char **argv) {

    ros::init(argc, argv, "example3_b");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<lab_2::ex2_srv>("message");
    ros::Rate loop_rate(atoll(argv[2]));
    lab_2::ex2_srv srv;

    srv.request.id = atoll(argv[1]);

    while (ros::ok()) {
        ros::spinOnce();

        if (client.call(srv)) {
            ROS_INFO("request: A=%d, B=%s C=%d", srv.response.msg.A, 
            srv.response.msg.B.c_str(), 
            srv.response.msg.C);

        } else {
            ROS_ERROR("Failed to call service add_3_ints");
            return 1;
        }
    }
    return 0;
}