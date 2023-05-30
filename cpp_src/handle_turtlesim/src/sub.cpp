#include <ros/ros.h>
#include <turtlesim/Pose.h>

void processMsg(const turtlesim::Pose& msg) {
    ROS_INFO("position: [%f, %f], direction: %f", msg.x, msg.y, msg.theta);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_pose");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &processMsg);

    ros::spin();
    return 0;
}