#include <ros/ros.h>
#include <std_msgs/String.h>
#include <beginner_tutorials/Num.h>

void std_cb(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("i received: %s", msg->data.c_str());
}

void num_cb(const beginner_tutorials::Num::ConstPtr &msg) {
    ROS_INFO("i received: %ld", msg->num);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_listener");
    ros::NodeHandle nh;

    ros::Subscriber std_sub = nh.subscribe("chatter", 1000, &std_cb);
    ros::Subscriber num_sub = nh.subscribe("num", 1000, &num_cb);


    ros::spin();


    return 0;
}