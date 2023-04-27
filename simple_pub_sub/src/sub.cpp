#include <ros/ros.h>
#include <std_msgs/String.h>

void std_cb(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("i received: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_sub");
    ros::NodeHandle nh;

    ros::Subscriber std_sub = nh.subscribe("chatter", 1000, &std_cb);

    ros::spin();
    return 0;
}