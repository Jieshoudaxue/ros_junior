#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_pub");
    ros::NodeHandle nh;

    ros::Publisher std_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "hello ycao " + std::to_string(count++);
        ROS_INFO("%s", msg.data.c_str());

        std_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}