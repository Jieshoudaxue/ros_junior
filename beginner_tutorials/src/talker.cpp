#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <beginner_tutorials/Num.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_talker");

    ros::NodeHandle nh;


    ros::Publisher std_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Publisher num_pub = nh.advertise<beginner_tutorials::Num>("num", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello ycao " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        std_pub.publish(msg);

        beginner_tutorials::Num msg_n;
        msg_n.num = rand();

        num_pub.publish(msg_n);

        ros::spinOnce();

        loop_rate.sleep();
        ++ count;
    }




    return 0;
}