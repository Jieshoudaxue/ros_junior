#include <ros/ros.h>
#include <msg_self/Student.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "msg_pub");

    ros::NodeHandle nh;
    ros::Publisher msg_pub = nh.advertise<msg_self::Student>("student", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        msg_self::Student msg;
        msg.name = "jieshoudaxue";
        msg.age = 30;
        ROS_INFO("name = %s, age = %u", msg.name.c_str(), msg.age);

        msg_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}