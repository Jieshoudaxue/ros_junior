#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_velocity");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    const std::string PARAM_NAME="~max_vel";
    double maxVel;
    bool ok = ros::param::get(PARAM_NAME, maxVel);
    if (!ok) {
        ROS_FATAL("can not get param ~max_vel");
        return -1;
    }

    srand(time(0));
    ros::Rate loop_rate(2);
    while(ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = maxVel*double(rand())/double(RAND_MAX);
        msg.angular.z = double(rand())/double(RAND_MAX);

        pub.publish(msg);

        ROS_INFO("sending rand velocity cmd: linear = %f, angular = %f", msg.linear.x, msg.angular.z);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
