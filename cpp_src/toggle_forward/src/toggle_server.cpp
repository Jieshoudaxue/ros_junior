#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <thread>

bool forward = true;

bool toggleForward(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    forward = !forward;
    ROS_INFO("now receiving : %s", forward ? "forward" : "rotate");
    return true;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "toggle_server");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("toggle_forward", &toggleForward);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    auto thread_pub = [=]() {
        ros::Rate rate(2);
        while(ros::ok()) {
            geometry_msgs::Twist msg;
            msg.linear.x = forward? 1.0 : 0.0;
            msg.angular.z = forward ? 0.0 : 1.0;
            pub.publish(msg);
            rate.sleep();
        }
    };

    std::thread th1 = std::thread(thread_pub);

    ros::spin();

    if (th1.joinable()) {
        th1.join();
    }

    return 0;
}