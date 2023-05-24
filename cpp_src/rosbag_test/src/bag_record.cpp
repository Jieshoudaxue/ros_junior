#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ctime>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include "cxxopts.hpp"


void poseCallback(const turtlesim::Pose::ConstPtr& msg, rosbag::Bag* bag) {
    bag->write("/turtle1/pose", ros::Time::now(), *msg);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, rosbag::Bag* bag) {
    bag->write("/turtle1/cmd_vel", ros::Time::now(), *msg);
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "bag_record");
    ros::NodeHandle nh;

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("O,output", "Name of the bag", cxxopts::value<std::string>())
        ("h,help", "show help");
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        ROS_INFO("%s", options.help().c_str());
        return 0;
    }

    std::time_t current_time = std::time(nullptr);
    std::string bag_name = "/home/ycao/Study/ros_noetic/bag_dir/" + std::to_string(current_time) + ".bag";
    if (result.count("output")) {
        bag_name = result["output"].as<std::string>();
    }

    rosbag::Bag bag;

    try {
        bag.open(bag_name.c_str(), rosbag::bagmode::Write);
    } catch (rosbag::BagException &e) {
        ROS_ERROR("open rosbag failed: %s", e.what());
        return -1;
    }

    ros::Subscriber sub_pose = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1000, std::bind(poseCallback, std::placeholders::_1, &bag));

    ros::Subscriber sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("turtle1/cmd_vel", 1000, std::bind(cmdVelCallback, std::placeholders::_1, &bag));

    ROS_INFO("subscribed to /turtle1/pose and /turtle1/cmd_vel, write ro square1.bag..");

    ros::spin();

    bag.close();


    return 0;
}




