#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

#include <boost/foreach.hpp>
#include "cxxopts.hpp"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "bar_echo");

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("b,bag", "Name of the bag", cxxopts::value<std::string>())
        ("h,help", "show help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        ROS_INFO("%s", options.help().c_str());
        return 0;
    }

    std::string bag_name = "/home/ycao/Study/ros_noetic/bag_dir/square_1.bag";
    if (result.count("bag")) {
        bag_name = result["bag"].as<std::string>();
    }    


    rosbag::Bag bag;

    try {
        bag.open(bag_name.c_str(), rosbag::bagmode::Read);
    } catch (rosbag::BagException &e) {
        ROS_ERROR("open rosbag failed: %s", e.what());
        return -1;
    }

    rosbag::View view(bag);
    ROS_INFO("starting to parse bag...");

    std::string pose_def;
    std::string cmd_def;
    for(const rosbag::MessageInstance& item : view) {
        std::string topic_name = item.getTopic();
        std::string data_type = item.getDataType();
        std::string md5_val = item.getMD5Sum();
        double time_sec = item.getTime().toSec();

        if (topic_name == "/turtle1/pose") {
            turtlesim::Pose::ConstPtr msg = item.instantiate<turtlesim::Pose>();
            ROS_INFO("[%s : %s : %s : %lf] x = %f, y = %f, theta = %f", 
                    topic_name.c_str(), data_type.c_str(), md5_val.c_str(), time_sec, msg->x, msg->y, msg->theta);

            pose_def = item.getMessageDefinition();
        } else if (topic_name == "/turtle1/cmd_vel") {
            geometry_msgs::Twist::ConstPtr msg = item.instantiate<geometry_msgs::Twist>();
            ROS_INFO("[%s : %s : %s : %lf] linear.x = %f, linear.y = %f, linear.z = %f", 
                    topic_name.c_str(), data_type.c_str(), md5_val.c_str(), time_sec, msg->linear.x, msg->linear.y, msg->linear.z);

            cmd_def = item.getMessageDefinition();
        }

    }

    ROS_INFO("/turtle1/pose message definition: \n%s\n", pose_def.c_str());
    ROS_INFO("/turtle1/cmd_vel message definition: \n%s", cmd_def.c_str());

    return 0;
}