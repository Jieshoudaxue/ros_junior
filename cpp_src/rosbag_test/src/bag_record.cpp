#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ctime>
#include <iomanip>
#include <topic_tools/shape_shifter.h>
#include "cxxopts.hpp"

void genericCallback(const topic_tools::ShapeShifter::ConstPtr& msg, rosbag::Bag* bag, const std::string& topic_name) {
    bag->write(topic_name, ros::Time::now(), *msg);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "bag_record");
    ros::NodeHandle nh;

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("O,output", "Name of the bag", cxxopts::value<std::string>())
        ("t,topic", "topic list", cxxopts::value<std::vector<std::string>>())
        ("a,all", "all topic")
        ("h,help", "show help");
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        ROS_INFO("%s", options.help().c_str());
        return 0;
    }

    if (!result.count("topic") && !result.count("all")) {
        ROS_ERROR("please specify topic using -t or -a");
        return -1;
    }

    std::time_t current_time = std::time(nullptr);
    std::tm* time_info = std::localtime(&current_time);
    char buffer[50];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", time_info);
    std::string bag_name = "./" + std::string(buffer) + ".bag";
    if (result.count("output")) {
        bag_name = result["output"].as<std::string>();
    }

    std::vector<std::string> topic_list;
    if (result.count("all")) {
        ros::master::V_TopicInfo topic_infos;
        if (ros::master::getTopics(topic_infos)) {
            for (const auto & topic_info : topic_infos) {
                topic_list.push_back(topic_info.name);
            }
        }
    } else if (result.count("topic")) {
        topic_list = result["topic"].as<std::vector<std::string>>();
    }

    rosbag::Bag bag;

    try {
        bag.open(bag_name.c_str(), rosbag::bagmode::Write);
    } catch (rosbag::BagException &e) {
        ROS_ERROR("open rosbag failed: %s", e.what());
        return -1;
    }

    std::vector<ros::Subscriber> sub_list;
    for (auto & item : topic_list) {
        ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(item, 1000, std::bind(genericCallback, std::placeholders::_1, &bag, item));        
        sub_list.push_back(sub);
        ROS_INFO("subscribed to %s", item.c_str());
    }

    ros::spin();

    bag.close();


    return 0;
}




