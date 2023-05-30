#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>
// https://github.com/facontidavide/ros_msg_parser
#include "ros_msg_parser/ros_parser.hpp"
#include "cxxopts.hpp"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "bar_echo");

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("b,bag", "Name of the bag", cxxopts::value<std::string>())
        ("t,topic", "topic name", cxxopts::value<std::string>())
        ("h,help", "show help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        ROS_INFO("%s", options.help().c_str());
        return 0;
    }

    if (!result.count("bag")) {
        ROS_ERROR("please specify bag using -b");
        return -1;
    } 

    if (!result.count("topic")) {
        ROS_ERROR("please specify a topic using -t");
        return -1;
    } 

    std::string bag_name;
    if (result.count("bag")) {
        bag_name = result["bag"].as<std::string>();
    }    

    std::vector<std::string> topic_list;
    if (result.count("topic")) {
        topic_list.push_back(result["topic"].as<std::string>());
    }  

    rosbag::Bag bag;

    try {
        bag.open(bag_name.c_str(), rosbag::bagmode::Read);
    } catch (rosbag::BagException &e) {
        ROS_ERROR("open rosbag failed: %s", e.what());
        return -1;
    }

    rosbag::TopicQuery topic_query(topic_list);
    rosbag::View view(bag, topic_query);

    RosMsgParser::ParsersCollection parsers;
    for (const rosbag::ConnectionInfo* connection : view.getConnections()) {
        const std::string& topic_name = connection->topic;
        parsers.registerParser(topic_name, *connection);
    }

    ROS_INFO("starting to parse bag...");
    for(const rosbag::MessageInstance& item : view) {
        std::string topic_name = item.getTopic();
        std::string data_type = item.getDataType();
        std::string md5_val = item.getMD5Sum();
        double time_sec = item.getTime().toSec();
        std::string msg_def = item.getMessageDefinition();

        ROS_INFO("--------- %s ----------\n", topic_name.c_str());
        const auto deserialized_msg = parsers.deserialize(topic_name, item);
        for (const auto& it : deserialized_msg->renamed_vals) {
            const std::string& key = it.first;
            const double value = it.second;
            ROS_INFO(" %s = %f\n", key.c_str(), value);
        }

        for (const auto& it : deserialized_msg->flat_msg.name) {
            const std::string& key = it.first.toStdString();
            const std::string& value = it.second;
            ROS_INFO(" %s = %s\n", key.c_str(), value.c_str());
        }
    }

    return 0;
}