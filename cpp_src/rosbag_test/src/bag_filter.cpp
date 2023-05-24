#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "cxxopts.hpp"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "bag_filter");

    // https://github.com/jarro2783/cxxopts
    cxxopts::Options options(argv[0], "parse cmd line");
    options.add_options()
        ("i,input_bag", "Name of the input bag", cxxopts::value<std::string>())
        ("o,output_bag", "Name of the output bag", cxxopts::value<std::string>())
        ("s,start_offset", "offset time base start time", cxxopts::value<int>())
        ("e,end_offset", "offset time base end time", cxxopts::value<int>())
        ("t,topic_list", "topic list", cxxopts::value<std::vector<std::string>>())
        ("h,help", "show help");
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      ROS_INFO("%s", options.help().c_str());
      return 0;
    }

    std::string input_bag = "/home/ycao/Study/ros_noetic/bag_dir/square_1.bag";
    std::string output_bag = "/home/ycao/Study/ros_noetic/bag_dir/square_1_filter.bag";
    int start_offset = 0;
    int end_offset = 0;
    std::vector<std::string> topic_list = {"/turtle1/pose", "/turtle1/cmd_vel"};
    if (result.count("input_bag")) {
        input_bag = result["input_bag"].as<std::string>();
    }  
    if (result.count("output_bag")) {
        output_bag = result["output_bag"].as<std::string>();
    }  
    if (result.count("start_offset")) {
        start_offset = result["start_offset"].as<int>();
    }  
    if (result.count("end_offset")) {
        end_offset = result["end_offset"].as<int>();
    }  
    if (result.count("topic_list")) {
        topic_list = result["topic_list"].as<std::vector<std::string>>();
    }  

    rosbag::Bag ibag;
    try {
        ibag.open(input_bag.c_str(), rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        ROS_ERROR("open input bag failed: %s", e.what());
        return -1;
    }

    rosbag::Bag obag;
    try {
        obag.open(output_bag.c_str(), rosbag::bagmode::Write);
    } catch (rosbag::BagException& e) {
        ROS_ERROR("open output bag failed: %s", e.what());
        return -1;
    }


    rosbag::TopicQuery topic_query(topic_list);
    rosbag::View view(ibag, topic_query);

    ros::Time begin_time = view.getBeginTime() + ros::Duration(start_offset, 0);
    ros::Time end_time = view.getEndTime() - ros::Duration(end_offset, 0);

    for (const auto& msg : view) {
        if (msg.getTime() >=  begin_time && msg.getTime() <= end_time) {
            obag.write(msg.getTopic(), msg.getTime(), msg);
        }
        
    }

    ibag.close();
    obag.close();


    ROS_INFO("successfully filtered bag : %s", output_bag.c_str());

    return 0;
}





















// function specify: base start and end offset time(sec), filter bag; base topic list, filter bag



