#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>
#include "cxxopts.hpp"

bool findTopic(const std::string topic, const std::vector<std::string>& topic_list) {
    if (topic_list.empty()) {
        return true;
    }

    for (auto & item : topic_list) {
        // ROS_INFO("%s %s", topic.c_str(), item.c_str());
        if (topic == item) {
            return true;
        }
    }

    return false;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "bag_play");
  ros::NodeHandle nh;

  // https://github.com/jarro2783/cxxopts
  cxxopts::Options options(argv[0], "parse cmd line");
  options.add_options()
      ("b,bag", "name of the bag", cxxopts::value<std::string>())
      ("s,start_offset", "sec offset of start time", cxxopts::value<int>())
      ("r,rate", "player rate", cxxopts::value<double>())
      ("t,topic", "topic list", cxxopts::value<std::vector<std::string>>())
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

  std::string bag_name;
  int start_offset_sec = 0;
  double play_delay_sec = 2.0;
  double play_rate = 1.0;
  std::vector<std::string> topic_list;
  if (result.count("bag")) {
    bag_name = result["bag"].as<std::string>();
  }  
  if (result.count("start_offset")) {
    start_offset_sec = result["start_offset"].as<int>();
  }
  if (result.count("rate")) {
    play_rate = result["rate"].as<double>();
  }  
  if (result.count("topic")) {
      topic_list = result["topic"].as<std::vector<std::string>>();
  } 

  rosbag::Bag bag;
  try {
    bag.open(bag_name.c_str(), rosbag::bagmode::Read);
  } catch (rosbag::BagException& e) {
    ROS_ERROR("open bag failed %s", e.what());
    return -1;
  }

  rosbag::View view(bag);
  std::map<std::string, ros::Publisher> pubs;

  const std::vector<const rosbag::ConnectionInfo*> connections = view.getConnections();
  for (auto& item : connections) {
    if (findTopic(item->topic, topic_list)) {
      std::string topic_name = item->topic;
      ROS_INFO("%s %s %s", topic_name.c_str(), item->datatype.c_str(), item->md5sum.c_str()/*, item->msg_def.c_str()*/);
      ros::AdvertiseOptions opts;
      opts.topic = topic_name;
      opts.queue_size = 1000;
      opts.md5sum = item->md5sum;
      opts.datatype = item->datatype;
      opts.message_definition = item->msg_def;
      
      pubs[topic_name] = nh.advertise(opts);
    }
  }

  ROS_INFO("start play bag after %f sec...", play_delay_sec);
  ros::Duration(0, (int)(play_delay_sec * 1e9)).sleep();

  ros::Time begin_time = view.getBeginTime() + ros::Duration(start_offset_sec, 0);
  ros::Duration stone_delta = ros::Time::now() - begin_time;

  int cnt = 0;
  ros::Duration sum_du = ros::Duration();
  for (const auto& msg : view) {
    if (msg.getTime() >= begin_time) {

      ros::Duration msg_delta = ros::Duration(ros::Duration(msg.getTime() - begin_time).toSec() / play_rate);
      while (begin_time + msg_delta + stone_delta > ros::Time::now()) {
        ros::Duration du = ros::Duration(ros::Duration(msg.getTime() - begin_time).toSec()/play_rate) - sum_du;
        sum_du += du;
        du.sleep();
      }

      printf("[RUNNING] Bag Time: %.6f  Duration: %.6f\r", msg.getTime().toSec(), ros::Duration(msg.getTime() - begin_time).toSec());
      fflush(stdout);

      if (findTopic(msg.getTopic(), topic_list)) {
        topic_tools::ShapeShifter::ConstPtr smsg = msg.instantiate<topic_tools::ShapeShifter>();
        pubs[msg.getTopic()].publish(smsg);
      }

    }

    if (!ros::ok()) {
      break;
    }
  }

  printf("\n");

  bag.close();


  return 0;
}




