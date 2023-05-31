#! /usr/bin/env python3

import os, sys
import rospy
import rosbag
import argparse

class BagPlay(object):
  def __init__(self, bag, start_offset, rate, topic_list=[]):
    self.__bag = bag
    self.__start_ofset = start_offset
    self.__rate = rate
    self.__topic_list = topic_list
    self.__publisher = {}

  def create_publisher(self):
    bag = rosbag.Bag(self.__bag, 'r')

    for topic, msg, t in bag.read_messages(topics=self.__topic_list, raw=False):
      if topic not in self.__publisher:
        self.__publisher[topic] = rospy.Publisher(topic, type(msg), queue_size=10)

    bag.close()

  def start_play(self):
    bag = rosbag.Bag(self.__bag, "r")

    bag_start_time = bag.get_start_time() + self.__start_ofset
    rospy_start_time =rospy.get_time()
    for topic, msg, t in bag.read_messages(topics=self.__topic_list, start_time=rospy.Time.from_sec(bag_start_time)):
      relative_time = (t.to_sec() - bag_start_time) / self.__rate
      while rospy.get_time() < (rospy_start_time + relative_time):
        rospy.sleep(0.01)

      self.__publisher[topic].publish(msg)

      # print("%f" %t.to_sec())
      cur = t.to_sec()
      du = rospy.Duration(cur - bag_start_time).to_sec()
      sys.stdout.write("[RUNNING] Bag Time: %f Duration: %f\r" %(cur, du))
      sys.stdout.flush()

      if rospy.is_shutdown():
        break

    print("\n")
    bag.close

def main():
  parser = argparse.ArgumentParser(description="play topic from rosbag")
  parser.add_argument("-b", "--bag", type=str, required=True, help="specify rosbag")
  parser.add_argument("-s", "--start_offset", type=int, default=0, help="offset time from bag start")
  parser.add_argument("-r", "--rate", type=float, default=1.0, help="specify play rate")
  parser.add_argument("-t", "--topic_list", nargs="+", type=str, help="specify play topics")
  parser.add_argument("extra_args", nargs='*', help=argparse.SUPPRESS)

  args = parser.parse_args()

  if not os.path.isfile(args.bag):
    print("%s is not found!" %args.bag)
    return

  rospy.init_node("bag_player")

  player = BagPlay(args.bag, args.start_offset, args.rate, args.topic_list)

  player.create_publisher()

  print("start play bag after 2 sec...")
  rospy.sleep(2.0)

  player.start_play()

if __name__ == "__main__":
  main()