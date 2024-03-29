#! /usr/bin/env python3

import rosbag
import rospy
import argparse
import logging
from datetime import datetime
import roslib.message
from threading import Lock
import signal, sys

logging.basicConfig(level=logging.DEBUG)


class BagRecord(object):
  def __init__(self, output_bag, topic_infos):
    self.__output_bag = output_bag
    self.__topic_infos = topic_infos
    self.__subscribers = []
    # __Lock is to protect bag write and bag close in two thread
    # in python, this is necessary !
    self.__Lock = Lock()
    self.__bag = rosbag.Bag(self.__output_bag, "w")
    signal.signal(signal.SIGINT, self.signal_handler)

  # msg must be placed before topic !!
  def topic_callback(self, msg, topic):
    self.__Lock.acquire()
    try:
      self.__bag.write(topic, msg, rospy.Time.now())
    except Exception as e:
      rospy.logerr("%s" %e)
    self.__Lock.release()

  def start_recorder(self):
    for topic, topic_type in self.__topic_infos:
      msg_class = roslib.message.get_message_class(topic_type)
      self.__subscribers.append(rospy.Subscriber(topic, msg_class, self.topic_callback, topic))

  def signal_handler(self, sig, frame):
    rospy.loginfo("ctrl+c, close bag")
    self.__Lock.acquire()
    self.__bag.close()
    self.__Lock.release()
    sys.exit(0)


def main():
  parser = argparse.ArgumentParser(description="record topic to rosbag")
  parser.add_argument("-O", "--output_bag", type=str, help="specify rosbag")
  parser.add_argument("-t", "--topics", nargs="+", type=str, help="specify topic")
  parser.add_argument("-a", "--all", action="store_true", help="record all topic")
  parser.add_argument('extra_args', nargs='*', help=argparse.SUPPRESS)
  args=parser.parse_args()  

  if not args.all and not args.topics:
    logging.error("please specify topic using -t or -a")
    return

  rospy.init_node("bag_recorder")

  output_bag = datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".bag"
  if args.output_bag:
    output_bag = args.output_bag

  topic_infos = []
  if args.all:
    topic_infos = rospy.get_published_topics()
  elif args.topics:
    topic_infos = [(topic, msg_class) for topic, msg_class in rospy.get_published_topics() if topic in args.topics]

  recorder = BagRecord(output_bag, topic_infos)

  rospy.loginfo("start recorder...")

  recorder.start_recorder()

  rospy.spin()

if __name__ == "__main__":
  main()