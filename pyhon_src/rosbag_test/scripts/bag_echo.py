#! /usr/bin/env python3

import os
import argparse
import logging

import rosbag

logging.basicConfig(level=logging.DEBUG)

class BagEcho(object):
  def __init__(self, bag, topics):
      self.__bag = bag
      self.__topics = topics
  
  def echo_bag(self):
      msg_cnts = {}
      bag = rosbag.Bag(self.__bag, "r")
      for topic, msg, t in bag.read_messages(topics=self.__topics, raw=False):
          if topic not in msg_cnts:  
            msg_cnts[topic] = 1
          else:
            msg_cnts[topic] += 1

          print("----------[%s]---------" %topic)
          print(msg)

      bag.close()

      logging.info("----------[rosbag summary]---------")
      for topic in msg_cnts:  
        logging.info("topic %s msg cnt is %d" %(topic, msg_cnts[topic]))


def main():
  parser = argparse.ArgumentParser(description="echo rosbag depends topic")
  parser.add_argument("-b", "--bag", type=str, required=True, help="specify rosbag")
  parser.add_argument("-t", "--topics", nargs="+", type=str, help="specify topic")
  parser.add_argument('extra_args', nargs='*', help=argparse.SUPPRESS)
  args=parser.parse_args()

  if not os.path.isfile(args.bag):
      logging.error("%s is no found!" %args.bag)
      return

  echo = BagEcho(args.bag, args.topics)
  echo.echo_bag()

if __name__ == "__main__":
    main()