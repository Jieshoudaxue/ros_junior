#! /usr/bin/env python3

import os
import argparse
import rosbag
import logging

logging.basicConfig(level=logging.DEBUG)

class BagFilter(object):
  def __init__(self, input_bag, output_bag, start_offset, end_offset, topics):
    self.__input_bag = input_bag
    self.__output_bag = output_bag
    self.__start_offset = start_offset
    self.__end_offset = end_offset
    self.__topics = topics

  def filter_bag(self):
    inbag = rosbag.Bag(self.__input_bag, "r")
    start_time = inbag.get_start_time() + self.__start_offset
    end_time = inbag.get_end_time() - self.__end_offset

    with rosbag.Bag(self.__output_bag, "w") as outbag:
      for topic, msg, t in inbag.read_messages(topics=self.__topics, raw=False):
        if start_time <= t.to_sec() <= end_time:
          outbag.write(topic, msg, t)

      logging.info("------------[%s summary]------------" %self.__output_bag)
      logging.info("time span %f [origin %f]" %(end_time-start_time, inbag.get_end_time()-inbag.get_start_time()))
      topic_list = [topic for topic in outbag.get_type_and_topic_info()[1].keys()]
      logging.info("topic list %s" %topic_list)

    inbag.close()

def main():
  parser = argparse.ArgumentParser(description="filter rosbag depends time offset and topics")
  parser.add_argument("-i", "--input_bag", type=str, required=True, help="specify input rosbag")
  parser.add_argument("-o", "--output_bag", type=str, required=True, help="specify output rosbag")
  parser.add_argument("-s", "--start_offset", type=int, default=0, help="specify start offset time")
  parser.add_argument("-e", "--end_offset", type=int, default=0, help="specify end offset time")
  parser.add_argument("-t", "--topics", nargs="+", type=str, help="specify topic")
  parser.add_argument('extra_args', nargs='*', help=argparse.SUPPRESS)
  args=parser.parse_args()

  if not os.path.isfile(args.input_bag):
    logging.error("%s is no found!" %args.input_bag)
    return

  filter = BagFilter(args.input_bag, args.output_bag, args.start_offset, args.end_offset, args.topics)
  filter.filter_bag()


if __name__ == "__main__":
    main()