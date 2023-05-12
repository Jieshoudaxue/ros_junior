#! /usr/bin/env python3

import rospy
import random
from srv_self.srv import *


def main():
    rospy.init_node("add_client")

    client = rospy.ServiceProxy("add_ints", AddTwoInts)

    rospy.wait_for_service("add_ints")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            req = AddTwoIntsRequest()
            req.a = random.randint(1, 100)
            req.b = random.randint(1, 100)

            resp = client(req)

            rospy.loginfo("client: send a = %ld, b = %ld, receive sum = %ld" %(req.a, req.b, resp.sum))
        except rospy.ServiceException as e:
            rospy.loginfo("service call fail: %s" %e)

        rate.sleep()


if __name__ == "__main__":
    main()