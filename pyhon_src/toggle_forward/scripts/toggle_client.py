#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyRequest

def main():
    rospy.init_node("toggle_client")
    client = rospy.ServiceProxy("toggle_forward", Empty)

    rospy.wait_for_service("toggle_forward")

    rate = rospy.Rate(2)

    while True:
        try:
            req = EmptyRequest()
            resp = client(req)
            rospy.loginfo("send toggle cmd")



        except rospy.ServiceException as e:
            rospy.loginfo("send toggle cmd failed: %s" %e)

        rate.sleep()

if __name__ == "__main__":
    main()