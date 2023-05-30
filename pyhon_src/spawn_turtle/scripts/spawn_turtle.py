#! /usr/bin/env python3

import rospy
import math
from turtlesim.srv import Spawn, SpawnRequest


def main():
    rospy.init_node("spawn_turtle")
    client = rospy.ServiceProxy("spawn", Spawn)
    rospy.wait_for_service("spawn")

    for i in range(10):
        try:

            req = SpawnRequest()
            req.x = 1 + i
            req.y = 1 + i
            req.theta = math.pi/2
            req.name = "Leo%s" %i

            resp = client(req)
            rospy.loginfo("spawned a turtle name %s" %resp.name)
        except rospy.ServiceException as e:
            rospy.loginfo("service call fail: %s" %e)

if __name__ == "__main__":
    main()