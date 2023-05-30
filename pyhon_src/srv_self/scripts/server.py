#! /usr/bin/env python3

import rospy
from srv_self.srv import AddTwoInts, AddTwoIntsResponse

def add(req):
    resp = AddTwoIntsResponse()
    resp.sum = req.a + req.b
    rospy.loginfo("server: receive a = %ld, b = %ld, return sum = %ld" %(req.a, req.b, resp.sum))

    return resp

def main():
    rospy.init_node("add_server")

    srv = rospy.Service("add_ints", AddTwoInts, add)

    
    
    
    rospy.loginfo("ready to add ints..")
    rospy.spin()





if __name__ == "__main__":
    main()