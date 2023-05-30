#include <ros/ros.h>
#include <srv_self/AddTwoInts.h>


bool add(srv_self::AddTwoInts::Request &req, srv_self::AddTwoInts::Response &resp) {
    resp.sum = req.a + req.b;

    ROS_INFO("server: receive a = %ld, b = %ld, return sum = %ld", req.a, req.b, resp.sum);

    return true;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "add_server");
    ros::NodeHandle nh;

    ros::ServiceServer srv = nh.advertiseService("add_ints", &add);


    ROS_INFO("ready to add two ints..");
    ros::spin();




    return 0;
}