#include <ros/ros.h>
#include <srv_self/AddTwoInts.h>
#include <cstdlib>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "add_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<srv_self::AddTwoInts>("add_ints");




    while (1) {
        srv_self::AddTwoInts::Request req;
        srv_self::AddTwoInts::Response resp;

        req.a = rand();
        req.b = rand();

        bool ok = client.call(req, resp);
        if (ok) {
            ROS_INFO("client: send a = %ld, b = %ld, receive sum = %ld", req.a, req.b, resp.sum);
        } else {
            ROS_ERROR("failed to send add_ints service");
        }

        sleep(2);

    }


    return 0;
}