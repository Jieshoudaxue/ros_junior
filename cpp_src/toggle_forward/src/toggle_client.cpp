#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "toggle_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("toggle_forward");

    ros::service::waitForService("toggle_forward");

    while (1) {
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response resp;

        bool ok = client.call(req, resp);
        if (ok) {
            ROS_INFO("send toggle cmd");
        } else {
            ROS_ERROR("failed to send cmd");
        }

        sleep(2);

    }





    return 0;
}