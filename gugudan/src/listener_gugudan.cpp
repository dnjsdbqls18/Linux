#include <ros/ros.h>
#include <std_msgs/Int32.h>

void gugudanCallback(const std_msgs::Int32::ConstPtr& msg) 
{
    ROS_INFO("result: %d", msg->data);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "listener_gugudan");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gugudan", 50, gugudanCallback);

    ros::spin();

    return 0;
}
