#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_gugudan");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 1;

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;

    for (int i = 1; i <= 9; ++i)
    {
      ss << count << " x " << i << " = " << count * i << "\n";
    }

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    if (count > 9)
    {
      count = 1;
    }
  }

  return 0;
}
