#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <stdio.h>
#include <unistd.h>

void cb_get_pose(const turtlesim::Pose& msg) {
  printf("x = %f, y = %f, theta = %f\n", msg.x, msg.y, msg.theta);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sub_turtlesim_pose");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, cb_get_pose);
  
  ros::Rate loop_rate(30.0); //20.0HZ
  
  while(ros::ok())
  {
	  ros::spinOnce();
	  loop_rate.sleep();
  }
  
  return 0;
}
