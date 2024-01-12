#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

geometry_msgs::Pose2D current_pos2d_msg;

void cb_get_pose(const turtlesim::Pose& msg)
{
  current_pos2d_msg.x     = msg.x;
  current_pos2d_msg.y     = msg.y;
  current_pos2d_msg.theta = msg.theta;
}

int main(int argc, char **argv) 
{
  double target_heading_angle_degree = 90;
  
  ros::init(argc, argv, "sub_turtlesim_pose");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, cb_get_pose);
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 5);
  
  geometry_msgs::Twist cmd_vel_msg;

  ros::Rate loop_rate(30.0);

  while(ros::ok())
  {
    printf("x = %f, y = %f, theta = %f\n", current_pos2d_msg.x , current_pos2d_msg.y , RAD2DEG(current_pos2d_msg.theta));

    double heading_error = target_heading_angle_degree - RAD2DEG(current_pos2d_msg.theta);

    if (heading_error > 180)
    {
      heading_error -= 360;
    }
    else if (heading_error < -180) 
    {
      heading_error += 360;
    }

    cmd_vel_msg.linear.x = 0.00;
    cmd_vel_msg.angular.z = 0.5 * (heading_error / 180.0);

    pub_cmd_vel.publish(cmd_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
