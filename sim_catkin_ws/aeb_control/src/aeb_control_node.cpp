#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
}
  
int main(int argc, char **argv)
{
  int count = 0;
  
  geometry_msgs::Twist cmd_vel;
  
  ros::init(argc, argv, "aeb_control_sonar");
  ros::NodeHandle n;
  
  ros::Subscriber front_sonar_range_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
  ros::Publisher sonar_cmd_vel_pub      = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  ros::Rate loop_rate(30.0);  
  
  while (ros::ok())
  {
   ROS_INFO("Sonar Range: [%f]", front_sonar);

   if (front_sonar < 1.0)
   {
      cmd_vel.linear.x = 0.0;
   }
   else
   {
      cmd_vel.linear.x = 0.5;
   }
   
    sonar_cmd_vel_pub.publish(cmd_vel);
  
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
