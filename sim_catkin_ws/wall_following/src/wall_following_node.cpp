#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;
double error_old = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
  printf("Front_Sonar Range: [%f]\n", front_sonar);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  left_sonar = msg->range;
  printf("Left_Sonar Range:  [%f]\n", left_sonar);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  right_sonar = msg->range;
  printf("Right_Sonar Range: [%f]\n\n", right_sonar);
}

geometry_msgs::Twist PID_wall_following(double Kp, double Ki, double Kd)
{
  geometry_msgs::Twist cmd_vel;

  double error = left_sonar - right_sonar;
  double error_d = error - error_old;  
  double error_sum = 0.0;
  error_sum += error;  
  
  double steering_control = Kp * error + Ki * error_sum + Kd * error_d;
    
  if (front_sonar < 1.2)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  else
  {
    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = steering_control;
  }

  error_old = error;
  
  return cmd_vel;
}

int main(int argc, char **argv)
{
  int count = 0;
  
  ros::init(argc, argv, "wall_following");
  ros::NodeHandle n;
  
  ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
  ros::Subscriber left_sonar_sub = n.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
  ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, Right_Sonar_Callback);
  
  ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  ros::Rate loop_rate(30.0);  
  
  double Kp = 0.8;
  double Ki = 0.0;
  double Kd = 0.8;

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel = PID_wall_following(Kp, Ki, Kd);
    sonar_cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
