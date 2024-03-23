#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;

int n = 0;
bool calculate = false;
double x_sum = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
  //printf("Front_Sonar Range: [%f]\n", front_sonar);
  n++;
  x_sum += front_sonar;
  calculate = true;
  //printf("%d\n\n",n);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  left_sonar = msg->range;
  //printf("Left_Sonar Range:  [%f]\n", left_sonar);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  right_sonar = msg->range;
  //printf("Right_Sonar Range: [%f]\n\n", right_sonar);
}

int main(int argc, char **argv)
{
  int count = 0;
  double x_n, x_avg, x_n_avg;
  
  ros::init(argc, argv, "kalman_filter");
  ros::NodeHandle nh;

  std::string front_sonar_topic   			 = "/range_front";
  std::string left_sonar_topic 			     = "/range_front_left";
  std::string right_sonar_topic 		     = "/range_front_right";
  
  ros::param::get("~front_sonar_topic",			front_sonar_topic);
  ros::param::get("~left_sonar_topic",			left_sonar_topic);
  ros::param::get("~right_sonar_topic",			right_sonar_topic);

  ros::Subscriber sub_front_sonar 			= nh.subscribe(front_sonar_topic, 1, Front_Sonar_Callback);
  ros::Subscriber sub_left_sonar  			= nh.subscribe(left_sonar_topic, 1, Left_Sonar_Callback);
  ros::Subscriber sub_right_sonar_			= nh.subscribe(right_sonar_topic, 1, Right_Sonar_Callback);

  ros::Rate loop_rate(30.0);

  while (ros::ok())
  {
    x_n = front_sonar;
	if(calculate = true)
	{
		if (n > 0)
		{
		  calculate = false;

		  printf("if calculate	       : [%d]\n", calculate);
		  x_avg = (x_avg * (n - 1) + x_n) / n;
		} 
		else 
		{
					  calculate = false;

		  x_avg = x_n;
		}
		printf("calculate	       : [%d]\n", calculate);

		printf("num	       : [%d]\n", n);
		printf("Front_Sonar: [%f]\n", x_n);
		printf("x_avg      : [%f]\n\n", x_avg);
	}
	
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
