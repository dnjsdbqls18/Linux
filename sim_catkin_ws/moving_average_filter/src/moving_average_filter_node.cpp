#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

#define Value 5

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;

double sonar[Value] = {0,};

int n = 6;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
  
  if(n == 0)
  {
	n = 6;
  }
  n--;
  sonar[n] = front_sonar;
  //printf("Num of n : [%d]\n", n);
  //printf("Front_Sonar Range: [%f]\n", front_sonar);
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
  
  ros::init(argc, argv, "moving_average_filter");
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
	double x_current_avg;
	double x_previous_avg;
	
	double Standard = 5.0;
	
	int k=5;
	
	x_previous_avg = (sonar[k-5] + sonar[k-4] + sonar[k-3] + sonar[k-2] + sonar[k-1]) / Standard;
	
	x_current_avg = x_previous_avg + ((1 / Standard) * (sonar[5] - sonar[0]));
	
	printf("x_previous_avg = [%lf]\n\n",x_previous_avg);
	printf("x_current_avg = [%lf]\n\n",x_current_avg);
	
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
