#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "math.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_waypoint/Target_waypoint_line.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI) 
#define MAX_WAYPOINT_NUM 300

enum FLAG_TYPE
{
    RESET = -1,
    STOP,
    RUN,
    ARRIVAL,
}flag_control;

#define NEW_WAYPOINT_TOR_XY 0.01
#define NEW_WAYPOINT_TOR_THETA 5.0

double target_move_speed = 0.25;

move_waypoint::Target_waypoint_line waypoints[MAX_WAYPOINT_NUM];

bool flag_new_waypoint     = false;
bool flag_new_start_id     = false;
bool flag_new_finish_id    = false;
int flag_waypoint_control  = STOP;
int no_waypoints           = -1;
int status_waypoint_move   =  0;
int waypoint_start_id	   = -1;
int waypoint_finish_id     =  0;

int waypoint_start_id_old  = -1;
int waypoint_finish_id_old =  0;

double roll, pitch, yaw;
geometry_msgs::Pose2D cur_pose2D;

move_waypoint::Target_waypoint_line target_waypoint_line_new;
move_waypoint::Target_waypoint_line target_waypoint_line_old;

typedef struct
{
   double a;
   double b;
   double c;
   double d;
} Line_Equation;

typedef struct
{
   double x;
   double y;
} Vector_2D;

void poseupdate_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    cur_pose2D.x       = msg->pose.pose.position.x;
    cur_pose2D.y       = msg->pose.pose.position.y;
    cur_pose2D.theta   = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    
    //cur_pose2D.theta = y;

    //printf("Received pose in = X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f\n", tx, ty, tz, RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
}

void status_waypoint_move_Callback(const std_msgs::Int8::ConstPtr& msg)
{
   status_waypoint_move = msg->data;
}

void waypoint_start_id_Callback(const std_msgs::Int8::ConstPtr& msg)
{
   waypoint_start_id = msg->data;

   if(waypoint_start_id != waypoint_start_id_old)
   {
      flag_new_start_id = true;
      waypoint_start_id_old = waypoint_start_id;
      printf("new start_id is received!\n");
   }
   else
   {
      flag_new_start_id = false;
   }
}

void waypoint_finish_id_Callback(const std_msgs::Int8::ConstPtr& msg)
{
   //waypoint_finish_id = msg->data;
   
   if(waypoint_finish_id != waypoint_finish_id_old)
   {
      flag_new_finish_id = true;
      waypoint_finish_id_old = waypoint_finish_id;
      printf("new finish_id is received!\n");
   }
   else
   {
      flag_new_finish_id = false;
   }
}

//bool Run;

void waypoint_run_command_Callback(const std_msgs::Bool::ConstPtr& msg)
{
   //Run = msg->data;
}

bool read_waypoint_file(std::string File_Name)
{
	bool success = false;
    int result   = -10;
   
    FILE *fp;
    
    fp = fopen(File_Name.c_str(), "r");
    no_waypoints = -1;

    if (fp == NULL)
    {
        ROS_ERROR("Waypoint   File does not exist!\n");
        success = false;
    }
    
    else
    {
      printf("File Open Success\n");

      do
      {
         ++no_waypoints;
         double x,y,theta;
         result = fscanf(fp, "%lf %lf %lf", &x , &y, &theta);
         waypoints[no_waypoints].waypoint_target_pose2d.x     = x;
         waypoints[no_waypoints].waypoint_target_pose2d.y     = y;
         waypoints[no_waypoints].waypoint_target_pose2d.theta = theta;
         //printf("target[%d].x = %.5lf   target[%d].y = %.5lf   target[%d].theta = %.5lf\n", no_waypoints, x, no_waypoints, y, no_waypoints, theta);
      }while(result != EOF);

      printf("\n");
      
      waypoints[no_waypoints].waypoint_start_pose2d.x     = 0;
      waypoints[no_waypoints].waypoint_start_pose2d.y     = 0;
      waypoints[no_waypoints].waypoint_start_pose2d.theta = 0;
      
      for(int i = 0; i<= no_waypoints + 1; i++)
      {
         waypoints[i + 1].waypoint_start_pose2d.x     = waypoints[i].waypoint_target_pose2d.x;
         waypoints[i + 1].waypoint_start_pose2d.y     = waypoints[i].waypoint_target_pose2d.y;
         waypoints[i + 1].waypoint_start_pose2d.theta = waypoints[i].waypoint_target_pose2d.theta;
         //printf("start[%d].x = %.5lf   start[%d].y = %.5lf   start[%d].theta = %.5lf\n", i, waypoints[i].waypoint_start_pose2d.x, i, waypoints[i].waypoint_start_pose2d.y, i, waypoints[i].waypoint_start_pose2d.theta);
      }
      
      waypoints[0].waypoint_start_pose2d.x     = 0;
      waypoints[0].waypoint_start_pose2d.y     = 0;
      waypoints[0].waypoint_start_pose2d.theta = 0;
      
      for(int i = 0; i <= no_waypoints; i++)
      {
         printf("Num : %d\n", i);
         printf("Start  : [%.4lf %.4lf %.4lf]\n", waypoints[i].waypoint_start_pose2d.x, waypoints[i].waypoint_start_pose2d.y, waypoints[i].waypoint_start_pose2d.theta);
         printf("Target : [%.4lf %.4lf %.4lf]\n\n", waypoints[i].waypoint_target_pose2d.x, waypoints[i].waypoint_target_pose2d.y, waypoints[i].waypoint_target_pose2d.theta);
      }      
    }
   
    fclose(fp);
    //success = true;    

    return success;
}

int main(int argc, char **argv)
{
    int count = 0;
    int start_waypoint_id  = 0;
    int finish_waypoint_id = 0;
    int current_waypoint_id = 0; //start_waypoint_id
   //bool run                = false;
   
    ros::init(argc, argv, "waypoint_control_system");
    ros::NodeHandle nh;

    std::string poseupdate_topic                 = "/poseupdate";
    std::string waypoint_file_path            	 = "/home/amap/sim_catkin_ws/src/waypoint_control_system/data/waypoint2.txt";
    std::string target_waypoint_line_topic       = "/target_waypoint_line";
    std::string status_waypoint_move_topic       = "/status_waypoint_move";
    std::string waypoint_start_id_topic       	 = "/start_waypoint_id_no";
    std::string waypoint_finish_id_topic      	 = "/finish_waypoint_id_no";
    std::string waypoint_run_command_topic     	 = "/flag/robot_run";
   
    ros::param::get("~poseupdate_topic",                 poseupdate_topic);
    ros::param::get("~waypoint_file_path",               waypoint_file_path);
    ros::param::get("~target_waypoint_line_topic",       target_waypoint_line_topic);
    ros::param::get("~status_waypoint_move_topic",       status_waypoint_move_topic);
    ros::param::get("~waypoint_start_id_topic",      	 waypoint_start_id_topic);
    ros::param::get("~waypoint_finish_id_topic",      	 waypoint_finish_id_topic);
    ros::param::get("~waypoint_run_command_topic",       waypoint_run_command_topic);
        
    ros::Subscriber sub_poseupdate                   	= nh.subscribe(poseupdate_topic, 1, poseupdate_Callback);
    ros::Subscriber sub_status_waypoint_move          	= nh.subscribe(status_waypoint_move_topic, 1, status_waypoint_move_Callback);
    ros::Subscriber sub_waypoint_start_id            	= nh.subscribe(waypoint_start_id_topic, 1, waypoint_start_id_Callback);
    ros::Subscriber sub_waypoint_finish_id              = nh.subscribe(waypoint_finish_id_topic, 1, waypoint_finish_id_Callback);
    ros::Subscriber sub_waypoint_run_command        	= nh.subscribe(waypoint_run_command_topic, 1, waypoint_run_command_Callback);
    ros::Publisher pub_target_waypoint_line             = nh.advertise<move_waypoint::Target_waypoint_line>(target_waypoint_line_topic, 1);    

    read_waypoint_file(waypoint_file_path);

    ros::Rate loop_rate(30.0); // 100HZ

   //current_waypoint_id = waypoint_start_id;
   //current_waypoint_id = 0;
   /*
   bool flag_new_start_id;
   {
      current_waypoint_id++;
   }
   */

	if(flag_new_start_id=false)
	{
		waypoint_start_id++;
	}
	
   while (ros::ok())
   {      
      if(flag_new_start_id = true)
      {
         current_waypoint_id = waypoint_start_id;
         flag_new_start_id = false;
      }
      
      //printf("current_waypoint_id = %d\n",current_waypoint_id);
      //printf("waypoint_start_id = %d\n\n",waypoint_start_id);
      
      if(count <= 50)
      {
         move_waypoint::Target_waypoint_line waypoint_line;
         waypoint_line = waypoints[current_waypoint_id];
         pub_target_waypoint_line.publish(waypoint_line);
      }
      
      if(status_waypoint_move == RUN)
      {
         printf("-------------------------------------\n");
         printf("           RUN             \n");
         printf("-------------------------------------\n");
         printf(" current_waypoint_id = %d\n",current_waypoint_id);

         printf("Num : %d\n", current_waypoint_id);
         printf("Target : [%.2lf %.2lf %.2lf]\n", waypoints[current_waypoint_id].waypoint_target_pose2d.x, waypoints[current_waypoint_id].waypoint_target_pose2d.y, waypoints[current_waypoint_id].waypoint_target_pose2d.theta);
         printf("Current: [%.2lf %.2lf %.2lf]\n", cur_pose2D.x, cur_pose2D.y, cur_pose2D.theta);
         printf("Start  : [%.2lf %.2lf %.2lf]\n", waypoints[current_waypoint_id].waypoint_start_pose2d.x, waypoints[current_waypoint_id].waypoint_start_pose2d.y, waypoints[current_waypoint_id].waypoint_start_pose2d.theta);
         printf("-------------------------------------\n");
         printf("count = %d\n",count);
      }
      else if(status_waypoint_move == ARRIVAL)
      {      
   
         printf("-------------------------------------\n");
         printf("        Arrival at waypoint %d       \n", current_waypoint_id);
         printf("-------------------------------------\n\n");

         ros::Duration(1.0).sleep(); //1 second delay
         count = 0;
      }
      else if(status_waypoint_move == STOP)
      {               
         printf("-------------------------------------\n");
         printf("           STOP             \n");
         printf("-------------------------------------\n\n");
      }
      else if(status_waypoint_move == RESET)
      {         
         printf("-------------------------------------\n");
         printf("           RESET             \n");
         printf("-------------------------------------\n\n");
      }
      else
      {
         
      }

      if(current_waypoint_id >= no_waypoints)
      {
         current_waypoint_id = no_waypoints;
      }
      
      
      if(count > 1000)
      {
         count = 1000;
      }

      ros::spinOnce();
      loop_rate.sleep();
      count++;
   }
   return 0;
}
