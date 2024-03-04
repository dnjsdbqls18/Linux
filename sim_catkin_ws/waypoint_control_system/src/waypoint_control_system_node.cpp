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

bool flag_new_waypoint     = false;
int  flag_waypoint_control = STOP;

double roll, pitch, yaw;

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

double tx,ty,tz;

void poseupdate_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tx = msg->pose.pose.position.x;
    ty = msg->pose.pose.position.y;
    tz = msg->pose.pose.position.z;

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

move_waypoint::Target_waypoint_line waypoints[MAX_WAYPOINT_NUM];

int no_waypoints = -1;

bool read_waypoint_file(std::string File_Name)
{
	bool success = false;
    int result     = -10;
   
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
			printf("target[%d].x = %.5lf   target[%d].y = %.5lf   target[%d].theta = %.5lf\n", no_waypoints, x, no_waypoints, y, no_waypoints, theta);
		}while(result != EOF);

		printf("\n");
      
		waypoints[0].waypoint_start_pose2d.x     = 0;
		waypoints[0].waypoint_start_pose2d.y     = 0;
		waypoints[0].waypoint_start_pose2d.theta = 0;
      
		for(int i = 0; i<= no_waypoints + 1; i++)
		{
			waypoints[i + 1].waypoint_start_pose2d.x     = waypoints[i].waypoint_target_pose2d.x;
			waypoints[i + 1].waypoint_start_pose2d.y     = waypoints[i].waypoint_target_pose2d.y;
			waypoints[i + 1].waypoint_start_pose2d.theta = waypoints[i].waypoint_target_pose2d.theta;
			printf("start[%d].x = %.5lf   start[%d].y = %.5lf   start[%d].theta = %.5lf\n", i, waypoints[i].waypoint_start_pose2d.x, i, waypoints[i].waypoint_start_pose2d.y, i, waypoints[i].waypoint_start_pose2d.theta);
		}
    }
   
    fclose(fp);
    return success;
}

int main(int argc, char **argv)
{
    int count = 0;
	
	ros::init(argc, argv, "waypoint_control_system");
	ros::NodeHandle nh;

	std::string poseupdate_topic 				= "/poseupdate";
    std::string type_cmd_vel_topic 				= "/type_cmd_vel";
    std::string waypoint_file_path 				= "/home/amap/sim_catkin_ws/src/waypoint_control_system/data/waypoint1.txt";

    ros::param::get("~poseupdate_topic", 				poseupdate_topic);
    ros::param::get("~type_cmd_vel_topic", 				type_cmd_vel_topic);
    ros::param::get("~waypoint_file_path", 				waypoint_file_path);

    ros::Subscriber sub_poseupdate 						= nh.subscribe(poseupdate_topic, 1, poseupdate_Callback);
    ros::Publisher pub_type_cmd_vel						= nh.advertise<std_msgs::Int8>(type_cmd_vel_topic, 1);    


    if (read_waypoint_file(waypoint_file_path)) 
    {
        printf("Waypoint file read successfully.");
    } 
    else 
    {
        printf("Failed to read waypoint file.");
    }

    ros::Rate loop_rate(30.0); // 100HZ

	while (ros::ok())
	{
		std_msgs::Int8 type_cmd_vel;
		
		pub_type_cmd_vel.publish(type_cmd_vel);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
