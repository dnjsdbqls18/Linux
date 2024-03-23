#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_2d_msgs/Point2D.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define ROBOT_WIDTH     0.2
#define ROBOT_WIDTH_TOR 0.1

int laser_point_no = 358;

nav_2d_msgs::Point2D obstacle_2d[358];

typedef struct
{
    double x1;
    double y1;
    double x2;
    double y2;
} ROI;

ROI front_ROI;
ROI front_right_ROI;
ROI front_left_ROI;

bool read_roi_data(std::string file_name)
{
    FILE *fp = fopen(file_name.c_str(), "r");
    
    if (fp == NULL)
    {
        ROS_ERROR("ERROR: File does not exist\n");
        return false;
    }
    else
    {
        bool success = true;
        printf("File open success\n");

        int result = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &front_ROI.x1, &front_ROI.y1, &front_ROI.x2, &front_ROI.y2, &front_right_ROI.x1, &front_right_ROI.y1, &front_right_ROI.x2, &front_right_ROI.y2, &front_left_ROI.x1, &front_left_ROI.y1, &front_left_ROI.x2, &front_left_ROI.y2);

        if (result != 12)
        {
            ROS_ERROR("ERROR: Cannot read ROI data\n");
            success = false;
        }

        fclose(fp);
        return success;
    }
}

bool front_obstacle_detection(int count)   // x방향 projection
{
    int sum = 0;
    for (int i = 0; i < count; i++)
    {
        if (obstacle_2d[i].x <= front_ROI.x1 && obstacle_2d[i].x >= front_ROI.x2 &&
            obstacle_2d[i].y <= front_ROI.y1 && obstacle_2d[i].y >= front_ROI.y2)
        {
            sum++;
        }
    }

    if (sum >= 10)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    ROS_INFO("LaserScan (val,angle)=(%f,%f", scan->range_min, scan->angle_min);
    int count = (int)(360. / RAD2DEG(scan->angle_increment));
    for (int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

        obstacle_2d[i].x = scan->ranges[i] * cos(scan->angle_min + scan->angle_increment * i);
        obstacle_2d[i].y = scan->ranges[i] * sin(scan->angle_min + scan->angle_increment * i);

        printf("%3d (%.2f  %.2lf) = (%.2f  %.2lf) \n", i, degree, scan->ranges[i], obstacle_2d[i].x, obstacle_2d[i].y);
    }
    printf("obstacle_detection_flag = %1d \n", front_obstacle_detection(count));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan");
    ros::NodeHandle n;

    std::string laser_scan_topic = "/scan";
    std::string ROI_file = "/home/amap/sim_catkin_ws/src/Lidar/laser_scan/src/ROI.txt";

    ros::Subscriber laser_scan_sub = n.subscribe(laser_scan_topic, 1, laser_scan_Callback);

    ros::param::get("~laser_scan_topic", laser_scan_topic);
    ros::param::get("~ROI_file", ROI_file);

    bool flag_file_read = read_roi_data(ROI_file);

    if (!flag_file_read)
    {
        ROS_ERROR("ERROR: read ROI file. EXIT.\n");
        return 1;
    }

    ros::Rate loop_rate(5.0);

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
