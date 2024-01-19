#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

///////////// CAMERA /////////////
#define TSL1401CL_SIZE 320
#define THRESHOLD 0.01
#define Line_Center 147
#define OFFSET 13

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < tsl1401cl_size; i++)
    {
        if (tsl1401cl_data[i] > threshold)
        {
            ThresholdData[i] = 255;
        }
        else
        {
            ThresholdData[i] = 0;
        }
    }
}


void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
}

int find_line_center()
{
    double centroid = 0.0;
    double mass_sum = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    if (mass_sum == 0)
    {
        mass_sum = 1;
    }

    centroid = centroid / mass_sum;

    return centroid;
}

double error_lane_old = 0.0;

geometry_msgs::Twist PID_lane_control(double Kp_lane, double Ki_lane, double Kd_lane)
{
    geometry_msgs::Twist cmd_vel;

    double lineCenter = find_line_center();

    double error_lane = Line_Center - lineCenter + OFFSET;
    double error_lane_d = error_lane - error_lane_old;
    double error__lane_sum = 0.0;

    error__lane_sum += error_lane;

    double steering_angle = Kp_lane * error_lane + Ki_lane * error__lane_sum + Kd_lane * error_lane_d;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = steering_angle;

    bool recognize_X = true;
    
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (LineSensor_threshold_Data[i] != 0)
        {
            recognize_X = false;
            break;
        }
    }

    if (recognize_X)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return cmd_vel;
    }

    error_lane_old = error_lane;

    return cmd_vel;
}

void print_lane()
{
    printf("Threshold Data: \n");

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        printf("%d ", LineSensor_threshold_Data[i]);
    }
    printf("\n");

    double centroid = find_line_center();
    printf("Line Centroid: %f\n", centroid);
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "pioneer_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);

    double Kp_lane = 0.0015;
    double Ki_lane = 0.0;
    double Kd_lane = 0.005;

    while (ros::ok())
    {
        print_lane();

        geometry_msgs::Twist cmd_vel = PID_lane_control(Kp_lane, Ki_lane, Kd_lane);

        cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
