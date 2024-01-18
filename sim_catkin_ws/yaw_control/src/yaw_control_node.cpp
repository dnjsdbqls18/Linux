#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double roll, pitch, yaw;
double error_old = 0.0;
double target_yaw_degree = 0.0;

double normalizeYaw(double yaw_deg)
{
    if (yaw_deg > 360)
    {
        yaw_deg = yaw_deg - 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg = yaw_deg + 360;
    }

    return yaw_deg;
}

geometry_msgs::Twist PID_yaw_control(double Kp, double Ki, double Kd)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_deg = RAD2DEG(yaw);
    yaw_deg = normalizeYaw(yaw_deg);

    double error = target_yaw_degree - yaw_deg;
    double error_sum = 0.0;
    double error_d = error - error_old;

    error_sum += error;

    double Steering_Angle = Kp * error + Ki * error_sum + Kd * error_d;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = Steering_Angle;

    if (fabs(error) < 0.5)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    error_old = error;

    return cmd_vel;
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    double yaw_deg = normalizeYaw(RAD2DEG(yaw));

    printf("%f\n", yaw_deg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yaw_control");
    ros::NodeHandle n;
    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback);
    ros::Publisher yaw_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);

    double Kp = 0.02;
    double Ki = 0.0;
    double Kd = 0.5;

    int count = 0;

    while (ros::ok())
    {
        geometry_msgs::Twist cmd_vel = PID_yaw_control(Kp, Ki, Kd);
        yaw_cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
