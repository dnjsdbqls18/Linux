#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SLAVE_ADDRESS 0x05 // 아두이노의 I2C 주소

//static const char *deviceName = "/dev/i2c-0";

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

unsigned char protocol_data[7] = {'#',0,0,0,0,0,'*'}; // start byte '#' - end bytte '*'
int file_I2C;

int open_I2C(void) 
{
    int file;
    const char *deviceName = "/dev/i2c-0"; // 사용할 I2C 장치 파일

    // I2C 장치 열기
    if ((file = open(deviceName, O_RDWR)) < 0) 
    {
        fprintf(stderr, "Failed to access %s\n", deviceName);
        exit(1);
    }
    printf("I2C: Connected\n");

    // I2C 장치와 통신 설정
    printf("I2C: acquiring bus to 0x%x\n", SLAVE_ADDRESS);
    if (ioctl(file, I2C_SLAVE, SLAVE_ADDRESS) < 0) 
    {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", SLAVE_ADDRESS);
        exit(1);
    }

    return file;
}

void close_I2C(int fd)
{
   close(fd);
}

double steering_angle = 0.0;

void steering_angle_Callback(const std_msgs::Float32::ConstPtr& msg)
{
	steering_angle = msg->data;
   //right_sonar = msg->range;
   //printf("Right_Sonar Range: [%f]\n\n", right_sonar);
}

void car_speed_Callback(const std_msgs::Float32::ConstPtr& msg)
{

}

double buffer_1 = 0.0;
double buffer_2 = 0.0;

int main(int argc, char **argv)
{
	int count = 0;
  
	ros::init(argc, argv, "car_control");
	ros::NodeHandle nh;
	
	std::string steering_angle_topic	 = "/steering_angle";
	std::string car_speed_topic 		 = "/car_speed";
	
	ros::param::get("~steering_angle_topic",		steering_angle_topic);
	ros::param::get("~car_speed_topic",				car_speed_topic);
	ros::param::get("~buffer_1",		     		buffer_1);
	ros::param::get("~buffer_2",					buffer_2);
	
	ros::Subscriber steering_angle_sub 			= nh.subscribe(steering_angle_topic, 1, steering_angle_Callback);
	ros::Subscriber car_speed_sub 				= nh.subscribe(car_speed_topic, 1, car_speed_Callback);

	//ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>(sonar_cmd_vel_topic, 1);
	
	ros::Rate loop_rate(30.0);  
	
    // I2C 장치 열기 및 설정
    file_I2C = open_I2C();
    if(file_I2C < 0)
    {
        ROS_ERROR_STREAM("Unable to open I2C");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("I2C is Connected");
    }

    char buffer[2];
    buffer[0] = 30; // First byte to send
    buffer[1] = 0; // Second byte to send
    if (write(file_I2C, buffer, 2) != 2)
    {
        perror("Failed to write to the I2C bus.");
        exit(1);
    }

    // Read the data from the I2C bus
    if (read(file_I2C, buffer, 2) != 2) 
    {
        perror("Failed to read from the I2C bus.");
        exit(1);
    }

    int received_angle_offset = (buffer[0] << 8) | buffer[1]; // Combine the two bytes into an integer
    printf("Received angle offset: %d\n", received_angle_offset);
    steering_angle = received_angle_offset;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close the I2C device
    close_I2C(file_I2C);

    return 0;
}
