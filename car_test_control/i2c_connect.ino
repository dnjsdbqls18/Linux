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

#define SLAVE_ADDRESS 0x05

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define Max_R_Angle 45
#define Max_L_Angle -45

#define Max_Car_Speed  255
#define Min_Car_Speed  -255

unsigned char protocol_data[9] = {'#','C',0,0,0,0,0,0,'*'}; // 시작 바이트 '#' - 끝 바이트 '*'

union 
{
	short steering_angle_data;
	char angle_byte[2];
} Steering;

union 
{
	int speed_data;
	char speed_byte[2];
} Car_Speed;

int file_I2C;

int open_I2C(void) 
{
    int file;
    const char *deviceName = "/dev/i2c-0"; // 사용할 I2C 장치 파일

    // I2C 장치 열기
    if ((file = open(deviceName, O_RDWR)) < 0) 
    {
        fprintf(stderr, "%s에 접근 실패\n", deviceName);
        exit(1);
    }
    printf("I2C: 연결됨\n");

    // I2C 장치와 통신 설정
    printf("I2C: 0x%x에 대한 버스 획득\n", SLAVE_ADDRESS);
    if (ioctl(file, I2C_SLAVE, SLAVE_ADDRESS) < 0) 
    {
        fprintf(stderr, "I2C: 버스 접근/스레이브 0x%x에 대한 통신 실패\n", SLAVE_ADDRESS);
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
	//steering_angle = msg->data;
}

void car_speed_Callback(const std_msgs::Float32::ConstPtr& msg)
{

}

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double linear_data;
    double angular_data;
   
    linear_data  = msg->linear.x ;
    angular_data = msg->angular.z ;

    if(linear_data >=  Max_Car_Speed)
    {
		linear_data = Max_Car_Speed;
	}
    if(linear_data <=  Min_Car_Speed)
    {
		linear_data = Min_Car_Speed;
	}

    Car_Speed.speed_data = (short)linear_data;
    
    if(angular_data <= Max_R_Angle)  
    {
		angular_data = Max_R_Angle;
	}
    if(angular_data >= Max_L_Angle)
    {
		angular_data = Max_L_Angle;
	}
	  
    Steering.steering_angle_data  = (short)angular_data;
}

int main(int argc, char **argv)
{
	int count = 0;
  
	ros::init(argc, argv, "car_control");
	ros::NodeHandle nh;
	
	std::string steering_angle_topic	 = "/steering_angle";
	std::string car_speed_topic 		 = "/car_speed";
	std::string cmd_vel_topic 			 = "/cmd_vel";

	ros::param::get("~steering_angle_topic",		steering_angle_topic);
	ros::param::get("~car_speed_topic",				car_speed_topic);
	ros::param::get("~cmd_vel_topic",				cmd_vel_topic);

	ros::Subscriber steering_angle_sub 			= nh.subscribe(steering_angle_topic, 1, steering_angle_Callback);
	ros::Subscriber car_speed_sub 				= nh.subscribe(car_speed_topic, 1, car_speed_Callback);
    ros::Subscriber car_control_sub 			= nh.subscribe(cmd_vel_topic, 1, cmd_vel_Callback);

	//ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
	
	ros::Rate loop_rate(30.0);  
	
    file_I2C = open_I2C();
    if(file_I2C < 0)
    {
         printf("I2C를 열 수 없습니다.");
        return -1;
    }
    else
    {
        printf("I2C가 연결되었습니다.");
    }

    while (ros::ok())
    {
		protocol_data[0] = '#';
		protocol_data[1] = 'C';
		protocol_data[2] = Steering.angle_byte[0];
		protocol_data[3] = Steering.angle_byte[1];
		protocol_data[4] = Car_Speed.speed_byte[0];
		protocol_data[5] = Car_Speed.speed_byte[1];
		protocol_data[6] = 0;  
		protocol_data[7] = 0;    
		protocol_data[8] = '*';
		
		write(file_I2C, protocol_data, 9);
	
		printf("car_speed : %d\n", Car_Speed.speed_data);
		printf("steering_angle : %d \n\n", Steering.steering_angle_data);
		
        ros::spinOnce();
        loop_rate.sleep();
    }
    //close_I2C(file_I2C);

    return 0;
}
