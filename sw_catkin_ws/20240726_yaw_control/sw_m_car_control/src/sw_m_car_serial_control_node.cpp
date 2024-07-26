/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
/*******************************************************************************
 *  INCLUDE #define POLYNORMIAL 0xA001FILES
 *******************************************************************************
 */
 
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>  // For errno
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>                   // B115200, CS8 등 상수 정의
#include <fcntl.h>                     // O_RDWR , O_NOCTTY 등의 상수 정의
#include <time.h>
#include <math.h>
#include <mutex>  // For mutex

#include <sys/stat.h> // For checking file permissions

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define _USE_MATH_DEFINES

#define MAX_MOTOR_SPEED_RPM  150
#define Neutral_Steer_angle   0

#define LOOP_RATE             20

#define NO_RECEIVE_BUFFER      7

/*****************************
* Robot STEER Control define 
******************************/
#define MAX_R_ANGLE -25
#define MAX_L_ANGLE  25

/*****************************
* Robot SPEED Control define 
******************************/
#define MAX_CAR_SPEED  240
#define MIN_CAR_SPEED -240

/*****************************
* Robot parameter define 
******************************/
#define ROBOT_WHEEL_BASE

typedef unsigned char BYTE;

#define DEG2RAD(x) (M_PI/180.0*(x))
#define RAD2DEG(x) ((x)*180.0/M_PI)

#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 

union
{
	float data ;
	char  bytedata[4];

} m_car_speed_float , m_current_car_speed_float;

union
{
	short data ;
	char  bytedata[2];
    
} m_car_angle_int16 , m_car_speed_int16, m_current_car_speed_int16;

int m_acceleration =  1;  //50pwm/s
int m_deceleration =  1;  //50pwm/s

#define BAUDRATE          B115200
//#define SERIAL_DEVICE   "/dev/CleanRobotHW"  
//#define SERIAL_DEVICE   "/dev/ttyACM0"  
#define SERIAL_DEVICE   "/dev/ttyUSB0"  

static int uart_fd;
unsigned char protocal_test[NO_RECEIVE_BUFFER] ={0,};
unsigned char read_buf[NO_RECEIVE_BUFFER];

std::mutex serial_mutex;

void write_serial(unsigned char *buf, int len)
{
    std::lock_guard<std::mutex> lock(serial_mutex);  // Ensure thread safety
	for(int i=0; i<len; i++)
	{
		write(uart_fd, &buf[i], 1);
	}
} 

int init_serial_port(std::string m_com_port)
{
	// Check if the serial port exists
	struct stat st;
	
	if (stat(m_com_port.c_str(), &st) != 0) 
	{
		printf("Error: Serial port %s does not exist.\n", m_com_port.c_str());
		return -1;
	}

	int serial_port = open(m_com_port.c_str(), O_RDWR | O_NOCTTY);
	
	if (serial_port < 0) 
	{
		printf("Error %i opening %s: %s\n", errno, m_com_port.c_str(), strerror(errno));
		return -1;
	}

	// Create new termios struct, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if (tcgetattr(serial_port, &tty) != 0) 
	{
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		close(serial_port);
		return -1;
	}

	// Configure the serial port settings
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, BAUDRATE);
	cfsetospeed(&tty, BAUDRATE);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
	{
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		close(serial_port);
		return -1;
	}

	return serial_port;
}


void *readserial_thread(void *pt)
{

	int num_bytes = -1;
	unsigned char insert_buf; 

	while(1)
	{ 
		while( (num_bytes = read(uart_fd, &insert_buf, 1) ) > 0 )	
		{
			//printf("No read %d\n",num_bytes);
		  
			for(int i=0;i<NO_RECEIVE_BUFFER-1;i++)
			{
				read_buf[i]=read_buf[i+1];
			}
			
			read_buf[NO_RECEIVE_BUFFER-1]=insert_buf;
			
			for(int i=0;i<NO_RECEIVE_BUFFER;i++)	    
			{
				printf("%X", read_buf[i]);
			}
			printf("\n");
		}
    }
}  


void send_serial_data(void)
{
	unsigned short protocal_crc16;

	protocal_test[0] = '#';
	protocal_test[1] = 'C';

	protocal_test[2] = m_car_angle_int16.bytedata[0];
	protocal_test[3] = m_car_angle_int16.bytedata[1];

	//protocal_test[4] = m_current_car_speed_int16.bytedata[0];
	//protocal_test[5] = m_current_car_speed_int16.bytedata[1];  
	protocal_test[4] = m_current_car_speed_int16.bytedata[0];
	protocal_test[5] = m_current_car_speed_int16.bytedata[1];  
	protocal_test[6] = '*';

	printf("steer %3d %3d \n", m_car_angle_int16.data, Neutral_Steer_angle);
	//printf("speed %3d \n", m_current_car_speed_int16.data);
	printf("speed %3d \n\n", m_car_speed_int16.data);


	//printf("protocal CRC16 %X \n", protocal_crc16);
	write_serial(protocal_test,8);
}

void cmd_callback(const geometry_msgs::Twist & cmd_input)
{
	/*
	float angular_temp;
	float linear_temp;

	linear_temp = cmd_input.linear.x ;//RPM
	angular_temp = cmd_input.angular.z ;//rad/s


	if(angular_temp >= MAX_R_ANGLE)  
	{
		angular_temp = MAX_R_ANGLE;
	}
	if(angular_temp <= MAX_L_ANGLE)  
	{
		angular_temp = MAX_L_ANGLE;   
	}
	m_car_angle_int16.data  = (short)angular_temp; //+ Neutral_Steer_angle; 
	   

	if(linear_temp >=  MAX_CAR_SPEED)    
	{
		linear_temp = MAX_CAR_SPEED;
	}
	if(linear_temp <=  MIN_CAR_SPEED)    
	{
		linear_temp = MIN_CAR_SPEED;
	}
	m_car_speed_int16.data = (short)linear_temp;      
	* */    
}

void car_speed_callback(const std_msgs::Int16 & speed_input)
{
	int speed_temp;

	speed_temp = speed_input.data;//m/s

	if(speed_temp >=  MAX_CAR_SPEED)    
	{
		speed_temp = MAX_CAR_SPEED;
	}
	if(speed_temp <=  MIN_CAR_SPEED)    
	{
		speed_temp = MIN_CAR_SPEED;
	}
	m_car_speed_int16.data = speed_temp;            
}

void car_steer_callback(const std_msgs::Int16 & steer_input)
{
	int steer_temp;

	steer_temp = -steer_input.data;//rad/s

	if(steer_temp <= MAX_R_ANGLE)  
	{
		steer_temp = MAX_R_ANGLE;
	}
	if(steer_temp >= MAX_L_ANGLE)  
	{
		steer_temp = MAX_L_ANGLE;   
	}
	m_car_angle_int16.data  = (short)steer_temp; 	  
}

void car_speed_profile_control(void)
{
  
	if(m_current_car_speed_int16.data > m_car_speed_int16.data)   //현재 속도가 명령 속도 보다 클 때 , 감속 조건  m_car_speed 가 명령어임
	{
		m_current_car_speed_int16.data -= m_deceleration;
		m_current_car_speed_int16.data = (m_current_car_speed_int16.data <= m_car_speed_int16.data ) ? m_car_speed_int16.data : m_current_car_speed_int16.data;
	}
	
	else if(m_current_car_speed_int16.data < m_car_speed_float.data) //현재 속도가 명령속도 보다 클때 , 가속 조건
	{
		m_current_car_speed_int16.data += m_acceleration; 
		m_current_car_speed_int16.data = (m_current_car_speed_int16.data >= m_car_speed_float.data ) ? m_car_speed_float.data : m_current_car_speed_int16.data;
	}
	
	else 
	{
		m_current_car_speed_int16.data = m_car_speed_int16.data;
	}   
	  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sw_m_car_serial_control node");
	ros::NodeHandle n;

	//std::string com_port = "/dev/ttyS0";
	std::string com_port = "/dev/ttyUSB0";
	std::string cmd_vel_topic = "cmd_vel";
	std::string cmd_car_speed = "Car_Control_cmd/Speed_Int16";
	std::string cmd_car_angle = "Car_Control_cmd/SteerAngle_Int16";
	std::string odom_pub_topic = "odom";
	std::string wheel_speed_pub_topic = "wheel_speed";
	std::string wheel_encoder_pub_topic = "wheel_encoder";
	std::string odom_frame_id = "odom";  
	std::string odom_child_frame_id = "base_footprint";

	ros::param::get("~com_port",     com_port);
	ros::param::get("~cmd_vel_topic",  cmd_vel_topic);
	ros::param::get("~m_accleration",  m_acceleration); //acceleraton  parameter
	ros::param::get("~m_deceleration", m_deceleration); //deceleration parameter

	ros::Subscriber sub           = n.subscribe(cmd_vel_topic, 20, cmd_callback);
	ros::Subscriber sub_wp_speed  = n.subscribe(cmd_car_speed, 20, car_speed_callback);
	ros::Subscriber sub_wp_angle = n.subscribe(cmd_car_angle, 20, car_steer_callback);

	uart_fd = init_serial_port(com_port); 
	ros::Rate loop_rate(LOOP_RATE); //10.0HZ

	m_car_speed_float.data = 0;
	m_car_angle_int16.data = 0;

	while(ros::ok())
	{
		//printf("test\n");
		//printf("S:%4.2lf | A:%3d\n", m_current_car_speed_float.data, m_car_angle_int16.data);
		car_speed_profile_control(); // 로봇의 가감속 속도 제어를 함    
		send_serial_data();     
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
