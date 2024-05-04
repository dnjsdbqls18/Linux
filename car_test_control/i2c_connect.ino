#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x05

#define NEUTRAL_ANGLE 90
#define RC_SERVO_PIN   2

#define encodPinA1   2
#define encodPinB1   3
#define MOTOR_DIR    4
#define MOTOR_PWM    5

int Steering_Angle = 0;
Servo Steeringservo;

union 
{
  short data;
  byte bytes[2];
} car_speed, car_servo;

void setup() 
{
  #if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  #endif
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  
  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);   
}

void motor_control(int speed)
{
  if (speed >= 0) 
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, speed);
  }
  else 
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, -speed);
  }
  Serial.print("speed: ");
  Serial.println(speed);
  Serial.println();    
}

void loop() 
{
  delay(100);
}

void receiveData(int byteCount) 
{
  if (Wire.available() >= 9) 
  {
    byte receivedData[9];
    for (int i = 0; i < 9; i++) 
    {
      receivedData[i] = Wire.read(); 
    }

    if (receivedData[0] == '#' && receivedData[1] == 'C' && receivedData[8] == '*') 
    {
      car_servo.bytes[0] = receivedData[2];
      car_servo.bytes[1] = receivedData[3];
      short angle = car_servo.data;

      car_speed.bytes[0] = receivedData[4];
      car_speed.bytes[1] = receivedData[5];
      float speed = car_speed.data;
      
      Steeringservo.write(NEUTRAL_ANGLE + angle);
  
      Serial.print("angle_offset: ");
      Serial.println(angle);
      Serial.print("angle: ");
      Serial.println(NEUTRAL_ANGLE + angle);
      Serial.println();
      delay(1000);
      
      motor_control(speed);
    } 
    else 
    {
      Serial.println("Invalid protocol");
    }
  }
}

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

//static const char *deviceName = "/dev/i2c-0";
unsigned char protocol_data[9] = {'#','C',0,0,0,0,0,0,'*'}; // start byte '#' - end bytte '*'

union Steering
{
	short steering_angle_data;
	char angle_byte[2];
};
union Steering s;

union Car_Speed
{
	int speed_data;
	char speed_byte[2];
};
union Car_Speed c;

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
	//steering_angle = msg->data;
}

void car_speed_Callback(const std_msgs::Float32::ConstPtr& msg)
{

}

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double linear_data;
    double angular_data;
   
    linear_data  = cmd_vel.linear.x ;
    angular_data = cmd_vel.angular.z ;

    if(linear_data >=  Max_Car_Speed)
    {
		linear_data = Max_Car_Speed;
	}
    if(linear_data <=  Min_Car_Speed)
    {
		linear_data = Min_Car_Speed;
	}

    c.data = (short)linear_data;
    
    if(angular_data <= Max_R_Angle)  
    {
		angular_data = Max_R_Angle;
	}
    if(angular_data >= Max_L_Angle)
    {
		angular_data = Max_L_Angle;
	}
	  
    s.data  = (short)angular_data;
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
         printf("Unable to open I2C");
        return -1;
    }
    else
    {
        printf("I2C is Connected");
    }

    while (ros::ok())
    {
		protocal_data[0] = '#';
		protocal_data[1] = 'C';
		protocal_data[2] = s.angle_byte[0];
		protocal_data[3] = s.angle_byte[1];
		protocal_data[4] = c.speed_byte[0];
		protocal_data[5] = c.speed_byte[1];
		protocal_data[6] = 0;  
		protocal_data[7] = 0;    
		protocal_data[8] = '*';
		
		write(file_I2C, protocal_data, 9);
	
		printf("car_speed : %d\n", s.angle_byte);
		printf("steering_angle : %d \n\n", c.speed_byte);
        ros::spinOnce();
        loop_rate.sleep();
    }
    close_I2C(file_I2C);

    return 0;
}

/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp: In function ‘void cmd_vel_Callback(const ConstPtr&)’:
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:93:20: error: ‘cmd_vel’ was not declared in this scope
     linear_data  = cmd_vel.linear.x ;
                    ^~~~~~~
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:105:7: error: ‘union Car_Speed’ has no member named ‘data’
     c.data = (short)linear_data;
       ^~~~
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:116:7: error: ‘union Steering’ has no member named ‘data’
     s.data  = (short)angular_data;
       ^~~~
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp: In function ‘int main(int, char**)’:
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:155:3: error: ‘protocal_data’ was not declared in this scope
   protocal_data[0] = '#';
   ^~~~~~~~~~~~~
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:155:3: note: suggested alternative: ‘protocol_data’
   protocal_data[0] = '#';
   ^~~~~~~~~~~~~
   protocol_data
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:167:42: warning: format ‘%d’ expects argument of type ‘int’, but argument 2 has type ‘char*’ [-Wformat=]
   printf("car_speed : %d\n", s.angle_byte);
                                          ^
/home/amap/w_catkin_ws/src/car_control/src/car_control_node.cpp:168:50: warning: format ‘%d’ expects argument of type ‘int’, but argument 2 has type ‘char*’ [-Wformat=]
   printf("steering_angle : %d \n\n", c.speed_byte);
                                                  ^
car_control/CMakeFiles/car_control_node.dir/build.make:62: recipe for target 'car_control/CMakeFiles/car_control_node.dir/src/car_control_node.cpp.o' failed
make[2]: *** [car_control/CMakeFiles/car_control_node.dir/src/car_control_node.cpp.o] Error 1
CMakeFiles/Makefile2:479: recipe for target 'car_control/CMakeFiles/car_control_node.dir/all' failed
make[1]: *** [car_control/CMakeFiles/car_control_node.dir/all] Error 2
Makefile:140: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j4 -l4" failed
