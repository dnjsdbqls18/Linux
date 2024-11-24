#include "ros/ros.h" 

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>                   // B115200, CS8 등 상수 정의
#include <fcntl.h>                     // O_RDWR , O_NOCTTY 등의 상수 정의
#include <time.h>
#include <math.h>

//multi thread
#include <pthread.h>

//#include "SimpleKalmanFilter.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 

 
typedef unsigned char BYTE;

union
{
    float data ;
    char  bytedata[4];
    
} m_robot_speed , m_current_robot_speed;

union
{
    short data ;
    char  bytedata[2];
    
} m_robot_angle;

union
{
    short data ;
    char  bytedata[2];
    
} m_motor_speed, m_current_motor_speed;

union 
{ 	
	long data; 
	unsigned char b[2]; 
} current_encoder_val;

union 
{ 	
	unsigned short a; 
	unsigned char b[4]; 
} crc_16_val;


BYTE sonar_read_tx_data[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x1A, 0xC4, 0x01};

#define BAUDRATE        B19200
//#define SERIAL_DEVICE   "/dev/robot"  
#define SERIAL_DEVICE   "/dev/ttyUSB0"  

static int uart_fd;
unsigned char protocal_test[8] ={0,};
unsigned char read_buf[60];

int serial_command_type = 0; // 0 : monitor , 1 : command

int target_rpm 		 = 0;
long current_encoder = 0;

int CRC16_MODBUS (const uint8_t *nData, uint16_t wLength)
{

  static const uint16_t wCRCTable[] = { 0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780, 
	                                    0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 
	                                    0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 
	                                    0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 
	                                    0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 
	                                    0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 
	                                    0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 
	                                    0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 
	                                    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981, 
	                                    0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 
	                                    0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 
	                                    0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 
	                                    0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 
	                                    0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 
	                                    0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 
	                                    0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 
	                                    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 
	                                    0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 
	                                    0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 
	                                    0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40, 
	                                    0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 
	                                    0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 
	                                    0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 
	                                    0X81C1, 0X8081, 0X4040 };
	                                    
	uint8_t nTemp; 
    uint16_t wCRCWord = 0xFFFF; 

    while (wLength--)
    { 
      nTemp = *nData++ ^ wCRCWord; 
      wCRCWord >>= 8; 
      wCRCWord ^= wCRCTable[nTemp]; 
    } 

    return wCRCWord;                                 
}

void write_serial(unsigned char *buf, int len)
{
	write(uart_fd, &buf[0], len);
} 

int init_serial_port(void)
{
	int serial_port = open(SERIAL_DEVICE, O_RDWR);

	// Create new termios struct, we call it 'tty' for convention
	struct termios tty;

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) 
	{
	  printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	  return 1;
	}

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
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, BAUDRATE);
	cfsetospeed(&tty, BAUDRATE);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
	{
	  printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	  return -1;
	}

	else
	{
	  return serial_port;
	} 
}


void *readserial_thread(void *pt)
{
    
	int num_bytes = -1;
	unsigned char insert_buf; 

	static int data_length = 8;
	
    while(1)
    { 
		printf("thread starts\n");
		while( (num_bytes = read(uart_fd, &insert_buf, 1)   ) > 0 )	
        {
			if(serial_command_type == 0) //57 bytes recieve
			{
				data_length = 57;
			}
			else if(serial_command_type == 1) //8 bytes recieve
			{
				data_length = 8;

			}
			else
			{
				
			}
			
			for(int i = 0; i < data_length - 1; i++)
			{
				read_buf[i]=read_buf[i+1];
			}
			read_buf[data_length - 1] = insert_buf;
			
			if( (read_buf[0] == 0x01) && (read_buf[1] == 0x03) )
			{
				printf("Monitor_Data : ");

				for(int i = 0; i < data_length; i++)
				{
					printf("0x%02X ", read_buf[i]);
				}
				printf("\n");   
			}
			
			if( (read_buf[0] == 0x01) && (read_buf[1] == 0x06) )
			{
				printf("Speed_Data : ");

				for(int i = 0; i < data_length; i++)
				{
					printf("0x%02X ", read_buf[i]);
				}
				printf("\n");   
			}
			
			if(serial_command_type == 0)
			{
				crc_16_val.a = CRC16_MODBUS(read_buf, 55);
				
				if( (read_buf[55] == crc_16_val.b[0]) && (read_buf[56] == crc_16_val.b[1]) )
				{
					//current_speed
					m_current_motor_speed.bytedata[1] = read_buf[7];
					m_current_motor_speed.bytedata[0] = read_buf[8];
					
					printf("current_target_speed : %4d\n", m_current_motor_speed.data);
					
					/*
					current_pwm.bytedata[1] = read_buf[7];
					current_pwm.bytedata[0] = read_buf[8];
					
					printf("current_pwm : %4d\n", current_pwm.data);
					*/
					
					current_encoder_val.b[0] = read_buf[48];
					current_encoder_val.b[1] = read_buf[47];
					current_encoder_val.b[2] = read_buf[50];
					current_encoder_val.b[3] = read_buf[49];
					
					printf("current_target_speed : %6ld\n", current_encoder_val.data);
				}
 
			}

	       //printf("thread\n");
           //printf("No read %d\n",num_bytes);
           
           /*
           if( (read_buf[1]==0x03) && (read_buf[2]==0x02) )
		   {
				   
				  crc_16_val.a = CRC16_MODBUS(read_buf,5); 
				  if( ( crc_16_val.b[0]==read_buf[5]) && ( crc_16_val.b[1]==read_buf[6]) )
				  {
					  //sonar_range[read_buf[0]] = read_buf[3]*256 + read_buf[4];  //sonar id는 1번 부터
			
					  //printf("CRC16 is O.K. %d %d\n", read_buf[0], sonar_range[read_buf[0]]);
				  }   
		   }*/       
		   
         
	   }
	}	
	
} 

int ComSetup(void)
{
    int serial_port= open(SERIAL_DEVICE, O_RDWR); //set name of serial-com
    if(serial_port == -1)
    {
        printf("Can't open serial port!\n");
    }
    
     struct termios options;

  // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &options) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
    }

    tcflush(serial_port, TCIFLUSH);
    cfsetispeed(&options, B19200);   //set recieve bps of serial-com
    cfsetospeed(&options, B19200);   //set send bps of serial-com
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL);

    if(tcsetattr(serial_port, TCSANOW, &options) != 0)
    {
        printf("Can't set serial port options!\n");
    }
}

void modbus_enable(void)
{
	unsigned char protocal[8] = {0,};
	serial_command_type = 0;
	
	protocal[0] = 0x01;
	protocal[1] = 0x06;
	protocal[2] = 0x00;
	protocal[3] = 0x00;
	protocal[4] = 0x00;
	protocal[5] = 0x01;	
	
	crc_16_val.a = CRC16_MODBUS(protocal, 6); 
	
	protocal[6] = crc_16_val.b[0];
	protocal[7] = crc_16_val.b[1];
	
	printf("modbus_enable : ");
	for(int i = 0; i < 8; i++)
	{
		printf("0x%02X ", protocal[i]);
	}
	printf("\n");
	write_serial(protocal, 8);
}

void EN_enable(void)
{
	unsigned char protocal[8] = {0,};
	serial_command_type = 0;
	
	protocal[0] = 0x01;
	protocal[1] = 0x06;
	protocal[2] = 0x00;
	protocal[3] = 0x01;
	protocal[4] = 0x00;
	protocal[5] = 0x01;	
	
	crc_16_val.a = CRC16_MODBUS(protocal, 6); 
	
	protocal[6] = crc_16_val.b[0];
	protocal[7] = crc_16_val.b[1];
	
	printf("EN_enable : ");
	for(int i = 0; i < 8; i++)
	{
		printf("0x%02X ", protocal[i]);
	}
	printf("\n");
	write_serial(protocal, 8);
}

void monitor_command(void)
{
	unsigned char protocal[8] = {0,};
	serial_command_type = 0;
	
	protocal[0] = 0x01;
	protocal[1] = 0x03;
	protocal[2] = 0x00;
	protocal[3] = 0x00;
	protocal[4] = 0x00;
	protocal[5] = 0x1A;
	
	crc_16_val.a = CRC16_MODBUS(protocal, 6); 
	
	protocal[6] = crc_16_val.b[0];
	protocal[7] = crc_16_val.b[1];
	
	printf("Send : ");
	for(int i = 0; i < 8; i++)
	{
		printf("0x%02X ", protocal[i]);
	}
	printf("\n");
	write_serial(protocal, 8);
}

void send_motor_speed(int speed)
{
	unsigned char protocal[8] = {0,};
	serial_command_type = 1;

	protocal[0] = 0x01;
	protocal[1] = 0x06;
	protocal[2] = 0x00;
	protocal[3] = 0x02;
	
	m_motor_speed.data = speed;
	
	protocal[4] = m_motor_speed.bytedata[1];
	protocal[5] = m_motor_speed.bytedata[0];
	
	crc_16_val.a = CRC16_MODBUS(protocal, 6); 
	
	protocal[6] = crc_16_val.b[0];
	protocal[7] = crc_16_val.b[1];
	
	printf("Send : ");
	for(int i = 0; i < 8; i++)
	{
		printf("0x%02X ", protocal[i]);
	}
	printf("\n");
	write_serial(protocal, 8);
}

int main(int argc, char **argv)
{
	int i = 0;
	ros::init(argc, argv, "bld_400R4_motor_control_node");
	ros::NodeHandle n;

	std::string com_port = "/dev/ttyUSB0"; 

	uart_fd = init_serial_port(); 
	/*
    if(uart_fd < 0) 
    {
        printf("Failed to init serial port\n");
        return -1;
    }
    */
    
    modbus_enable();
    EN_enable();
    
	pthread_t id_1;
	int ret1=pthread_create(&id_1,NULL,*readserial_thread,NULL);

	ros::Rate loop_rate(1);

	while(ros::ok())
	{
		//monitor_command();
		send_motor_speed(0);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
