#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <math.h>

using namespace cv; 
using namespace std; 

#define FORWARD_SPEED 0.5 
#define BACKWARD_SPEED 0.3 
#define OBSTACLE_RANGE0 0.1 // for PARK 
#define OBSTACLE_RANGE1 0.3 // in FORWARD 
#define OBSTACLE_RANGE2 0.5 // in BACKWARD 

#define ROI_CENTER_Y 120
#define ROI_WIDTH 20

#define NO_LINE 20
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

Mat cv_image;

int img_width = 640; 
int img_height = 360; 

Mat mat_image_org_color;
Mat mat_image_org_gray;
Mat mat_image_roi;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge_roi;

std_msgs::Float32 steer_angle;
float steer_angle_new = 0;
float steer_angle_old = 0;
int m_roi_center, m_roi_height, m_roi_width,m_roi_width_large;

float line_center_x = 0.0; 
float line_center_x_old = 0.0;

float steering_conversion = -0.0030; // -0.0025 for 0.3 

float linear_x=0.0;
float angular_z=0.0;

float range = 0.;
//float m_Kp,m_Kd,m_Ki;
float R_Kp,R_Kd,R_Ki;
float L_Kp,L_Kd,L_Ki;
ros::Publisher one_lane_cross_track_error_pub_cmd;
std_msgs::Float32 cmd_steering_msg;  
double  line_center_x_offset =-4.5; //0.0

struct Rect_Region
{
   int left;
   int right;
   int top;
   int bottom;      
};

enum DriveMode
{
    PARK = 0,
    RANGE = 1,
    CAMERA = 2
} drive_mode = PARK;

struct Rect_Region ROI_line_center;

bool one_lane_control_run_set = true;

/*
void run_one_lane_control_flagCallback(const std_msgs::Bool& msg)
{
    one_lane_control_run_set = msg.data;

}
*/

Mat Region_of_Interest(Mat image, Point *points)
{
  Mat img_mask =Mat::zeros(image.rows,image.cols,CV_8UC1);    
  
  Scalar mask_color = Scalar(255,255,255);
  const Point* pt[1]={ points };       
  int npt[] = { 4 };
  cv::fillPoly(img_mask,pt,npt,1,Scalar(255,255,255),LINE_8);
  Mat masked_img;   
  bitwise_and(image,img_mask,masked_img);
  
  return masked_img;
}

Mat Region_of_Interest_crop(Mat image, Point *points)
{
   Mat img_roi_crop;   

   Rect bounds(0,0,image.cols,image.rows);    
   Rect r(points[0].x, points[0].y, points[2].x-points[0].x, points[2].y-points[0].y);    
 
   img_roi_crop = image(r & bounds);
   
   return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));   
   Canny(mat_blur_img,mat_canny_img, 70,150,3);
   
   return mat_canny_img;
}

void Update_line_detect_ROI(int center_x , int no_line)
{
   int r_pos,l_pos;
   
   ROI_line_center.top = m_roi_center - m_roi_height;
   ROI_line_center.bottom = m_roi_center + m_roi_height;
   // printf("line_count : %d\n",no_line); 
    if(no_line !=0)
    {
      
      r_pos = center_x + m_roi_width + img_width/2;
      l_pos = center_x - m_roi_width + img_width/2;
    }
    else
    {
     r_pos = center_x + m_roi_width_large + img_width/2;
      l_pos = center_x - m_roi_width_large + img_width/2;
   }
  
    
    r_pos = ( r_pos >= img_width ) ? img_width-1 : r_pos;     
    l_pos = ( l_pos <= 0 ) ? 0 : l_pos; 
   //printf("m_point: %d %d \n",l_pos,r_pos);
   ROI_line_center.right = r_pos;
   ROI_line_center.left = l_pos;
   
}

void read_left_pid_gain(void)
{
   
   FILE *fp;
   int result = 0;
      
   fp = fopen("/home/amap/w_catkin_ws/src/line_control/left_pid.txt","r");
   if(fp == NULL) 
   {
      ROS_INFO("L_PID gain File does not exit ~~ \n\n");
      exit(0);
   }
   result = fscanf(fp,"%f", &L_Kp);
   result = fscanf(fp,"%f", &L_Kd);
   result = fscanf(fp,"%f", &L_Ki);
   
   printf("PID: %.4f %.4f %.4f \n",L_Kp,L_Kd,L_Ki);
   
   fclose(fp);
}

void read_right_pid_gain(void)
{
   
   FILE *fp;
   int result = 0;
      
   fp = fopen("/home/amap/w_catkin_ws/src/line_control/right_pid.txt","r");
   if(fp == NULL) 
   {
      ROS_INFO("R_PID gain File does not exit ~~ \n\n");
      exit(0);
   }
   result = fscanf(fp,"%f", &R_Kp);
   result = fscanf(fp,"%f", &R_Kd);
   result = fscanf(fp,"%f", &R_Ki);
   
   printf("PID: %.4f %.4f %.4f \n",R_Kp,R_Kd,R_Ki);
   
   fclose(fp);
}
 
void read_roi_data(void)
{
   
   FILE *fp;
   int result = 0;
   
   
   fp = fopen("/home/amap/w_catkin_ws/src/line_control/roi.txt","r");
   
   if(fp == NULL) 
   {
      ROS_INFO("L_ROI File does not exit ~~ \n\n");
      exit(0);
   }
   
   result = fscanf(fp,"%d", &m_roi_center);
   result = fscanf(fp,"%d", &m_roi_height);
   result = fscanf(fp,"%d", &m_roi_width);
   result = fscanf(fp,"%d", &m_roi_width_large);

   
    ROI_line_center.top = m_roi_center - m_roi_height;
    ROI_line_center.bottom = m_roi_center + m_roi_height;
    ROI_line_center.left = img_width/2 - m_roi_width ;
    ROI_line_center.right = img_width/2 + m_roi_width;
    
     
    fclose(fp);
    
    printf("ROI : %3d %3d %3d %3d \n",ROI_line_center.left, ROI_line_center.top, ROI_line_center.right, ROI_line_center.bottom);
    
    
}

void line_detection(void)
{
   Point points[4]; // ROI(Region of Interest) 

    int count = 0;
    int line_count = 0;
    float c[NO_LINE] = {0.0, };
    float d[NO_LINE] = {0.0, };
    

    int inter_sect_x[NO_LINE] = {0, };
    int inter_sect_group[NO_LINE] = {0, };  
    
    /*
    points[0] = Point(0,ROI_CENTER_Y-ROI_WIDTH);
    points[1] = Point(0,ROI_CENTER_Y+ROI_WIDTH);
    points[2] = Point(img_width,ROI_CENTER_Y+ROI_WIDTH);
    points[3] = Point(img_width,ROI_CENTER_Y-ROI_WIDTH);
    
    */
    points[0] = Point(ROI_line_center.left , ROI_line_center.top);
    points[1] = Point(ROI_line_center.left , ROI_line_center.bottom);
    points[2] = Point(ROI_line_center.right , ROI_line_center.bottom);
    points[3] = Point(ROI_line_center.right , ROI_line_center.top);
    
   // color to gray conversion  
    cvtColor(mat_image_org_color, mat_image_org_gray, cv::COLOR_RGB2GRAY);

    // ROI 영역 추출      
    mat_image_roi = Region_of_Interest_crop(mat_image_org_gray,points);

    // Canny Edge 추출
    mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);

  

    vector<Vec4i> linesP;
    HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/180, 15, 10, 10); // ... threshold, minLength, maxGap
     
    line_count = line_center_x = 0.0;    
    //printf("Line number : %d \n",linesP.size()); 
   for(int i=0; i<linesP.size();i++)
   {
      float intersect = 0.0;
      if(i>=NO_LINE) break;
      Vec4i L= linesP[i];
        if(fabs(L[3]-L[1])>1.0e-7)
         c[i] = (float)(L[2]-L[0])/(float)(L[3]-L[1]);
        else 
           c[i] = 1.0e7;
      d[i] = L[0]-c[i] *L[1] ;
      if(fabs(c[i])< tan(DEG2RAD(65)))
      {
          intersect = c[i]*(float)m_roi_height + d[i];
          line_center_x += intersect;
          inter_sect_x[line_count]=intersect;
          line_count++;
          line(mat_image_org_color,Point(L[0]+ROI_line_center.left,L[1]+ROI_line_center.top),Point(L[2]+ROI_line_center.left,L[3]+ROI_line_center.top), Scalar(0,255,0), 1, LINE_AA);         
          
          circle(mat_image_org_color, Point((int)inter_sect_x[i]+ROI_line_center.left,m_roi_center), 4, Scalar(255,255,0), 1, LINE_AA);
       }
   } 

    if(line_count >0)
    {
      
		rectangle(mat_image_org_color, points[0], points[2], Scalar(255, 0, 0), 1);
        line_center_x = line_center_x / (float)linesP.size() - img_width/2 + ROI_line_center.left + line_center_x_offset;
        

        if(drive_mode == CAMERA)
        {
            linear_x = FORWARD_SPEED;
            angular_z = steer_angle_new;
        }
        
       //line(mat_image_org_color, Point(0,ROI_CENTER_Y),Point(img_width,ROI_CENTER_Y), Scalar(0,0,255), 2, LINE_AA);   
       //line(mat_image_org_color, Point((int)line_center_x+img_width/2,ROI_CENTER_Y-ROI_WIDTH),Point((int)line_center_x+img_width/2,ROI_CENTER_Y+ROI_WIDTH), Scalar(255,255,0), 2, LINE_AA);
        if(((int)line_center_x+img_width/2) > 0 && ((int)line_center_x+img_width/2) < img_width)
        {
            circle(mat_image_org_color, Point((int)line_center_x+img_width/2,m_roi_center), 5, Scalar(0,255,0), 2, LINE_AA);
		}
        
        if(line_center_x >= 0)
        {
			steer_angle_new = -(line_center_x * R_Kp + (line_center_x-line_center_x_old) * R_Kd) ; //스티어링 조정값 PI 계수 수정 필요   
		}
		else
		{
			steer_angle_new = -(line_center_x * L_Kp + (line_center_x-line_center_x_old) * L_Kd) ; //스티어링 조정값 PI 계수 수정 필요   
		}
        line_center_x_old = line_center_x;  
        steer_angle_old = steer_angle_new ;
    }
    else
    {
      
      rectangle(mat_image_org_color, points[0], points[2], Scalar(0, 0, 255), 1);
        if(drive_mode == CAMERA) {
          // linear_x = -BACKWARD_SPEED;
          // angular_z = 0.0;
        }
        //circle(mat_image_org_color, Point(img_width/2,m_roi_center), 5, Scalar(0,0,255), 1, LINE_AA);
        circle(mat_image_org_color, Point((int)line_center_x_old+img_width/2,m_roi_center), 5, Scalar(0,0,255), 2, LINE_AA);
            
    }
   
   Update_line_detect_ROI( (int)(line_center_x_old + 0.5) , line_count );
   //steer_angle_old = steer_angle_new ;
   
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (one_lane_control_run_set == true) 
    {
        cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        img_width = cv_image.size().width;
        img_height = cv_image.size().height;

        mat_image_org_color = cv_image.clone();

        line_detection();
        cv::imshow("view", mat_image_org_color);
        cv::waitKey(1);

        //printf("image_topic_flag : %d \n", image_topic_flag);
        double steering_cmd = steer_angle_new;
        if (steering_cmd >= 45.0) {
            steering_cmd = 45.0;
        }
        else if (steering_cmd <= -45.0) {
            steering_cmd = -45.0;
        }
        cmd_steering_msg.data = steering_cmd;
        printf("Steer : %.3f\n", cmd_steering_msg.data);

        one_lane_cross_track_error_pub_cmd.publish(cmd_steering_msg);
    }
}
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("Range=%d", (int)msg->range);
    range = msg->range;

    drive_mode = CAMERA;

    // Parking
    if(range < OBSTACLE_RANGE0) {
        drive_mode = PARK;
        linear_x = 0.;
        angular_z = 0.;
    }
    // Driving Forward
    else if(range <= OBSTACLE_RANGE1 && linear_x > 0.) {
        drive_mode = RANGE;
        linear_x = -BACKWARD_SPEED;
        //angular_z = 0.;
    }
    // Driving Backward
    else if(range >= OBSTACLE_RANGE2 && linear_x < 0.) {
        line_detection();
        drive_mode = CAMERA;
        linear_x = FORWARD_SPEED;
        //angular_z = 0.;
    } 
}

  
int main(int argc, char **argv)
{
    Point points[4]; // ROI(Region of Interest) 

    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;
    float steering_change_limit = 0.052;
    float steering_cmd_current =0.0;
    //float steering_cmd =0.0;
	std::string one_lane_xte_topic       = "/xte/one_lane";
	std::string steering_angle_topic     = "/steering_angle";

    ros::param::get("~steering_angle_topic",       steering_angle_topic);
    ros::param::get("~line_center_x_offset",line_center_x_offset);

    ros::Subscriber sub_rng = nh.subscribe("/range", 1000, rangeCallback);
	ros::Publisher steering_angle_pub = nh.advertise<std_msgs::Float32>(steering_angle_topic, 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
    
    one_lane_cross_track_error_pub_cmd = nh.advertise<std_msgs::Float32>(one_lane_xte_topic, 1);
      
    cmd_steering_msg.data = 0;
   
    geometry_msgs::Twist msg_cmd;
    
    //ros::Duration(0.5).sleep(); // sleep for 0.5 senods

    //ros::Subscriber run_lane_control_flag_sub = nh.subscribe("/flag/one_lane_control_set", 1, &run_one_lane_control_flagCallback);
    
    read_roi_data();
    read_right_pid_gain();
    read_left_pid_gain();
    
    ros::Rate rate(60);
    while (ros::ok())
    {       
       //ROS_INFO("linear_x=%f, angular_z=%f %f", linear_x, angular_z, line_center_x);
       
       //printf("%.3f %.3f %.3f\n", line_center_x, line_center_x_old,steer_angle_new);
       
       //msg_cmd.linear.x = linear_x;
       //msg_cmd.angular.z = angular_z;
       //pub_cmd_vel.publish(msg_cmd);
       
       //steering_cmd = steer_angle_new;
       /*
       if(steering_cmd >= steering_cmd_current)
       {
         steering_cmd_current += steering_change_limit; // v= v0+ a * t
      }
      else if(steering_cmd <= steering_cmd_current)
       {
         steering_cmd_current -= steering_change_limit;
      }
       cmd_steering_msg.data = steering_cmd_current;
     
      // cmd_steering_msg.data = steer_angle_new;*/
      
       steering_angle_pub.publish(steer_angle);
       
       steer_angle.data = cmd_steering_msg.data;
    
       ros::spinOnce();
       rate.sleep();       
    }
    cv::destroyWindow("view");
}
