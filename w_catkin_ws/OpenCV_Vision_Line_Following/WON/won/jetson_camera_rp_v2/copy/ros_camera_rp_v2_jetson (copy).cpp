//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

#define IMG_Width     1280
#define IMG_Height    720

#define USE_DEBUG  0   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용  0 CAMERA 미사용

#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}


Mat mat_image_org_color;  // Image 저장하기 위한 변수


int img_width = 640;
int img_height = 360;

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
   Rect r(points[0].x,points[0].y,image.cols, points[2].y-points[0].y);  
   //printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x, points[2].y-points[0].y);
   //printf("%d  %d\n", image.cols, points[2].y-points[0].y);

   img_roi_crop = image(r & bounds);
   
   return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat mat_img)
{
  
   Mat mat_blur_img, mat_canny_img;
  // blur(img, mat_blur_img, Size(3,3));	
  // Canny(mat_blur_img,mat_canny_img, 70,170,3);
	
   Canny(mat_img,mat_canny_img, 70,170,3);
   return mat_canny_img;	
}

int main(int argc, char **argv)
{
     
   int count;
   
   int capture_width = 1280 ;
   int capture_height = 720 ;
   int display_width = 640 ;
   int display_height = 360 ;
   int framerate = 60 ;
   int flip_method = 2 ;
    
  
   img_width = 640;
   img_height = 320;
   if(USE_CAMERA == 0) 
   {
	   img_width = 640;
	   img_height = 480;	 
   }
   
   std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  // return 1;
    
   std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
   cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
   
  
   if (!cap.isOpened()) 
   {
	cerr << "에러 - 카메라를 열 수 없습니다.\n";		
	return -1;	
   }
   
   cap.read(mat_image_org_color);
   img_width = mat_image_org_color.size().width ;
   img_height = mat_image_org_color.size().height;
	
   if(USE_CAMERA == 0) printf("Image size[%3d,%3d]\n", img_width,img_height);	
  
  
   ros::init(argc, argv, "ros_camera_image_topic_pub");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Publisher image_pub = it.advertise("/camera/image1", 1);     
   
   
   ros::Rate loop_rate(60);
 
   sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();

    
   while (ros::ok())
   { 
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     if(USE_CAMERA == 1)  cap.read(mat_image_org_color);
     else                 mat_image_org_color = imread("./img/line_1.jpg", IMREAD_COLOR);
     
     img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();
     image_pub.publish(img_msg);
     
     ros::spinOnce();

     loop_rate.sleep();
     ++count;
  }

  
   return 0;
}

 
