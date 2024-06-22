#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	int capture_width = 1280;
	int capture_height = 720;
	int framerate = 60;
	int flip_method = 2;

	std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
                       std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
                       "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

	if (!cap.isOpened()) 
	{
		cerr << "에러 - 카메라를 열 수 없습니다.\n";
		return -1;	
	}

	ros::init(argc, argv, "ros_camera_image_topic_pub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("/camera/image1", 1);

	ros::Rate loop_rate(60);

	Mat mat_image_org_color;

	while (ros::ok())
	{
		cap.read(mat_image_org_color);

		sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();
		image_pub.publish(img_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
