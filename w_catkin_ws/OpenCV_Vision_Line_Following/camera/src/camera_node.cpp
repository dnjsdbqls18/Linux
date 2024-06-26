#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;

std::string gstreamer_pipeline(int capture_width, int capture_height, int framerate) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Camera_Pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image", 1);

    // 카메라 설정
    int capture_width = 640;
    int capture_height = 360;
    int framerate = 60;
    std::string pipeline = gstreamer_pipeline(capture_width, capture_height, framerate);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
   
    if (!cap.isOpened()) {
        ROS_ERROR("Error - Could not open camera.");
        ros::shutdown();
        return -1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(60);  // 초당 30프레임으로 설정

    while (ros::ok()) {
        cap >> frame;  // 카메라로부터 프레임 가져오기

        if (frame.empty()) {
            ROS_ERROR("Error - Frame is empty.");
            break;
        }

        // 프레임 180도 회전
        cv::rotate(frame, frame, cv::ROTATE_180);

        // OpenCV Mat을 ROS 이미지 메시지로 변환
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // ROS 이미지 퍼블리싱
        image_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();  // 카메라 해제

    return 0;
}
