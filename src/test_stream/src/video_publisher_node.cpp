#include <stdio.h>
#include "ros/ros.h"
// #include "cvUtils.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
using namespace ros;
cv::VideoCapture importVideo(std::string path){
    cv::VideoCapture cap(path, cv::CAP_FFMPEG);
    if(!cap.isOpened()){
        printf("Error opening video\n");
        // return 0;
    }
    return cap;
}

void publishVideo(image_transport::Publisher pub, std::string path){
    
   while(1){ cv::VideoCapture cap = importVideo(path);
        ROS_INFO("Video Imported");
        if(!cap.isOpened()){
            printf("Error opening video\n");
            ROS_INFO("Error opening video\n");
            exit(0);
        }
        
        while(1){

            cv::Mat frame;
            ROS_INFO("publishing video frame");
        
            cap>>frame;
            // check if we succeeded
            if (frame.empty()) {
                ROS_INFO("ERROR! blank frame grabbed\n");
                break;
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            // cv::imshow("view",frame);
            // cv::waitKey(1);
            // printf("%d, %d, %d\n", frame.rows, frame.cols, frame.type());
            // if (cv::waitKey(5) >= 0)
            //     break;
        }
        cap.release();
    }
    
}

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc,argv,"video_publisher_node");
    ROS_INFO("Initialized");
    ros::NodeHandle n;
    image_transport::ImageTransport video(n);
    image_transport::Publisher videoPub = video.advertise("/in_video", 1);
    std::string path = argv[1];
    
    publishVideo(videoPub, path);
    ros::spin();
    return 0;
}

