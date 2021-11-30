
#include <stdio.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"
#include <chrono>
#include "pack.h"

class Stream{
  public:
    cv::Mat out;
    cv::VideoWriter video;
    Stream();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rotationCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void run();
    float rotation[4];
    int timestamp;
    Pack packed;
};
Stream::Stream(void){
    video = cv::VideoWriter("appsrc  ! videoconvert ! x264enc bitrate=10000 speed-preset=ultrafast ! rtph264pay ! udpsink host=127.0.0.1 port=5000",cv::CAP_GSTREAMER,0, 30, cv::Size(1920,960), true); //Size(1440,160)
    rotation[0] = 1;
    rotation[1] = 0;
    rotation[2] = 0;
    rotation[3] = 0;
    timestamp = 0;
}
void Stream::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  out = cv_bridge::toCvCopy(msg, "bgr8")->image;
  auto start = std::chrono::high_resolution_clock::now();

  packed.pack(out, rotation);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  ROS_INFO("Pack time: %ld",duration.count());
  std::string dt = std::to_string(timestamp);
  timestamp++;
  cv::putText(packed.packed, 
              dt,
              cv::Size(10, 100),
              cv::FONT_HERSHEY_SCRIPT_COMPLEX, 1,
              (210, 155, 155),
              4, cv::LINE_8);

   namedWindow("face", cv::WINDOW_NORMAL);
    resizeWindow("face", Size(768,384));
    imshow("face", packed.packed);
    waitKey(1);
  // video.write(out);

}
void Stream::rotationCallback(const sensor_msgs::Imu::ConstPtr& msg){
  rotation[0] = msg->orientation.w;
  rotation[1] = -msg->orientation.x;
  rotation[2] = -msg->orientation.y;
  rotation[3] = msg->orientation.z; 
}
void Stream::run(){
  //initialize node 
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  if(!video.isOpened()) {
      ROS_INFO("Writer not opened\n");
      exit(0);
  }
  else{
    ROS_INFO("SUCCESS");
  }
  // image_transport::Subscriber itSub = it.subscribe("ben/sensors/cameras/qoocam/image_raw/", 1, &Stream::imageCallback, this, image_transport::TransportHints("compressed"));
  image_transport::Subscriber itSub = it.subscribe("in_video/", 1, &Stream::imageCallback, this, image_transport::TransportHints("compressed"));
  // ros::Subscriber qSub= n.subscribe("/ben/sensors/posmv/orientation", 1, &Stream::rotationCallback, this);
// 
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"test_stream_node");

  Stream stream;

  stream.run();
  // ROS_INFO("Hello world");
  return 0;
}

