
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
    int frame_num;
    Pack packed;
};
Stream::Stream(void){
    video = cv::VideoWriter("appsrc  ! videoconvert ! video/x-raw !  x264enc  ! rtph264pay ! udpsink host=127.0.0.1 port=5000",cv::CAP_GSTREAMER,0, 10, cv::Size(2304,960), true); //Size(1440,160)
    //  video = cv::VideoWriter("out.mp4",VideoWriter::fourcc('H','V','C','1'),30, Size(2880,1920));

    rotation[0] = 1;
    rotation[1] = 0;
    rotation[2] = 0;
    rotation[3] = 0;
    frame_num = 0;
    packed.precompute();

}
void Stream::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  out = cv_bridge::toCvCopy(msg, "bgr8")->image;
  auto start = std::chrono::high_resolution_clock::now();

  packed.pack(out, frame_num);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  ROS_INFO("%d, %d", packed.packed.rows, packed.packed.cols);
  ROS_INFO("Pack time: %ld",duration.count());
  std::string dt = std::to_string(frame_num);
  frame_num++;
  packed.unpack();
  // cv::putText(packed.packed, 
  //             dt,
  //             cv::Size(10, 100),
  //             cv::FONT_HERSHEY_SCRIPT_COMPLEX, 1,
  //             (210, 155, 155),
  //             4, cv::LINE_8);
  
  cv::line(packed.unpacked, cv::Point(0,packed.unpacked.rows/2), cv::Point(packed.unpacked.cols,packed.unpacked.rows/2), cv::Scalar(0,255,0), 4, cv::LINE_8);

  namedWindow("face", cv::WINDOW_NORMAL);
  resizeWindow("face", Size(768,384));
  imshow("face", packed.unpacked);
  waitKey(1);
  video.write(packed.packed);

}
void Stream::rotationCallback(const sensor_msgs::Imu::ConstPtr& msg){
  rotation[0] = msg->orientation.w;
  rotation[1] = msg->orientation.x;
  rotation[2] = msg->orientation.y;
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
  image_transport::Subscriber itSub = it.subscribe("ben/sensors/cameras/qoocam/image_raw/", 1, &Stream::imageCallback, this, image_transport::TransportHints("compressed"));
  // image_transport::Subscriber itSub = it.subscribe("in_video/", 1, &Stream::imageCallback, this, image_transport::TransportHints("compressed"));
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

