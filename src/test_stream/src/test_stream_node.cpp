
#include <stdio.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"
#include <chrono>
// #include "Quaternion.h"
#include "pack.h"

class Stream{
  public:
    cv::Mat out;
    cv::VideoWriter video;
    Stream();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rotationCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void run();
    Quaternion rotation;
    int frame_num;
    Pack packed;
    Pack no_rotation;
};
Stream::Stream(void){
    // video = cv::VideoWriter("appsrc  ! videoconvert ! video/x-raw !  x264enc  ! rtph264pay ! udpsink host=127.0.0.1 port=5000",cv::CAP_GSTREAMER,0, 10, cv::Size(2304,960), true); //Size(1440,160)
    video = cv::VideoWriter("out.mp4",VideoWriter::fourcc('H','V','C','1'),30, Size(2880,1920));
    frame_num = 0;
    packed.precompute();
    no_rotation.precompute();
    namedWindow("face", cv::WINDOW_NORMAL);
    resizeWindow("face", Size(768,384));
    namedWindow("face2", cv::WINDOW_NORMAL);
    resizeWindow("face2", Size(768,384));

}
void Stream::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  out = cv_bridge::toCvCopy(msg, "bgr8")->image;
  auto start = std::chrono::high_resolution_clock::now();
  // packed.with_rotation = 1;
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  // ROS_INFO("%d, %d", packed.packed.rows, packed.packed.cols);
  ROS_INFO("Pack time: %ld",duration.count());
  frame_num++;
  // rotation.CreateFromYawPitchRoll(0,0,0) ;

  packed.pack(out, rotation, frame_num);
  ROS_INFO("%f, %f, %f, %f\n", rotation.m_w,rotation.m_x,rotation.m_y,rotation.m_z);
  packed.unpack();
  packed.theta+=10;
  if (packed.theta>360){
    packed.theta = 0;
  }
  Quaternion norot;
  no_rotation.pack(out, norot, frame_num);
  no_rotation.unpack();
  // //debugging frame latency
  // std::string dt = std::to_string(frame_num);
  // cv::putText(packed.packed, 
  //             dt,
  //             cv::Size(10, 100),
  //             cv::FONT_HERSHEY_SCRIPT_COMPLEX, 1,
  //             (210, 155, 155),
  //             4, cv::LINE_8);

  //horizon line for debugging
  cv::line(packed.unpacked, cv::Point(0,packed.unpacked.rows/2), cv::Point(packed.unpacked.cols,packed.unpacked.rows/2), cv::Scalar(0,0,255), 4, cv::LINE_8);
  cv::line(no_rotation.unpacked, cv::Point(0,no_rotation.unpacked.rows/2), cv::Point(no_rotation.unpacked.cols,no_rotation.unpacked.rows/2), cv::Scalar(0,0,255), 4, cv::LINE_8);

  //show frame
  
  imshow("face", packed.unpacked);

  imshow("face2", no_rotation.unpacked);
  waitKey(1);

  //write to output
  video.write(packed.packed);

}
void Stream::rotationCallback(const sensor_msgs::Imu::ConstPtr& msg){
  rotation.m_w = msg->orientation.w;
  rotation.m_x = -msg->orientation.y;
  rotation.m_y = msg->orientation.z;
  rotation.m_z = msg->orientation.x; 
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
  ros::Subscriber qSub= n.subscribe("/ben/sensors/posmv/orientation", 1, &Stream::rotationCallback, this);
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

