
#include <stdio.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "pack.h"

class Stream{
  public:
    cv::Mat out;
    cv::VideoWriter video;
    Stream();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void run(int argc, char **argv);
    float roll;
};
Stream::Stream(void){
    video = cv::VideoWriter("appsrc! video/x-raw,framerate=60/1 ! videoscale ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000",cv::CAP_GSTREAMER,0, 60, cv::Size(1920,960), true); //Size(1440,160)
    roll = 0;
}
void Stream::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("callback");
  cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, "bgr8");
  cv::Mat out = cvImg->image;

  float *rotation = CreateFromYawPitchRoll(roll, roll, 0);
  cv::Mat packed = pack(out,rotation);
  cv::imshow("packed", packed);
  cv::waitKey(1);
  roll = (roll + 0.0174533);
  if (roll > 6.28319) roll = 0;
  // video.write(out);

}
void Stream::run (int argc, char **argv){
  //initialize node 
  ros::init(argc,argv,"test_stream_node");
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

  ros::spin();
}

int main(int argc, char **argv)
{
  Stream stream;

  stream.run(argc, argv);
  return 0;
}

