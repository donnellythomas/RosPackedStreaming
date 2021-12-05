#include <stdio.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"
#include <chrono>
#include "pack.h"
//import bag video
//import bag orientation information
//pack video without rotation
//pack video with rotation

class Rotation{
    public:
        void run();
        Rotation();
        float rotation[4];
        Pack packed;
};
Rotation::Rotation(){
    rotation[0] = 1;
    rotation[1] = 0;
    rotation[2] = 0;
    rotation[3] = 0;
    packed.precompute();

}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"rotation_node");

  Rotation rotation;

  rotation.run();

  return 0;

}