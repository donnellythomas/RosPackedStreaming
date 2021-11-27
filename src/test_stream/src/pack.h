#include <opencv2/opencv.hpp>
using namespace cv;
Mat pack(Mat &in, float rotation[4]);
float* CreateFromYawPitchRoll(float yaw, float pitch, float roll);
static float *quaternion_inverse(const float q[4]);
