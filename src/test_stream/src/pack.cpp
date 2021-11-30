#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "pack.h"

#include <thread>
using namespace std::chrono;
using namespace std;
using namespace cv;
Pack::Pack(void){          
    packed = Mat(CUBESIZE, CUBESIZE*2+CUBESIZE*0.4f, 16);
}
 float *Pack::quaternion_mult(const float q[4], const float r[4]) {
    static float ret[4];
    // printf("before1 w: %f, X: %f, Y: %f, Z: %f\n", q[0], q[1], q[2], q[3]);
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];
    float r0 = r[0];
    float r1 = r[1];
    float r2 = r[2];
    float r3 = r[3];
    ret[0] = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3;
    ret[1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
    ret[2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
    ret[3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;
    return ret;
}
 float *Pack::quaternion_inverse(const float q[4]){
     static float ret[4];
    // printf("before1 w: %f, X: %f, Y: %f, Z: %f\n", q[0], q[1], q[2], q[3]);
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];
   
    ret[0] = q0;
    ret[1] = -q1;
    ret[2] = -q2;
    ret[3] = -q3;
    return ret;
}
float* Pack::CreateFromYawPitchRoll(float yaw, float pitch,
                              float roll)  // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    float m_w = cy * cp * cr + sy * sp * sr;
    // printf("%f\n", m_w);
    float m_x = cy * cp * sr - sy * sp * cr;
    // printf("%f\n", m_x);

    float m_y = sy * cp * sr + cy * sp * cr;
    // printf("%f\n", m_y);

    float m_z = sy * cp * cr - cy * sp * sr;
    // printf("%f\n", m_z);

    static float rotation[4];
    rotation[0] = m_w;
    rotation[1] = m_x;
    rotation[2] = m_y;
    rotation[3] = m_z;
    return rotation;
}
float* Pack::computeUV(int x, int y, int faceID ){
    // Map face pixel coordinates to [-1, 1] on plane
    int faceTrans = packedCoords[faceID][0];
    float ftu = faceTransform[faceTrans][0];
    float ftv = faceTransform[faceTrans][1];
    int faceWidth = packedCoords[faceID][5];
    float nx = ((float)(y) / faceWidth - 0.5f);
    float ny = ((float)(x) / faceWidth - 0.5f);
    nx *= 2;
    ny *= 2;
    nx *= AN;
    ny *= AN;
    float u, v;

    if (ftv == 0) {
        // Center faces
        u = atan2(nx, AK);
        v = atan2(ny * cos(u), AK);
        u += ftu;
    } else if (ftv > 0) {
        // Bottom face
        float d = sqrt(nx * nx + ny * ny);
        v = M_PI / 2 - atan2(d, AK);
        u = atan2(ny, nx);
    } else {
        // Top face
        float d = sqrt(nx * nx + ny * ny);
        v = -M_PI / 2 + atan2(d, AK);
        u = atan2(-ny, nx);
    }

    u = u / (M_PI);
    v = v / (M_PI / 2);

    // Warp around, if our coordinates are out of bounds.
    while (v < -1) {
        v += 2;
        u += 1;
    }
    while (v > 1) {
        v -= 2;
        u += 1;
    }

    while (u < -1) {
        u += 2;
    }
    while (u > 1) {
        u -= 2;
    }

    u = u / 2.0f + 0.5f;
    v = v / 2.0f + 0.5f;
    static float uv[2];
    uv[0] = u;
    uv[1] = v;
    return uv;
}
void Pack::computeFaceMap(Mat &in, Mat &face, int faceID, float rotation[4]){
    float width = face.cols;
    float height = face.rows;
    Mat mx(height,width,CV_32F);
    Mat my(height,width,CV_32F);
    //determine start and end points based on face type 
    int start = 0;
    int end = height;
    int faceType = packedCoords[faceID][1];
    if(faceType == 1){ //MID
        start = CUBESIZE/4;
        end = height + CUBESIZE/4;
    }
    if(faceType == 2){ //bott
        start = width*0.75;
        end = height + width*0.75;
    }
    for (int y = 0; y < width; y++) {
        for (int x = start; x < end; x++) {
            float u,v;
            
            float *uv = computeUV(x,y, faceID);
            u = uv[0];
            v = uv[1];
            float* rot_uv = new_rotation(u, v, (float *)rotation, in.rows - 1,
                              in.cols - 1);
            u = rot_uv[0];
            v = rot_uv[1];

            mx.at<float>(x-start, y) = u;
            my.at<float>(x-start, y) = v;
        }
    }
   
    remap(in, face, mx, my, INTER_LINEAR, BORDER_CONSTANT,
          Scalar(0, 0, 0));

} 
float *Pack::new_rotation(float u, float v, float *rotation, float inHeight,
                    float inWidth) {
    // Helpful resource for this function
    // https://github.com/DanielArnett/360-VJ/blob/d50b68d522190c726df44147c5301a7159bf6c86/ShaderMaker.cpp#L678
    float latitude = v * M_PI - M_PI / 2.0;
    float longitude = u * 2.0 * M_PI - M_PI;
    // Create a ray from the latitude and longitude
    float p[4];
    p[0] = 0;
    p[1] = cos(latitude) * sin(longitude);
    p[2] = sin(latitude);
    p[3] = cos(latitude) * cos(longitude);

    // Rotate the ray based on the user input
    float rotationInv[4] = {rotation[0], -rotation[1], -rotation[2],
                            -rotation[3]};
    float *p_ret = quaternion_mult(
        quaternion_mult((float *)rotation, (float *)p), (float *)rotationInv);

    float x = p_ret[1];
    float y = p_ret[2];
    float z = p_ret[3];
    // Convert back to latitude and longitude
    latitude = asin(y);
    longitude = atan2(x, z);
    // Convert back to the normalized M_PIxel coordinate
    x = (longitude + M_PI) / (2.0 * M_PI);
    y = (latitude + M_PI / 2.0) / M_PI;
    static float uv[2];
    // Convert to xy source M_PIxel coordinate
    uv[1] = y * inHeight;
    uv[0] = x * inWidth;

    return uv;
}


void Pack::pack(Mat &in, float rotation[4]){
    int height, width;
    //compute all faces
    for(int i = 0; i < 14; i++){
        height = packedCoords[i][4];
        width = packedCoords[i][5];
        Mat packedFace =  Mat(height, width, in.type());
        computeFaceMap(in, packedFace, i, rotation);
        packedFace.copyTo(packed(Rect(packedCoords[i][2],  packedCoords[i][3],packedFace.cols, packedFace.rows)));
    }
}

