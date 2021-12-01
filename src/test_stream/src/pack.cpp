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
    unpacked = Mat(CUBESIZE * 3, CUBESIZE * 4, 16);

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
    static float precomp[7];
    float latitude = v * M_PI - M_PI / 2.0;
    float longitude = u * 2.0 * M_PI - M_PI;
    precomp[0] = u;
    precomp[1] = v;
    precomp[2] = latitude;
    precomp[3] = longitude;
    // Create a ray from the latitude and longitude
    precomp[4] = cos(latitude) * sin(longitude);
    precomp[5] = sin(latitude);
    precomp[6] = cos(latitude) * cos(longitude);
    return precomp;
}
void Pack::computeFaceMap(Mat &in, Mat &face, int faceID, float rotation[4]){
    int width = face.cols;
    int height = face.rows;
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

    for (int x = start; x < end; x++) {
        for (int y = 0; y < width; y++) {

            float u,v;
            u = uvPrecomp[faceID][x][y][0];
            v = uvPrecomp[faceID][x][y][1];
            float latitude = uvPrecomp[faceID][x][y][2];
            float longitude = uvPrecomp[faceID][x][y][3];
            float x_rot = uvPrecomp[faceID][x][y][4];
            float y_rot = uvPrecomp[faceID][x][y][5];
            float z_rot = uvPrecomp[faceID][x][y][6];
            float* rot_uv = new_rotation(latitude,longitude,x_rot,y_rot,z_rot, (float *)rotation, in.rows - 1,
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
float *Pack::new_rotation(float latitude, float longitude, float x, float y, float z, float *rotation, float inHeight,
                    float inWidth) {
    // Helpful resource for this function
    // https://github.com/DanielArnett/360-VJ/blob/d50b68d522190c726df44147c5301a7159bf6c86/ShaderMaker.cpp#L678
    // float latitude = v * M_PI - M_PI / 2.0;
    // float longitude = u * 2.0 * M_PI - M_PI;
    // // Create a ray from the latitude and longitude
    float p[4];
    p[0] = 0; 
    p[1] = x;
    p[2] = y;
    p[3] = z;

    // Rotate the ray based on the user input
    float rotationInv[4] = {rotation[0], -rotation[1], -rotation[2],
                            -rotation[3]};
    float *p_ret = quaternion_mult(
        quaternion_mult((float *)rotation, (float *)p), (float *)rotationInv);

    float x_rot = p_ret[1];
    float y_rot = p_ret[2];
    float z_rot = p_ret[3];
    // Convert back to latitude and longitude
    latitude = asin(y);
    longitude = atan2(x_rot, z_rot);

    // Convert back to the normalized M_PIxel coordinate
     x_rot = (longitude + M_PI) / (2.0 * M_PI);
     y_rot = (latitude + M_PI / 2.0) / M_PI;
    static float uv[2];
    // Convert to xy source M_PIxel coordinate
    uv[1] = y_rot * inHeight;
    uv[0] = x_rot * inWidth;

    return uv;
}

void Pack::packFace(Mat &in, float rotation[4], int faceID){
    int height = packedCoords[faceID][4];
    int width = packedCoords[faceID][5];
    Mat packedFace =  Mat(height, width, in.type());
    computeFaceMap(in, packedFace, faceID, rotation);
    packedFace.copyTo(packed(Rect(packedCoords[faceID][2],  packedCoords[faceID][3],packedFace.cols, packedFace.rows)));
}
void Pack::pack(Mat &in, float rotation[4], int frame_num){
    int height, width;
    //compute all faces
    int horizon[4] = {1,4,7,10};
    int horizonTopBott[8]={0,2,3,5,6,8,9,11};
    int topBott[2] = {12,13};
    if(frame_num%2==0){

    
    // if(frame_num%20 == 0){
        packFace(in, rotation, topBott[0]);
        packFace(in, rotation, topBott[1]);    // float rotation[4];
    // }
    // if(frame_num%4 == 0){
        for(int i = 0; i < 8; i++)
            packFace(in, rotation, horizonTopBott[i]); 
    // }
    for(int i = 0; i < 4; i++)
        packFace(in, rotation, horizon[i]); 
        }
}

void Pack::precompute(){
    uvPrecomp = (float****)malloc(sizeof(float****) * 14);
    for(int i = 0; i < 14; i++){
        int height = packedCoords[i][4];
        int width = packedCoords[i][5];
        int start = 0;
        int end = height;
        int faceType = packedCoords[i][1];

        if(faceType == 1){ //MID
            start = CUBESIZE/4;
            end = height + CUBESIZE/4;
        }
        if(faceType == 2){ //bott
            start = width*0.75;
            end = height + width*0.75;
        }
        uvPrecomp[i] = (float***)malloc(sizeof(float***) * end);
        for (int x = start; x < end; x++) {
            uvPrecomp[i][x] = (float**)malloc(sizeof(float**) * width);
            for (int y = 0; y < width; y++) {
                uvPrecomp[i][x][y] = (float*)malloc(sizeof(float*) * 7);
                float u,v;
                float*uv = computeUV(x,y, i);
                uvPrecomp[i][x][y][0] = uv[0];
                uvPrecomp[i][x][y][1] = uv[1];
                uvPrecomp[i][x][y][2] = uv[2];
                uvPrecomp[i][x][y][3] = uv[3];
                uvPrecomp[i][x][y][4] = uv[4];
                uvPrecomp[i][x][y][5] = uv[5];
                uvPrecomp[i][x][y][6] = uv[6];
            }
        }
    }
}


void Pack::unpack(){
    Mat face;
    for(int i = 0; i < 12; i++){
        face = packed(Range(packedCoords[i][3],packedCoords[i][3]+packedCoords[i][4]),Range(packedCoords[i][2],packedCoords[i][2]+packedCoords[i][5]));
        resize(face, face, Size(),2.5, 2.5, INTER_CUBIC); //upscale 4
        face.copyTo(unpacked(Rect(cubeCoords[i][0],  cubeCoords[i][1],face.cols, face.rows)));
        i++;

        face = packed(Range(packedCoords[i][3],packedCoords[i][3]+packedCoords[i][4]),Range(packedCoords[i][2],packedCoords[i][2]+packedCoords[i][5]));
        face.copyTo(unpacked(Rect(cubeCoords[i][0],  cubeCoords[i][1], face.cols, face.rows)));
        i++;

        face = packed(Range(packedCoords[i][3],packedCoords[i][3]+packedCoords[i][4]),Range(packedCoords[i][2],packedCoords[i][2]+packedCoords[i][5]));
        resize(face, face, Size(),2.5, 2.5, INTER_CUBIC); //upscale 4
        face.copyTo(unpacked(Rect(cubeCoords[i][0],  cubeCoords[i][1],face.cols, face.rows)));
    }

   
    int i = 12;
    Mat top = packed(Range(packedCoords[i][3],packedCoords[i][3]+packedCoords[i][4]),Range(packedCoords[i][2],packedCoords[i][2]+packedCoords[i][5]));
    resize(top, top, Size(),5, 5, INTER_CUBIC); //upscale 4
    rotate(top, top, ROTATE_90_CLOCKWISE);
    top.copyTo(unpacked(Rect(cubeCoords[i][0],  cubeCoords[i][1],top.cols, top.rows)));
    
    i = 13;
    Mat bottom = packed(Range(packedCoords[i][3],packedCoords[i][3]+packedCoords[i][4]),Range(packedCoords[i][2],packedCoords[i][2]+packedCoords[i][5]));
    resize(bottom, bottom, Size(),5, 5, INTER_CUBIC); //upscale 4
    rotate(bottom, bottom, ROTATE_90_COUNTERCLOCKWISE);
    bottom.copyTo(unpacked(Rect(cubeCoords[i][0],  cubeCoords[i][1],bottom.cols, bottom.rows)));
}

