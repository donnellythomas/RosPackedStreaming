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
    theta = 0;
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
void Pack::computeFaceMap(Mat &in, Mat &face, int faceID, Quaternion rotation){
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
            
            float* rot_uv = new_rotation(latitude,longitude,x_rot,y_rot,z_rot, rotation);
            u = rot_uv[0] ;
            v = rot_uv[1] ;
            
            u = u * in.cols-1;
            v = v * in.rows-1;

            mx.at<float>(x-start, y) = u;
            my.at<float>(x-start, y) = v;
        }
    }
   
    remap(in, face, mx, my, INTER_LINEAR, BORDER_CONSTANT,
          Scalar(0, 0, 0));

} 

float *Pack::new_rotation(float latitude, float longitude, float x, float y, float z, Quaternion rotation) {
     // Helpful resource for this function
    // https://github.com/DanielArnett/360-VJ/blob/d50b68d522190c726df44147c5301a7159bf6c86/ShaderMaker.cpp#L678
    // // Create a ray from the latitude and longitude

    // //testing quaternion
    // Quaternion zyx;
    // zyx.CreateFromRollYawPitch(1.588, 0.733,0.401);//zyx
    // Quaternion extractYaw = zyx;
    // extractYaw.m_x = 0;
    // extractYaw.m_y = 0;
    // extractYaw.normalize();
    // // extractYaw.CreateFromRollYawPitch(1.588,0,0);
    // zyx = zyx * extractYaw.getConjugate();
    // vec3 test = zyx.getRollYawPitch();
    // cout<<zyx.m_w << " " <<zyx.m_x<<" "<<zyx.m_y<< " " <<zyx.m_z<<endl;
    // cout<<test.m_x<<" "<<test.m_y<< " " <<test.m_z<<endl;

    // // Quaternion extractYaw;
    // // extractYaw.CreateFromRollYawPitch(1.588,0,0);
    // // extractYaw = extractYaw*extractYaw.getConjugate();
    // // vec3 yawVec3 = extractYaw.getRollYawPitch();
    // // cout<<extractYaw.m_w << " " <<extractYaw.m_x<<" "<<extractYaw.m_y<< " " <<extractYaw.m_z<<endl;
    // // cout<<yawVec3.m_x<<" "<<yawVec3.m_y<< " " <<yawVec3.m_z<<endl;

    // exit(0);
    // rotation.CreateFromRollYawPitch(0,0,0); //roll yaw pitch
    Quaternion align;
    align.CreateFromRollYawPitch(0,M_PI,0);
    rotation = rotation * align;
    Quaternion ignore_yaw = rotation;
    ignore_yaw.m_x = 0;
    ignore_yaw.m_z = 0;
    ignore_yaw.normalize();
    rotation = rotation*ignore_yaw.getConjugate();
    Quaternion rotationConj = rotation.getConjugate();
    // Rotate the ray based on the user input
    Quaternion p(x,y,z,0);
    Quaternion result = (rotationConj*p)*rotation;

    // Convert back to latitude and longitude
    latitude = asin(result.m_y);
    longitude = atan2(result.m_x, result.m_z);

    // Convert back to the normalized M_PIxel coordinate
    float x_rot = (longitude + M_PI) / (2.0 * M_PI);
    float y_rot = (latitude + M_PI / 2.0) / M_PI;
    static float uv[2];
    // Convert to xy source M_PIxel coordinate
    uv[1] = y_rot;
    uv[0] = x_rot;

    return uv;
}

void Pack::packFace(Mat &in, Quaternion rotation, int faceID){
    int height = packedCoords[faceID][4];
    int width = packedCoords[faceID][5];
    Mat packedFace =  Mat(height, width, in.type());
    computeFaceMap(in, packedFace, faceID, rotation);
    packedFace.copyTo(packed(Rect(packedCoords[faceID][2],  packedCoords[faceID][3],packedFace.cols, packedFace.rows)));
}
void Pack::pack(Mat &in, Quaternion rotation, int frame_num){
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

