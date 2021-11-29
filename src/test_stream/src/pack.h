#include <opencv2/opencv.hpp>
using namespace cv;
class Pack {
    public:
        
        void computeFaceMap(Mat &in, Mat &face, int faceID, int faceType, float rotation[4]);
        Mat pack(Mat &in, float rotation[4]);
        Mat pack2(Mat &in, float rotation[4]);
        Pack(Mat m, float r[4]);
        Mat mat;
    private:
        const float AN = sin(M_PI / 4);
        const float AK = cos(M_PI / 4);
        const float CUBESIZE = 480;
        float faceTransform[6][2] = {{-M_PI / 2, 0},{0, 0},{M_PI / 2, 0},{M_PI, 0}, {0, -M_PI / 2}, {0, M_PI / 2}}; // left front right back top bottom
        float rotation[4];
        float packedCoords[14][2] = {
        {CUBESIZE*4,0},             //leftTop
        {0,0},                      //left                   
        {CUBESIZE*4,CUBESIZE/16},   //leftBottom
        {CUBESIZE*4,CUBESIZE/16*2}, //frontTop
        {CUBESIZE,0},               //front
        {CUBESIZE*4,CUBESIZE/16*3}, //frontBot
        {CUBESIZE*4,CUBESIZE/16*4}, //rightTop
        {CUBESIZE*2,0},             //right
        {CUBESIZE*4,CUBESIZE/16*5}, //rightBott
        {CUBESIZE*4,CUBESIZE/16*6}, //backTop
        {CUBESIZE*3,0},             //back
        {CUBESIZE*4,CUBESIZE/16*7}, //backBott
        {CUBESIZE*4+CUBESIZE/4,0},  //top
        {CUBESIZE*4+CUBESIZE/4,CUBESIZE/4}}; //bott

        float packedCoords2[14][2] = {
        {CUBESIZE*2,0},             //leftTop
        {0,0},                      //left                   
        {CUBESIZE*2,CUBESIZE*0.1f},   //leftBottom
        {CUBESIZE*2,CUBESIZE*2*0.1f}, //frontTop
        {CUBESIZE,0},               //front
        {CUBESIZE*2,CUBESIZE*3*0.1f}, //frontBot
        {CUBESIZE*2,CUBESIZE*4*0.1f}, //rightTop
        {0,CUBESIZE*0.5f},             //right
        {CUBESIZE*2,CUBESIZE*5*0.1f}, //rightBott
        {CUBESIZE*2,CUBESIZE*6*0.1f}, //backTop
        {CUBESIZE,CUBESIZE*0.5f},             //back
        {CUBESIZE*2,CUBESIZE*7*0.1f}, //backBott
        {CUBESIZE*2,CUBESIZE*8*0.1f},  //top
        {CUBESIZE*2+CUBESIZE*0.2f,CUBESIZE*8*0.1f}}; //bott

        float cubeCoords[14][2] = {
        {CUBESIZE*3,CUBESIZE},          //leftTop
        {CUBESIZE*3, CUBESIZE*1.25f},   //left
        {CUBESIZE*3,CUBESIZE*1.75f},    //leftBottom
        {0,CUBESIZE},                   //frontTop
        {0,CUBESIZE*1.25f},             //front
        {0,CUBESIZE*1.75f},             //frontBottom
        {CUBESIZE,CUBESIZE},            //rightTop
        {CUBESIZE, CUBESIZE*1.25f},     //right
        {CUBESIZE,CUBESIZE*1.75f},      //rightBottom
        {CUBESIZE*2,CUBESIZE},          //backTop
        {CUBESIZE*2, CUBESIZE*1.25f},   //back
        {CUBESIZE*2,CUBESIZE*1.75f},    //backBottom
        {CUBESIZE,0},                   //top
        {CUBESIZE,CUBESIZE*2} };  

        float* new_rotation(float u, float v, float * rotation, float inHeight, float inWidth);
        float *quaternion_mult(const float q[4], const float r[4]) ;
        float *quaternion_inverse(const float q[4]);  
        float *CreateFromYawPitchRoll(float yaw, float pitch,
                              float roll) ;
        float*** precompute();
        float*** uvStore;
        };

