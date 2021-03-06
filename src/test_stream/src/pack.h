#include <opencv2/opencv.hpp>
using namespace cv;
class Pack {
    public:
        void computeFaceMap(Mat &in, Mat &face, int faceID);
        void pack(Mat &in, int frame_num);
        void precompute();
        Pack();
        Mat packed;
        Mat unpacked;
        void unpack();

    private:
        const float AN = sin(M_PI / 4);
        const float AK = cos(M_PI / 4);
        const float CUBESIZE = 960;
        float faceTransform[6][2] = {{-M_PI / 2, 0},{0, 0},{M_PI / 2, 0},{M_PI, 0}, {0, -M_PI / 2}, {0, M_PI / 2}}; // left front right back top bottom
        float rotation[4];
        // Mat uvPrecomp[16][2];
        float **** uvPrecomp;
        float packedCoords[14][6] = { //faceID -> faceTransform,faceType, posx, posy, height, width 
        {0, 0, CUBESIZE*2,0,CUBESIZE*0.1f, CUBESIZE*0.4f},             //leftTop
        {0, 1, 0,0,CUBESIZE/2, CUBESIZE},                      //left                   
        {0, 2, CUBESIZE*2,CUBESIZE*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f},   //leftBottom
        {1, 0, CUBESIZE*2,CUBESIZE*2*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f}, //frontTop
        {1, 1, CUBESIZE,0,CUBESIZE/2, CUBESIZE},               //front
        {1, 2, CUBESIZE*2,CUBESIZE*3*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f}, //frontBot
        {2, 0, CUBESIZE*2,CUBESIZE*4*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f}, //rightTop
        {2, 1, 0,CUBESIZE*0.5f,CUBESIZE/2, CUBESIZE},             //right
        {2, 2, CUBESIZE*2,CUBESIZE*5*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f}, //rightBott
        {3, 0, CUBESIZE*2,CUBESIZE*6*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f}, //backTop
        {3, 1, CUBESIZE,CUBESIZE*0.5f,CUBESIZE/2, CUBESIZE},             //back
        {3, 2, CUBESIZE*2,CUBESIZE*7*0.1f,CUBESIZE*0.1f, CUBESIZE*0.4f}, //backBott
        {4, 0, CUBESIZE*2,CUBESIZE*8*0.1f,CUBESIZE*0.2f, CUBESIZE*0.2f},  //top
        {5, 0, CUBESIZE*2+CUBESIZE*0.2f,CUBESIZE*8*0.1f,CUBESIZE*0.2f, CUBESIZE*0.2f}}; //bott

        float cubeCoords[14][2] = {
        {0,CUBESIZE},                   //lefttop
        {0,CUBESIZE*1.25f},             //left
        {0,CUBESIZE*1.75f},             //leftbottom
        {CUBESIZE,CUBESIZE},            //fronttop
        {CUBESIZE, CUBESIZE*1.25f},     //frontRight
        {CUBESIZE,CUBESIZE*1.75f},      //frontBottom
        {CUBESIZE*2,CUBESIZE},          //backTop
        {CUBESIZE*2, CUBESIZE*1.25f},   //back
        {CUBESIZE*2,CUBESIZE*1.75f},    //backBottom
        {CUBESIZE*3,CUBESIZE},          //righttop
        {CUBESIZE*3, CUBESIZE*1.25f},   //right
        {CUBESIZE*3,CUBESIZE*1.75f},    //rightbottom                
        {CUBESIZE,0}, //top
        {CUBESIZE,CUBESIZE*2} //bottom
        };  
        float *new_rotation(float longitude, float latitude, float x, float y, float z, float inHeight,
                    float inWidth);
                            float *quaternion_mult(const float q[4], const float r[4]) ;
        float *quaternion_inverse(const float q[4]);  
        float *CreateFromYawPitchRoll(float yaw, float pitch,
                              float roll) ;
        float* computeUV(int x, int y,int faceID );
        void packFace(Mat &in, int faceID);

        };

