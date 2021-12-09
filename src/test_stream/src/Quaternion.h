#include <stdio.h>
#include "vec3.h"
using namespace std;
class Quaternion{
    public:
        Quaternion();
        Quaternion(float x, float y, float z, float w);
        ~Quaternion();
        void reset();
        void CreateFromAxisAngle(float x, float y, float z, float degrees);
        void CreateFromRollYawPitch(double yaw, double pitch, double roll);
        void CreateMatrix(float *pMatrix);
        Quaternion operator *(Quaternion q);
        float getMagnitude();
        void normalize();
        Quaternion getConjugate();
        vec3 getRollYawPitch();

        float m_x, m_y, m_z, m_w;

};
