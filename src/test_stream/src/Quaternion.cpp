#include "Quaternion.h"
#include <stdio.h>
#include <math.h>
using namespace std;
#define PI 3.14159265

Quaternion::Quaternion()
{
	m_x = m_y = m_z = 0.0f;
	m_w = 1.0f;
}

Quaternion::Quaternion(float x, float y, float z, float w)
{
	m_x = x;
	m_y = y;
	m_z = z;
	m_w = w;
}

Quaternion::~Quaternion()
{

}

void Quaternion::reset()
{
	m_x = m_y = m_z = 0.0f;
	m_w = 1.0f;
}

void Quaternion::CreateFromAxisAngle(float x, float y, float z, float degrees)
{
	// First we want to convert the degrees to radians 
	// since the angle is assumed to be in radians
	float angle = float((degrees / 180.0f) * PI);

	// Here we calculate the sin( theta / 2) once for optimization
	float result = (float)sin( angle / 2.0f );
		
	// Calcualte the w value by cos( theta / 2 )
	m_w = (float)cos( angle / 2.0f );

	// Calculate the x, y and z of the quaternion
	m_x = float(x * result);
	m_y = float(y * result);
	m_z = float(z * result);
}


void Quaternion::CreateFromRollYawPitch(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
	// Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	m_w = cy * cp * cr + sy * sp * sr;
	m_x = cy * cp * sr - sy * sp * cr;
	m_y = sy * cp * sr + cy * sp * cr;
	m_z = sy * cp * cr - cy * sp * sr;
}

void Quaternion::CreateMatrix(float *pMatrix)
{
	// Make sure the matrix has allocated memory to store the rotation data
	if(!pMatrix) return;

	// First row
	pMatrix[ 0] = 1.0f - 2.0f * ( m_y * m_y + m_z * m_z ); 
	pMatrix[ 1] = 2.0f * (m_x * m_y + m_z * m_w);
	pMatrix[ 2] = 2.0f * (m_x * m_z - m_y * m_w);
	pMatrix[ 3] = 0.0f;  

	// Second row
	pMatrix[ 4] = 2.0f * ( m_x * m_y - m_z * m_w );  
	pMatrix[ 5] = 1.0f - 2.0f * ( m_x * m_x + m_z * m_z ); 
	pMatrix[ 6] = 2.0f * (m_z * m_y + m_x * m_w );  
	pMatrix[ 7] = 0.0f;  

	// Third row
	pMatrix[ 8] = 2.0f * ( m_x * m_z + m_y * m_w );
	pMatrix[ 9] = 2.0f * ( m_y * m_z - m_x * m_w );
	pMatrix[10] = 1.0f - 2.0f * ( m_x * m_x + m_y * m_y );  
	pMatrix[11] = 0.0f;  

	// Fourth row
	pMatrix[12] = 0;  
	pMatrix[13] = 0;  
	pMatrix[14] = 0;  
	pMatrix[15] = 1.0f;

	// Now pMatrix[] is a 4x4 homogeneous matrix that can be applied to an OpenGL Matrix
}

Quaternion Quaternion::operator *(Quaternion q)
{
	Quaternion r;
	r.m_x = m_w*q.m_x + m_x*q.m_w + m_y*q.m_z - m_z*q.m_y;
	r.m_y = m_w*q.m_y + m_y*q.m_w + m_z*q.m_x - m_x*q.m_z;
	r.m_z = m_w*q.m_z + m_z*q.m_w + m_x*q.m_y - m_y*q.m_x;
	r.m_w = m_w*q.m_w - m_x*q.m_x - m_y*q.m_y - m_z*q.m_z;
	
	return(r);
}

float Quaternion::getMagnitude()
{
	return sqrt(m_w*m_w + m_x*m_x + m_y*m_y + m_z*m_z);
}

void Quaternion::normalize()
{
	float mag = getMagnitude();
	if (mag !=0)
	{
		m_w /= mag;
		m_x /= mag;
		m_y /= mag;
		m_z /= mag;      
	}  
}
vec3 Quaternion::getRollYawPitch() {
    vec3 angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (m_w * m_x + m_y * m_z);
    double cosr_cosp = 1 - 2 * (m_x * m_x + m_y * m_y);
    angles.m_x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (m_w * m_y - m_z * m_x);
    if (std::abs(sinp) >= 1)
        angles.m_y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.m_y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (m_w * m_z + m_x * m_y);
    double cosy_cosp = 1 - 2 * (m_y * m_y + m_z * m_z);
    angles.m_z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
Quaternion Quaternion::getConjugate()
{
	return Quaternion(-m_x, -m_y, -m_z, m_w);
}
