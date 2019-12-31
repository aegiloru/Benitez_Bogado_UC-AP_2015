//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
struct Vector{
  float Xaxis;
  float Yaxis;
  float Zaxis;
};
struct VectorX
{
    float qt0;
    float qt1;
    float qt2;
    float qt3;
	float qt4;
	float qt5;
	float qt6;
	float qt7;
	float qt8;
};

//----------------------------------------------------------------------------------------------------
// Variable declaration
class MadgwickAHRS
{
 public:
	// quaternion of sensor frame relative to auxiliary frame


//---------------------------------------------------------------------------------------------------
// Function declarations

void update2(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z,float &SEq_1, float &SEq_2, float &SEq_3,float &SEq_4,float &b_x, float &b_z,float &w_bx, float &w_by, float &w_bz,float &beta3,float dt);
void updateIMU2(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z,float &SEq_1, float &SEq_2, float &SEq_3,float &SEq_4,float &beta2,float dt);
void MahonyupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float &q0, float &q1, float &q2, float &q3, float &integralFBx, float &integralFBy,float &integralFBz, float dt);

// Function declarations


};

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
