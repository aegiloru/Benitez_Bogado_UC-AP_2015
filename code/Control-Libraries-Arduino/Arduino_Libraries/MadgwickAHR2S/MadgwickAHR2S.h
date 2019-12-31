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
#ifndef MadgwickAHR2S_h
#define MadgwickAHR2S_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
/*
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
};*/

//----------------------------------------------------------------------------------------------------
// Variable declaration
class MadgwickAHR2S
{
 public:
	// quaternion of sensor frame relative to auxiliary frame


//---------------------------------------------------------------------------------------------------
// Function declarations

//VectorX update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,double q0,double q1,double q2,double q3,float beta3,float dt);
void updateIMU(float &gx, float &gy, float &gz, float &ax, float &ay, float &az,float &q0,float &q1,float &q2,float &q3,float beta2,float &dt);
/*private:
VectorX qu;
VectorX qur;*/


};

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
