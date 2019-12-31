//=====================================================================================================
// AHRS.h
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
//
// See AHRS.c file for description.
// 
//=====================================================================================================
#ifndef AHRS_h
#define AHRS_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
//----------------------------------------------------------------------------------------------------
// Variable declaration

struct VectorX
{
    float qt0;
    float qt1;
    float qt2;
    float qt3;
	float qt4;
	float qt5;
	float qt6;

};	// quaternion elements representing the estimated orientation

//---------------------------------------------------------------------------------------------------
// Function declaration
class AHRS
{
 public:
VectorX update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float exInt, float eyInt, float ezInt, float q0, float q1, float q2, float q3);
private:
VectorX qu;


// Function declarations


};

#endif

//=====================================================================================================
// End of file
//=====================================================================================================