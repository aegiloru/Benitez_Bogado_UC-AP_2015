//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files


/*#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif*/
#include "MadgwickAHRS.h"
#include <math.h>
//---------------------------------------------------------------------------------------------------
// Definitions

	// quaternion of sensor frame relative to auxiliary frame

//#define deltat 0.01f // sampling period in seconds (shown as 1 ms)
#define twoKpDef	(5.5f)	// 2 * proportional gain
#define twoKiDef	(1.0f)	// 2 * integral gain
#define gyroMeasError 3.14159265358979 * (3.5f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.13f/ 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//---------------------------------------------------------------------------------------------------
//====================================================================================================
// Functions
float invSqrt(float x);
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MadgwickAHRS::update2(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z,float &SEq_1, float &SEq_2, float &SEq_3,float &SEq_4,float &b_x, float &b_z,float &w_bx, float &w_by, float &w_bz,float &beta3,float dt)
{
// local system variables
float &deltat=dt;
float norm; // vector norm
float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
float h_x, h_y, h_z; // computed flux in the earth frame
// axulirary variables to avoid reapeated calcualtions
float halfSEq_1 = 0.5f * SEq_1;
float halfSEq_2 = 0.5f * SEq_2;
float halfSEq_3 = 0.5f * SEq_3;
float halfSEq_4 = 0.5f * SEq_4;
float twoSEq_1 = 2.0f * SEq_1;
float twoSEq_2 = 2.0f * SEq_2;
float twoSEq_3 = 2.0f * SEq_3;
float twoSEq_4 = 2.0f * SEq_4;
float twob_x = 2.0f * b_x;
float twob_z = 2.0f * b_z;
float twob_xSEq_1 = 2.0f * b_x * SEq_1;
float twob_xSEq_2 = 2.0f * b_x * SEq_2;
float twob_xSEq_3 = 2.0f * b_x * SEq_3;
float twob_xSEq_4 = 2.0f * b_x * SEq_4;
float twob_zSEq_1 = 2.0f * b_z * SEq_1;
float twob_zSEq_2 = 2.0f * b_z * SEq_2;
float twob_zSEq_3 = 2.0f * b_z * SEq_3;
float twob_zSEq_4 = 2.0f * b_z * SEq_4;
float SEq_1SEq_2;
float SEq_1SEq_3 = SEq_1 * SEq_3;
float SEq_1SEq_4;
float SEq_2SEq_3;
float SEq_2SEq_4 = SEq_2 * SEq_4;
float SEq_3SEq_4;
float twom_x = 2.0f * m_x;
float twom_y = 2.0f * m_y;
float twom_z = 2.0f * m_z;
// normalise the accelerometer measurement
norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
a_x /= norm;
a_y /= norm;
a_z /= norm;
// normalise the magnetometer measurement
norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
m_x /= norm;
m_y /= norm;
m_z /= norm;
// compute the objective function and Jacobian
f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
J_12or23 = 2.0f * SEq_4;
J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
J_14or21 = twoSEq_2;
J_32 = 2.0f * J_14or21; // negated in matrix multiplication
J_33 = 2.0f * J_11or24; // negated in matrix multiplication
J_41 = twob_zSEq_3; // negated in matrix multiplication
J_42 = twob_zSEq_4;
J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
J_52 = twob_xSEq_3 + twob_zSEq_1;
J_53 = twob_xSEq_2 + twob_zSEq_4;
J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
J_61 = twob_xSEq_3;
J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
J_64 = twob_xSEq_2;
// compute the gradient (matrix multiplication)
SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
// normalise the gradient to estimate direction of the gyroscope error
norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
SEqHatDot_1 = SEqHatDot_1 / norm;
SEqHatDot_2 = SEqHatDot_2 / norm;
SEqHatDot_3 = SEqHatDot_3 / norm;
SEqHatDot_4 = SEqHatDot_4 / norm;
// compute angular estimated direction of the gyroscope error
w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
// compute and remove the gyroscope baises
w_bx += w_err_x * deltat * zeta;
w_by += w_err_y * deltat * zeta;
w_bz += w_err_z * deltat * zeta;
w_x -= w_bx;
w_y -= w_by;
w_z -= w_bz;
// compute the quaternion rate measured by gyroscopes
SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
// compute then integrate the estimated quaternion rate
SEq_1 += (SEqDot_omega_1 - ( SEqHatDot_1*beta3)) * deltat;
SEq_2 += (SEqDot_omega_2 - ( SEqHatDot_2*beta3)) * deltat;
SEq_3 += (SEqDot_omega_3 - ( SEqHatDot_3*beta3)) * deltat;
SEq_4 += (SEqDot_omega_4 - ( SEqHatDot_4*beta3)) * deltat;
/*SEq_1 = ((SEq_1 +SEqDot_omega_1* deltat)*0.96 + (SEq_1-SEqHatDot_1*beta3)*0.04) ;
SEq_2 = ((SEq_2 +SEqDot_omega_2* deltat)*0.96 + (SEq_2-SEqHatDot_2*beta3)*0.04) ;
SEq_3 = ((SEq_3 +SEqDot_omega_3* deltat)*0.96 + (SEq_3-SEqHatDot_3*beta3)*0.04) ;
SEq_4 = ((SEq_4 +SEqDot_omega_4* deltat)*0.96 + (SEq_4-SEqHatDot_4*beta3)*0.04) ;*/
// normalise quaternion
norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
SEq_1 /= norm;
SEq_2 /= norm;
SEq_3 /= norm;
SEq_4 /= norm;
// compute flux in the earth frame
SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
SEq_1SEq_3 = SEq_1 * SEq_3;
SEq_1SEq_4 = SEq_1 * SEq_4;
SEq_3SEq_4 = SEq_3 * SEq_4;
SEq_2SEq_3 = SEq_2 * SEq_3;
SEq_2SEq_4 = SEq_2 * SEq_4;
h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
// normalise the flux vector to have only components in the x and z
b_x = sqrt((h_x * h_x) + (h_y * h_y));
b_z = h_z;

	
}


void MadgwickAHRS::updateIMU2(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z,float &SEq_1, float &SEq_2, float &SEq_3,float &SEq_4, float &beta2,float dt)
{
// Local system variables
float &deltat=dt;
float norm; // vector norm
float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
float f_1, f_2, f_3; // objective function elements
float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
// Axulirary variables to avoid reapeated calcualtions
float halfSEq_1 = 0.5f * SEq_1;
float halfSEq_2 = 0.5f * SEq_2;
float halfSEq_3 = 0.5f * SEq_3;
float halfSEq_4 = 0.5f * SEq_4;
float twoSEq_1 = 2.0f * SEq_1;
float twoSEq_2 = 2.0f * SEq_2;
float twoSEq_3 = 2.0f * SEq_3;
// Normalise the accelerometer measurement
norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
norm=1/norm;
a_x *= norm;
a_y *= norm;
a_z *= norm;
// Compute the objective function and Jacobian
f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
J_12or23 = 2.0f * SEq_4;
J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
J_14or21 = twoSEq_2;
J_32 = 2.0f * J_14or21; // negated in matrix multiplication
J_33 = 2.0f * J_11or24; // negated in matrix multiplication
// Compute the gradient (matrix multiplication)
SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
// Normalise the gradient
norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
norm=1/norm;
SEqHatDot_1 *= norm;
SEqHatDot_2 *= norm;
SEqHatDot_3 *= norm;
SEqHatDot_4 *= norm;
// Compute the quaternion derrivative measured by gyroscopes
SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;



// Compute then integrate the estimated quaternion derrivative
float SEq_1_ant=SEq_1;
float SEq_2_ant=SEq_2;
float SEq_3_ant=SEq_3;
float SEq_4_ant=SEq_4;

/*SEq_1 = ((SEq_1 +SEqDot_omega_1* deltat)*0.996 + (SEq_1-SEqHatDot_1*beta2)*0.004) ;
SEq_2 = ((SEq_2 +SEqDot_omega_2* deltat)*0.996 + (SEq_2-SEqHatDot_2*beta2)*0.004) ;
SEq_3 = ((SEq_3 +SEqDot_omega_3* deltat)*0.996 + (SEq_3-SEqHatDot_3*beta2)*0.004) ;
SEq_4 = ((SEq_4 +SEqDot_omega_4* deltat)*0.996 + (SEq_4-SEqHatDot_4*beta2)*0.004) ;*/
SEq_1 += (SEqDot_omega_1 - ( SEqHatDot_1*beta2)) * deltat;
SEq_2 += (SEqDot_omega_2 - ( SEqHatDot_2*beta2)) * deltat;
SEq_3 += (SEqDot_omega_3 - ( SEqHatDot_3*beta2)) * deltat;
SEq_4 += (SEqDot_omega_4 - ( SEqHatDot_4*beta2)) * deltat;
if (isnan(SEq_1)){
SEq_1=SEq_1_ant;
}
if (isnan(SEq_2)){
SEq_2=SEq_2_ant;
}
if (isnan(SEq_3)){
SEq_3=SEq_3_ant;
}
if (isnan(SEq_4)){
SEq_4=SEq_4_ant;
}
// Normalise quaternion
norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
norm=1/norm;

SEq_1 *= norm;
SEq_2 *= norm;
SEq_3 *= norm;
SEq_4 *= norm;

}

void MadgwickAHRS::MahonyupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float &q0, float &q1, float &q2, float &q3, float &integralFBx, float &integralFBy,float &integralFBz, float dt) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1/sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f * dt);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f * dt);
		//	integralFBz += twoKi * halfez * (1.0f * dt);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			//gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (dt));		// pre-multiply common factors
	gy *= (0.5f * (dt));
	gz *= (0.5f * (dt));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
};

//====================================================================================================
// END OF CODE
//====================================================================================================
