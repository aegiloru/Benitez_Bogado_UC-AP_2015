#ifndef FuerzasyAerodinamica_h
#define FuerzasyAerodinamica_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
struct Omega{
	float Omega_1;
	float Omega_2;
	float Omega_3;
	float Omega_4;
	};
struct PWMX{
    unsigned int PWM1;
	unsigned int PWM2;
	unsigned int PWM3;
	unsigned int PWM4;
	};
class FuerzasyAerodinamica
{
    public:

	FuerzasyAerodinamica(float drag_N,float lift_N, float radio,float U1_A, float U2_B,float U3_C,float U4_D);
	Omega calcRot(float &U1, float &U2, float &U3,float &U4);
	Omega calcRotX(float &U1, float &U2, float &U3,float &U4);
    PWMX calcPWM(float Omega_1,float Omega_2,float Omega_3, float Omega_4);
	PWMX calcRotX_PWM(float &U1, float &U2, float &U3,float &U4);
    private:
    float d, b, l,U1A,U2B,U3C,U4D;
	Omega r;
	PWMX VM;
};

#endif