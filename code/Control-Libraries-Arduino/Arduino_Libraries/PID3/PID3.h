#ifndef PID2_h
#define PID2_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class PID2
{
    public:

	PID2(float P,float I, float D);
	double calc(float ref, double med, float rate,float P, float I, float D,float dt);
    void reset(void);
    private:
    double uik1;
	double errork1;
	double dt;
	float KP_ant,KI_ant,KD_ant;
};

#endif