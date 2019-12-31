#ifndef PID_h
#define PID_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class PID
{
    public:

	PID(float P,float I, float D);
	float calc(float ref, float med, float rate);
    void reset(void);
    private:
    double uik1;
	double errork1;
	double dt;
	float KP,KI,KD;
};

#endif