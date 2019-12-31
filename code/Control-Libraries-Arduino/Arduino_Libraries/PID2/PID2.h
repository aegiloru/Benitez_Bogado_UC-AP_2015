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
	float calc(float &ref, float &med, float &rate,float P, float I, float D,float &dt);
    void reset(void);
    private:
    float uik1;
	float errork1;
	float KP_ant,KI_ant,KD_ant;
};

#endif