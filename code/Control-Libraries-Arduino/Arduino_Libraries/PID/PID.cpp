#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID.h"

PID::PID(float P, float I, float D){
	double dt=0.01;
	double uik1=0;
	double errork1=0;
	float KP=P;
	float KI=I;
	float KD=D;
									}
float PID::calc(float ref, float med, float rate){
		float error, U;
		double up=0,ui=0,ud=0;
		error=ref-med;
		up=KP*error;
		if (uik1>1.7){
		   ui=1.7
		} 
		else {
		   ui=uik1+KI*(dt/2)*(error + errork1);
		}
		ud=KD*rate;
		U=(float)(ui+ud+up);
		uik1=ui;
		errork1=error;
		return U;
												}	
void PID::reset(void){
    double uik1=0;
	double errork1=0;	}									
		
	