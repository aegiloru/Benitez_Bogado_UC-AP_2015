#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID2.h"

PID2::PID2(float P, float I, float D){
	this->uik1=0;
	this->errork1=0;
	this->KP_ant=P;
	this->KI_ant=I;
	this->KD_ant=D;
									}
double PID2::calc(float ref, double med, float rate,float P, float I, float D,float dt){
		double error, U;
		double up=0,ui=0,ud=0;
		float KP,KI,KD;
		KP=KP_ant + P;
		KI=KI_ant + I;
		KD=KD_ant + D;
		error=ref-med;
		up=KP*error;
		if (uik1>1){
		   ui=1;
		} 
		else	if (uik1<(-1)){
		   ui=-1;
		} 
		else {
		   ui=uik1+KI*(dt/2)*(error + errork1);
		}
		ud=KD*rate;
		U=(ui+ud+up);
		uik1=ui;
		errork1=error;
		return U;
												}	
void PID2::reset(void){
   uik1=0;
	errork1=0;	}									
		
	