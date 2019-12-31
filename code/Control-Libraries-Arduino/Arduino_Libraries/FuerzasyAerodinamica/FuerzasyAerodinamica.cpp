
#include "FuerzasyAerodinamica.h"
#include <math.h>

FuerzasyAerodinamica::FuerzasyAerodinamica(float drag_N,float lift_N, float radio,float U1_A, float U2_B,float U3_C,float U4_D){
		
		this-> b=lift_N;
		this-> l=radio;
		this-> d=drag_N;
		this-> U1A=U1_A;
		this-> U2B=U2_B;
		this-> U3C=U3_C;
		this-> U4D=U4_D;
		}
		
Omega FuerzasyAerodinamica::calcRotX(float &U1, float &U2, float &U3,float &U4){
			float Omega2_1, Omega2_2, Omega2_3,Omega2_4;
		//	U1-=U1A;
		//	U2-=U2B*l;
		//	U3-=U3C*l;
		//	U4-=U4D*0.29;
			float _1_4b=(1/(4*b));
			float _1_4bl=(1/(4*b*l));
			float _1_4d=(1/(4*d));
			Omega2_1=_1_4b*U1 - _1_4bl*U2  - _1_4bl*U3 - _1_4d*U4;
		//	Serial.println(Omega2_1);
			Omega2_3=_1_4b*U1 - _1_4bl*U2  + _1_4bl*U3 + _1_4d*U4;
			Omega2_2=_1_4b*U1 + _1_4bl*U2  + _1_4bl*U3 - _1_4d*U4;
			Omega2_4=_1_4b*U1 + _1_4bl*U2  - _1_4bl*U3 + _1_4d*U4;
			this-> r.Omega_1=sqrt(Omega2_1);
			this-> r.Omega_2=sqrt(Omega2_2);
			this-> r.Omega_3=sqrt(Omega2_3);
			this-> r.Omega_4=sqrt(Omega2_4);
			if (r.Omega_1<100 || Omega2_1<0){
			   this-> r.Omega_1=100;
			   };
			if (r.Omega_2<100|| Omega2_2<0){
			   this-> r.Omega_2=100;
			   };
			if (r.Omega_3<100|| Omega2_3<0){
			   this-> r.Omega_3=100;
			   };
			if (r.Omega_4<100 || Omega2_4<0){
			   this-> r.Omega_4=100;
			   };
			if (r.Omega_1>780){
			   this-> r.Omega_1=780;
			   };
			if (r.Omega_2>780){
			   this-> r.Omega_2=780;
			   };
			if (r.Omega_3>780){
			   this-> r.Omega_3=780;
			   };
			if (r.Omega_4>780){
			   this-> r.Omega_4=780;
			   };
			
			
			return r;
			}
			
PWMX FuerzasyAerodinamica::calcRotX_PWM(float &U1, float &U2, float &U3,float &U4){
		//	U1-=U1A;
		//	U2-=U2B*l;
		//	U3-=U3C*l;
		//	U4-=U4D*0.29;
			float _1_4b=(1/(4*b));
			float _1_4bl=(1/(4*b*l));
			float _1_4d=(1/(4*d));
			this-> r.Omega_1=sqrt(_1_4b*U1 - _1_4bl*U2  - _1_4bl*U3 - _1_4d*U4);
			this-> r.Omega_3=sqrt(_1_4b*U1 - _1_4bl*U2  + _1_4bl*U3 + _1_4d*U4);
			this-> r.Omega_2=sqrt(_1_4b*U1 + _1_4bl*U2  + _1_4bl*U3 - _1_4d*U4);
			this-> r.Omega_4=sqrt(_1_4b*U1 + _1_4bl*U2  - _1_4bl*U3 + _1_4d*U4);
			if (r.Omega_1<150 || isnan(r.Omega_1)==1){
			   this-> r.Omega_1=150;
			   };
			if (r.Omega_2<150 || isnan(r.Omega_2)==1){
			   this-> r.Omega_2=150;
			   };
			if (r.Omega_3<150 || isnan(r.Omega_3)==1){
			   this-> r.Omega_3=150;
			   };
			if (r.Omega_1<150 || isnan(r.Omega_4)==1){
			   this-> r.Omega_4=150;
			   };
			   
			if (r.Omega_1>800){
			   this-> r.Omega_1=800;
			   };
			if (r.Omega_2>800){
			   this-> r.Omega_2=800;
			   };
			if (r.Omega_3>800){
			   this-> r.Omega_3=800;
			   };
			if (r.Omega_4>800){
			   this-> r.Omega_4=800;
			   };
      VM.PWM1=(int)round(1.276*r.Omega_1+889.16);
	  VM.PWM2=(int)round(1.276*r.Omega_2+889.16);
	  VM.PWM3=(int)round(1.276*r.Omega_3+889.16);
	  VM.PWM4=(int)round(1.276*r.Omega_4+889.16);
	
			
			return VM;
			}

Omega FuerzasyAerodinamica::calcRot(float &U1, float &U2, float &U3,float &U4){
			float Omega2_1, Omega2_2, Omega2_3,Omega2_4;
		//	U1-=U1A;
		//	U2-=U2B*l;
		//	U3-=U3C*l;
		//	U4-=U4D*0.29;
			float _1_4b=(1/(4*b));
			float _1_2bl=(1/(2*b*l));
			float _1_4d=(1/(4*d));
			Omega2_1=_1_4b*U1 - _1_2bl*U3 - _1_4d*U4;
			Omega2_3=_1_4b*U1 - _1_2bl*U2 + _1_4d*U4;
			Omega2_2=(_1_4b*U1 + _1_2bl*U3 - _1_4d*U4);
			Omega2_4=_1_4b*U1 + _1_2bl*U2 + _1_4d*U4;
			this-> r.Omega_1=sqrt(Omega2_1);
			this-> r.Omega_2=sqrt(Omega2_2);
			this-> r.Omega_3=sqrt(Omega2_3);
			this-> r.Omega_4=sqrt(Omega2_4);
			if (r.Omega_1<100 || Omega2_1<0){
			   this-> r.Omega_1=100;
			   };
			if (r.Omega_2<100|| Omega2_2<0){
			   this-> r.Omega_2=100;
			   };
			if (r.Omega_3<100|| Omega2_3<0){
			   this-> r.Omega_3=100;
			   };
			if (r.Omega_4<100 || Omega2_4<0){
			   this-> r.Omega_4=100;
			   };
			   
			if (r.Omega_1>800){
			   this-> r.Omega_1=800;
			   };
			if (r.Omega_2>800){
			   this-> r.Omega_2=800;
			   };
			if (r.Omega_3>800){
			   this-> r.Omega_3=800;
			   };
			if (r.Omega_4>800){
			   this-> r.Omega_4=800;
			   };
			
			return r;
			}
PWMX FuerzasyAerodinamica::calcPWM(float Omega_1,float Omega_2,float Omega_3, float Omega_4){
	//VM.PWM1=(0.4810*(Omega_1*0.5)+35.3889);
	//VM.PWM1=round(0.9765*Omega_1+952.2019);
	//VM.PWM2=round(0.9765*Omega_2+952.2019);
	//VM.PWM3=round(0.9765*Omega_3+952.2019);
	//VM.PWM4=round(0.9765*Omega_4+952.2019);
	  VM.PWM1=(int)round(1.31*Omega_1+903);
	  VM.PWM2=(int)round(1.31*Omega_2+903);
	  VM.PWM3=(int)round(1.31*Omega_3+903);
	  VM.PWM4=(int)round(1.31*Omega_4+903);
	
	//VM.PWM2=(0.4810*(Omega_2*0.5)+35.3889);
	//VM.PWM1=(0.4589*(Omega_1/2)+42.7198);
	//VM.PWM2=(0.4402*(Omega_4/2)+44.4801);
	
	//VM.PWM3=(0.4589*(Omega_3/2)+42.7198);
	//VM.PWM4=(0.4402*(Omega_4/2)+44.4801);
	//VM.PWM3=(0.4810*(Omega_3*0.5)+35.3889);
	//VM.PWM4=(0.4810*(Omega_4*0.5)+35.3889);
	
		
	return VM;
	}
	
	
	
	