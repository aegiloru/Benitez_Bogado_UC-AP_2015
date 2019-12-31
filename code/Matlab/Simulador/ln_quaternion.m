function ln_quaternion
global A;
 persistent uik2;
  persistent errork2;
   persistent uik3;
  persistent errork3;
   persistent uik4;
  persistent errork4;
  persistent r_ant;
  persistent q_ant;
  persistent p_ant;
  persistent c;
  persistent error_p;
  persistent error_q;
  persistent error_r;
  persistent ui_q;
  persistent ui_p;
  persistent ui_r;
  persistent ud_gx;
  persistent ud_gy;
  persistent ud_gz;
  if A.init==0, 
      uik2    = 0; 
      errork2 = 0;
      uik3    = 0; 
      errork3 = 0;
      uik4    = 0; 
      errork4 = 0;
      c=1;
      error_p=0;
      error_q=0;
      error_r=0;
      ui_p=0;
      ui_q=0;
      ui_r=0;
     r_ant=0;
     p_ant=0;
     q_ant=0;
     ud_gx=0;
     ud_gy=0;
     ud_gz=0;
  end
  
if (c==1)
q=[A.q0 -A.q1 -A.q2 -A.q3];

r=[A.q0_des A.q1_des A.q2_des A.q3_des];
q_m=quatconj(q);
if (r(1)<0)
    r=-r;
end 
n = quatmultiply(q,r);

if ((n(1))<0)
    n=-n;
end
  
a_error=2*acos(n(1));
sin_error=sin(a_error/2);
if (sin_error==0)
    sin_error=1;
end

A.Ex=n(2)*a_error/sin_error;
A.Ey=n(3)*a_error/sin_error;
A.Ez=n(4)*a_error/sin_error;


%    if (abs(100*A.Ex)<0.05)
%      A.phi_KI=0.6;
    
%    else 
%      A.phi_KI=0.7;
%    end
%    if (abs(100*A.Ey)<0.05)
%      A.theta_KI=0.6;
%    else 
%      A.theta_KI=0.7;
 %   end 
%ui2=uik2 + A.phi_KI* A.Ts * (A.Ex + errork2);
%ui2=max(-0.2,min(0.2,ui2));
%ui3=uik3 + A.theta_KI * A.Ts * (A.Ey + errork3);
%ui3=max(-0.2,min(0.2,ui3));
%ui4=uik4 + 0.35* A.Ts * (-A.r+A.r_des);
%if (abs(A.Ex)<0.13)
%  p_error=0;
%else 
%  p_error=A.p*0.75;
%end
%if (abs(A.Ey)<0.13)
%  q_error=0;
%else 
%  q_error=A.q*0.75;
%end
%A.U2=A.phi_KP*A.Ex+A.phi_KD*(A.p-p_error) + ui2 ;
%A.U3=A.theta_KP*A.Ey+A.phi_KD*(A.q-q_error) + ui3;
A.p_des=A.phi_KP*A.Ex;
A.q_des=A.theta_KP*A.Ey;
%if (abs(A.phi)<A.pi/2 && abs(A.theta)<A.pi/2 )
A.r_des=3*((A.psi_des-A.psi_meas));
%else
%A.r_des=0.85*A.Ez;
%end
  

%A.r_des=0.85*A.Ez;
%j=A.q_des
%errork2=A.Ex;
%errork3=A.Ey;
%uik2=ui2;
%uik3=ui3;
%uik4=ui4;
%A.U4=-0.8*(A.r-A.r_des)-0.5*(A.r-r_ant)+ui4;

c=0;
end
c=c+1;

errorp=A.p_des-A.p-A.p_error(A.counter);
errorq=A.q_des-A.q-A.q_error(A.counter);
errorr=A.r_des-A.r-A.r_error(A.counter);
uiq=ui_q+0.35*A.Ts*errorq;
uiq=max(-1,min(1,uiq));
uip=ui_p+0.35*A.Ts*errorp;
uip=max(-1,min(1,uip));
uir=ui_r+0.21*A.Ts*errorr;
uir=max(-1,min(1,uir));
ud_gxn=0.01*(errorp-error_p)*(1/A.Ts);
ud_gyn=0.01*(errorq-error_q)*(1/A.Ts);
ud_gzn=0.01*(errorr-error_r)*(1/A.Ts);
%LowPASS FILTERING
ud_gx=ud_gxn*0.531914+ud_gx*0.468086;
ud_gy=ud_gxn*0.531914+ud_gy*0.468086;
%ud_gz=ud_gxn*0.166666+ud_gz*0.833334;
A.U2=0.75*(errorp)+uip+ud_gx;
A.U3=0.75*(errorq)+uiq+ud_gy;
A.U4=0.7*(errorr)+uir;

r_ant=A.r;
q_ant=A.q;
p_ant=A.p;
ui_p=uip;
ui_q=uiq;
ui_r=uir;
%A.U3_plot(A.counter)=A.q;
error_q=errorq;
error_r=errorr;
error_p=errorp;

%errork4=A.Ez;

end