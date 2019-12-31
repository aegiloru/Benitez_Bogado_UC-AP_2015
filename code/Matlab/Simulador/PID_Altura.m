function PID_Altura
global A;
%persistent ui;
  persistent alt_ant;
  persistent E_vel_ant;
  persistent uvkz;
  persistent vel_ant;
 % persistent uikz;
  persistent d;
  % establece los parametros que permaneceran en la simulacion
  if A.init==0
 %     ui    = 0; 
      errork1 = 0;
      vhatk1 = 0;
      E_vel_ant=0;
      uikz=0;
      uvkz=0;
      x_p=0;
      alt_ant=0;
      vel_ant=0;
      d=0;
  end
  d=d+1;
 if (d==2)
     d=0;
 % error = A.Z_des-A.Z;
  E_alt = A.Z_des-A.Z_meas;
  
  %Control Proporcional
 x_f=(A.Z_dot*A.Z_dot - vel_ant*vel_ant)*(1/2*(A.Z_ddot));  
 up = A.Z_KP * (E_alt + 0*(x_f+alt_ant-A.Z_meas)) ;
  
  % Control Integral
  
 %Control Derivativo
 % Z_ant=A.Z_meas;
%Implementacion del PID
%if (abs(E_alt)>0.08)
  A.Vel_des =min(2.5,max(-2.5,(A.Z_KP*E_alt)));
%else
%  A.Vel_des=0;
%end
  alt_ant=A.Z_meas;
  vel_ant =A.Z_dot;

 end
  
  E_vel=A.Vel_des-A.Z_dot-A.Z_dot_error(A.counter);
  uivz = uvkz + 0.3 * A.Ts/2 * (E_vel+E_vel_ant);
  uivz = max(-6.5,min(6.5,uivz));
  udv=0.0015*(E_vel-E_vel_ant)*(1/A.Ts);
  upv=E_vel*2.9;
  F=upv+udv+uivz;
  A.U1=(F+A.m*A.g)/(A.q0*A.q0-A.q1*A.q1-A.q2*A.q2+A.q3*A.q3);
  % Actualizacion de la variable para KI
  E_vel_ant=E_vel;
  uvkz=uivz;
  
end