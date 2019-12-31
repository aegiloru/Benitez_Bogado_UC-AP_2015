function PID_Y
global A
persistent p_conty;
  % establece los parametros que permaneceran en la simulacion
  if A.init==0
      p_conty=0;
  end 
p_conty=p_conty+1;  
if (p_conty==20)
A.Y_des = A.Y_des_EF*cos(A.psi_meas)-A.X_des_EF*sin(A.psi_meas);    % calculating the desired X in BF
%A.Y_des= A.Y_des_EF*(A.q0^2-A.q1^2+A.q2^2-A.q3^2)+A.X_des_EF*2*(A.q1*A.q2-A.q0*A.q3);
A.Y_BF = (A.Y+A.Y_error(A.counter))*cos(A.psi_meas)-(A.X+A.X_error(A.counter))*sin(A.psi_meas);            % calculating X in BF
%A.Y_BF= A.Y*(A.q0^2-A.q1^2+A.q2^2-A.q3^2)+A.X*2*(A.q1*A.q2-A.q0*A.q3);
p_conty=0;
end
A.Y_dot_BF = A.Y_dot*cos(A.psi_meas)-A.X_dot*sin(A.psi_meas);    % calculating X_dot in BF
%A.Y_dot_BF=A.Y_dot_BF*(A.q0^2-A.q1^2+A.q2^2-A.q3^2)+A.X_dot_BF*2*(A.q1*A.q2-A.q0*A.q3);
% PD controller for X_position
A.phi_des = -1*(A.Y_KP*(A.Y_des - A.Y_BF) + A.Y_KD*A.Y_dot);

if(abs(A.phi_des) >0.2)        % limiter
    A.phi_des = sign(A.phi_des)*0.2;
end
end