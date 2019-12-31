function PID_X
global A
persistent p_contx;
  % establece los parametros que permaneceran en la simulacion
  if A.init==0
      p_contx=0;
  end 
p_contx=p_contx+1;  
if (p_contx==20)
A.X_des = A.X_des_EF*cos(A.psi)+A.Y_des_EF*sin(A.psi);% calculating the desired X in BF
%X_des=A.X_des
%A.X_des = A.X_des_EF*(A.q0^2+A.q1^2-A.q2^2-A.q3^2)+A.Y_des_EF*2*(A.q1*A.q2+A.q0*A.q3);
%Xq_des=A.X_des
A.X_BF = (A.X+A.X_error(A.counter))*cos(A.psi_meas)+(A.Y+A.Y_error(A.counter))*sin(A.psi_meas);            % calculating X in BF
%A.X_BF = A.X*(A.q0^2+A.q1^2-A.q2^2-A.q3^2)+A.Y*2*(A.q1*A.q2+A.q0*A.q3);
p_contx=0;
end 
A.X_dot_BF = A.X_dot*cos(A.psi_meas)+A.Y_dot*sin(A.psi_meas);    % calculating X_dot in BF
%A.X_dot_BF = A.X_dot*(A.q0^2+A.q1^2-A.q2^2-A.q3^2)+A.Y_dot*2*(A.q1*A.q2+A.q0*A.q3);

% PD controller for X_position
A.theta_des = A.X_KP*(A.X_des - A.X_BF) + A.X_KD*A.X_dot_BF;

if(abs(A.theta_des) > 0.2)        % limiter
    A.theta_des = sign(A.theta_des)*0.2;
end
end