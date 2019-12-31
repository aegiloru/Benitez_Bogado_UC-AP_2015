function quadmodel
global A;
% Esta funcion simula la dinamica del cuadricopter tomara los angulos
%  de Euler y las fuerzas como entradas y nos dara la aceleracion lineal
%  WRT y las que son segun el eje Inercial

A.X_ddot = (sin(A.psi)*sin(A.phi) + cos(A.psi)*sin(A.theta)*cos(A.phi))*A.U1/A.m;
A.Y_ddot = (-cos(A.psi)*sin(A.phi) + sin(A.psi)*sin(A.theta)*cos(A.phi))*A.U1/A.m;
A.Z_ddot = -A.g + (cos(A.theta)*cos(A.phi))*A.U1/A.m;                                     



A.p_dot = ((A.Iyy - A.Izz)*A.q*A.r -A.Jtp*A.q*A.O + A.U2)/A.Ixx;
A.q_dot = ((A.Izz - A.Ixx)*A.p*A.r +A.Jtp*A.p*A.O + A.U3)/A.Iyy;
A.r_dot = ((A.Ixx - A.Iyy)*A.p*A.q + A.U4)/A.Izz;

A.phi_dot   = A.p + sin(A.phi)*tan(A.theta)*A.q + cos(A.phi)*tan(A.theta)*A.r;
A.theta_dot = cos(A.phi)*A.q - sin(A.phi)*A.r;
A.psi_dot   = sin(A.phi)/cos(A.theta)*A.q + cos(A.phi)/cos(A.theta)*A.r;

% Modelado de la friccion del aire
%A.X_ddot = A.X_ddot - A.X_dot*1.2;
%A.Y_ddot = A.Y_ddot - A.Y_dot*1.2;

 %    function d1(varargin)   % perturbacion en Z
 %        A.Z_dis = -0.2*A.m*A.g;
 %    end
% 
  %  function d2(varargin)   % Perturbacion en Yaw
  %       A.psi_dis = -200*A.Izz;
  %   end
% 
 %    function d3(varargin)   % Perturbacion en Pitch
 %        A.theta_dis = -200*A.Iyy;
 %    end
    
  %   function d4(varargin)   % Perturbacion en Roll
  %      A.phi_dis = -200*A.Ixx;
  %   end
%    d3;
%    d2;
%    d1;
%    d4;
% Modelado de las perturbaciones segun la aceleracion
%A.Z_ddot = A.Z_ddot + A.Z_dis/A.m;

%A.psi_dot_d = A.psi_dot + A.psi_dis/A.Izz*A.Ts;

%A.theta_dot_d = A.theta_dot + A.theta_dis/A.Iyy*A.Ts;

%A.phi_dot_d = A.phi_dot + A.phi_dis/A.Ixx*A.Ts;


% Calculando la posicion y la velocidad en Z
A.Z_dot = A.Z_ddot*A.Ts + A.Z_dot;
A.Z = A.Z_dot*A.Ts + A.Z ;

% Calculando la posicion y la velocidad en X
A.X_dot = A.X_ddot*A.Ts + A.X_dot;
A.X = A.X_dot*A.Ts + A.X ;

% Calculando la posicion y la velocidad en Y
A.Y_dot = A.Y_ddot*A.Ts + A.Y_dot;
A.Y = A.Y_dot*A.Ts + A.Y ;

% Calculando p,q,r
A.p = A.p_dot*A.Ts+A.p;
A.q = A.q_dot*A.Ts+A.q;
A.r = A.r_dot*A.Ts+A.r;

% Calculando phi,theta,psi
A.phi = A.phi_dot*A.Ts+A.phi;
A.theta = A.theta_dot*A.Ts+A.theta;
A.psi = A.psi_dot*A.Ts+A.psi;

if (abs(A.phi)>A.pi)
    A.phi=A.phi-sign(A.phi)*A.pi*2;
end
if (abs(A.theta)>A.pi)
    A.theta=A.theta-sign(A.theta)*A.pi*2;
end
if (abs(A.psi)>A.pi)
    A.psi=A.psi-sign(A.psi)*A.pi*2;
end
% Sistema para realizar los graficos
%A.Z_plot(A.counter) = A.Z;
%A.Z_ref_plot(A.counter) = A.Z_des;
%A.Z_dis_plot(A.counter) = A.Z_dis;

%A.X_plot(A.counter) = A.X;
%A.X_ref_plot(A.counter) = A.X_des;
%A.X_dis_plot(A.counter) = A.X_dis;

%A.Y_plot(A.counter) = A.Y;
%A.Y_ref_plot(A.counter) = A.Y_des;
%A.Y_dis_plot(A.counter) = A.Y_dis;

%A.phi_plot(A.counter) = A.phi;
%A.phi_ref_plot(A.counter) = A.phi_des;
%A.phi_dis_plot(A.counter) = A.phi_dis;

%A.theta_plot(A.counter) = A.theta;
%A.theta_ref_plot(A.counter) = A.theta_des;
%A.theta_dis_plot(A.counter) = A.theta_dis;

%A.psi_plot(A.counter) = A.psi;
%A.psi_ref_plot(A.counter) = A.psi_des;
%A.psi_dis_plot(A.counter) = A.psi_dis;

A.counter = A.counter + 1;

%Sistema que remueve las perturbaciones se debe quitar si se desea estudiar
%las perturbaciones
%if(A.Z_dis ~= 0)
 %   A.Z_dis = 0;
%end

%if(A.psi_dis ~= 0)
 %   A.psi_dis = 0;
%end

%if(A.theta_dis ~= 0)
 %   A.theta_dis = 0;
%end

%if(A.phi_dis ~= 0)
 %   A.phi_dis = 0;
%end

end