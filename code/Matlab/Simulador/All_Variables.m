%Basado en el software de Muhannad Al-Omari
%En esta seccion definimos todas las variavles
global A;

% Modelo 3D del cuadricoptero
copter_model_3D;

A.init=0;

% Parametros del Entorno
A.g = 9.8071;     % Gravedad
A.m = 1.48;      % Masa del Cuadricoptero
A.Ts = .001;     % Tiempo de Muestreo (50 Hz)
A.pi=3.14159265;
A.b = 11.08E-6;    % Coeficiente de Sustentacion
A.l = .23;     % Radio del Cuadricoptero
A.d = 1.284403E-6;    % Coeficiente de arrastre

A.Motors_limit = 750^2; % Limite Maximo del Motor +
A.Motors_lowerlimit =100^2; % Limite Minimo del Motor -
A.cont=0;

% Parametros PID

A.Z_KP =1;    % KP para control de altitud
A.Z_KI = 0;    % KI para control de altitud
A.Z_KD = 0.0015 ;  % KD para control de altitud
A.Vel_des=0;
A.X_KP = 0.05;          % KP para control de posicion X
A.X_KD = -0.2;         % KD para control de posicion X
A.X_KI=0;

A.Y_KP = 0.05;          % KP para control de posicion Y
A.Y_KD = -0.2;         % KD para control de posicion Y
A.Y_KI=0;


 A.theta_KP =3;   % KP para control del angulo roll
 A.theta_KI =0;    % KI para control del angulo roll
 A.theta_KD =-0.72;   % KD para control del angulo roll 

 A.phi_KP = 3;  % KP para control del angulo pitch
 A.phi_KI = 0;  % KI para control del angulo pitch
 A.phi_KD =    -0.72; % KD para control del angulo pitch


A.psi_KP = 0.85;     % KP para control del angulo yaw
A.psi_KI = 0.01;         % KI para control del angulo yaw
A.psi_KD = -0.3825;      % KD para control del angulo yaw


% Condiciones Iniciales
A.X = 0;        % Posicion Inicial segun el eje X Inercial
A.Y = 0;        % Posicion Inicial segun el eje Y Inercial
A.X_BF = 0;        % Posicion Inicial segun el eje X SV3
A.Y_BF = 0;        % Posicion Inicial segun el eje Y SV3
A.Z = 0;           % Posicion Inicial segun el eje Z Inercial

A.q0=1;    %Quaterniones de Rotacion 
A.q1=0;
A.q2=0;
A.q3=0;  


A.q0_des=1;    %Quaterniones de Rotacion 
A.q1_des=0;
A.q2_des=0;
A.q3_des=0;  

A.Ez=0;
A.Ex=0;
A.Ey=0;
A.Ez_KP=0;
A.Ez_KI=0;
A.Ez_KD=0;
A.Ex_KP=0.01;
A.Ex_KI=0;
A.Ex_KD=-0;
A.Ey_KP=0.01;
A.Ey_KI=0;
A.Ey_KD=-0;

A.X_dot = 0;    % Velocidad Inicial segun el eje X Inercial
A.Y_dot = 0;    % Velocidad Inicial segun el eje Y Inercial
A.X_dot_BF = 0;    % Velocidad Inicial segun el eje X SV3
A.Y_dot_BF = 0;    % Velocidad Inicial segun el eje Y SV3
A.Z_dot = 0;    % Velocidad Inicial segun el eje Z Inercial

A.p_des=0;
A.q_des=0;
A.r_des=0;
A.ex_dot=0;

A.ey_dot=0;

A.p = 0;        % Valor Inicial de p (rotacion angular en relacion a X segun el Sistema de Coordenadas SV3)
A.q = 0;        % Valor Inicial de q (rotacion angular en relacion a Y segun el Sistema de Coordenadas SV3)
A.r = 0;        % Valor Inicial de r (rotacion angular en relacion a Z segun el Sistema de Coordenadas SV3)

A.theta = 0;    % Valor Inicial del angulo theta (rotacion sobre el eje Y en SV2 de acuerdo a SV1)
A.phi = 0;      % Valor Inicial del angulo phi (rotacion sobre el eje X en SV3 de acuerdo a SV2)
  
A.psi = 0;      % Valor Inicial del angulo psi (rotacion sobre el eje Z en SV1 de acuerdo a SV)

A.Z_kalman = 0;   % Valor Inicial para Z de acuerdo al filtro KALMAN
A.X_kalman = 0;   % Valor Inicial para X de acuerdo al filtro KALMAN
A.Y_kalman = 0;   % Valor Inicial para Y de acuerdo al filtro KALMAN
A.phi_kalman = 0;   % Valor Inicial para phi de acuerdo al filtro KALMAN
A.theta_kalman = 0; % Valor Inicial para theta de acuerdo al filtro KALMAN
A.psi_kalman = 0;   % Valor Inicial para psi de acuerdo al filtro KALMAN



A.Ixx = 9.86E-3;     % Inercia de acuerdo al eje X 
A.Iyy = 9.86E-3;     % Inertia de acuerdo al eje Y 
A.Izz = 16.6446E-3;    % Inertia de acuerdo al eje Z
A.Jtp = 74.12E-6;     % Inercia de las helices por rotacion

A.U1 =A.m*A.g/(cos(A.theta)*cos(A.phi));       % Sustentacion
A.U2 = 0;       % Torque sobre el eje X de acuerdo a SV3
A.U3 = 0;       % Torque sobre el eje Y de acuerdo a SV3
A.U4 = 0;       % Torque sobre el eje Z de acuerdo a SV3
A.O = 0;        % Suma de todas las velocidades angulares de los motores

A.O1 =0;       % Velocidad del motor delantero

A.O2 = 0;       % Velocidad del motor derecho
A.O3 = 0;       % Velocidad del motor trasero
A.O4 = 0;       % Velocidad del motor izquierdo

% Desired variables
A.Z_des = 0;            % Valor deseado de Z
A.X_des_EF = 1;        
A.Y_des_EF = 1;       
A.X_des = 0;            % Valor deseado de X
A.Y_des = 0;            % Valor deseado de Y

A.phi_des = 0;          % Valor deseado de phi
A.theta_des = 0;        % Valor deseado de theta
A.psi_des = 0;          % Valor deseado de psi
A.r_des=0;
% plotting variables                % they will remain up to 10 min
A.Z_plot = zeros(1,60*10/A.Ts);      % the Z actual value
A.Z_ref_plot = zeros(1,60*10/A.Ts);  % the Z reference value
A.Z_dis_plot = zeros(1,60*10/A.Ts);  % the Z disturbance value
A.Z_kalman_plot = zeros(1,60*10/A.Ts);  % the Z kalman value

A.X_plot = zeros(1,60*10/A.Ts);      % the X actual value
A.X_ref_plot = zeros(1,60*10/A.Ts);  % the X reference value
A.X_dis_plot = zeros(1,60*10/A.Ts);  % the X disturbance value
A.X_kalman_plot = zeros(1,60*10/A.Ts);  % the X kalman value

A.Y_plot = zeros(1,60*10/A.Ts);      % the Y actual value
A.Y_ref_plot = zeros(1,60*10/A.Ts);  % the Y reference value
A.Y_dis_plot = zeros(1,60*10/A.Ts);  % the Y disturbance value
A.Y_kalman_plot = zeros(1,60*10/A.Ts);  % the Y kalman value

A.phi_plot = zeros(1,60*10/A.Ts);      % the phi actual value
A.phi_ref_plot = zeros(1,60*10/A.Ts);  % the phi reference value
A.phi_dis_plot = zeros(1,60*10/A.Ts);  % the phi disturbance value
A.phi_kalman_plot = zeros(1,60*10/A.Ts);  % the phi kalman value

A.theta_plot = zeros(1,60*10/A.Ts);      % the theta actual value
A.theta_ref_plot = zeros(1,60*10/A.Ts);  % the theta reference value
A.theta_dis_plot = zeros(1,60*10/A.Ts);  % the theta disturbance value
A.theta_kalman_plot = zeros(1,60*10/A.Ts);  % the theta kalman value

A.psi_plot = zeros(1,60*10/A.Ts);      % the phi actual value
A.psi_ref_plot = zeros(1,60*10/A.Ts);  % the phi reference value
A.psi_dis_plot = zeros(1,60*10/A.Ts);  % the phi disturbance value
A.psi_kalman_plot = zeros(1,60*10/A.Ts);  % the psi kalman value




A.U1_plot = zeros(1,60*10/A.Ts);      % the throttle plot
A.U2_plot = zeros(1,60*10/A.Ts);      % the Roll plot
A.U3_plot = zeros(1,60*10/A.Ts);      % the Pitch plot
A.U4_plot = zeros(1,60*10/A.Ts);      % the Yaw plot

A.O1_plot = zeros(1,60*10/A.Ts);      % the front motore plot
A.O2_plot = zeros(1,60*10/A.Ts);      % the right motore plot
A.O3_plot = zeros(1,60*10/A.Ts);      % the rear motore plot
A.O4_plot = zeros(1,60*10/A.Ts);      % the left motore plot
A.t_plot = 0:A.Ts:60*10-A.Ts;       % Matriz para determinar el valor del tiempo
A.counter = 1;                      % El contador que determina los puntos en la matriz de Tiempo

% Variables de disturbios iniciales
A.Z_dis = 0.5;            % Disturbio en Z
A.X_dis = 0.5;            % Disturbio en X
A.Y_dis = 0.5;            % Disturbio en Y

A.psi_dis = 0.5;            % Disturbio en Yaw
A.theta_dis = 0.5;            % Disturbio en el Pitch
A.phi_dis = 0.5;            % Disturbio en el Roll 

% Variables medidas
A.Z_meas = 0;
A.phi_meas = 0;
A.theta_meas = 0;
A.psi_meas = 0;

% Error de medicion en señal
% rng(1)
A.Z_dot_error=randn(1,60*10/A.Ts)*0.1;
A.Z_error = randn(1,60*10/A.Ts)*0.1;

% rng(2)
A.phi_error = randn(1,60*10/A.Ts)*0.05;

% rng(3)
A.theta_error = randn(1,60*10/A.Ts)*0.05;

A.M1_error=randn(1,60*10/A.Ts)*0.0;
A.M2_error=randn(1,60*10/A.Ts)*0.0;
A.M3_error=randn(1,60*10/A.Ts)*0.0;
A.M4_error=randn(1,60*10/A.Ts)*0.0;

% rng(4)
A.psi_error = randn(1,60*10/A.Ts)*0.05;
A.p_error=randn(1,60*10/A.Ts)*0.05;
A.q_error=randn(1,60*10/A.Ts)*0.05;
A.r_error=randn(1,60*10/A.Ts)*0.05;

% rng(5)
A.X_error = randn(1,60*10/A.Ts)*0;

% rng(6)
A.Y_error = randn(1,60*10/A.Ts)*0;

%Establece la inicializacion del simulado


% Bandera para poder realizar el plot en tiempo real
A.flag = 0;

% Senos y Cosenos

A.C_phi = cos(A.phi);
A.C_theta = cos(A.theta);
A.C_psi = cos(A.psi);

A.S_phi = sin(A.phi);
A.S_theta = sin(A.theta);
A.S_psi = sin(A.psi);

