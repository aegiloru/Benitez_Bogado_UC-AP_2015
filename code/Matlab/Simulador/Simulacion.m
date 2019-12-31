clc
clear all
global A;
start_time = 0;
end_time = 300;
dt = 0.01;
times = start_time:dt:end_time;
figure('units','normalized','position',[.1 .1 .8 .8],'name','Cuadricoptero','numbertitle','off','color','w');
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')
grid on
grid minor
axis([-1 1 -1 1 0 1]);

view([30,30]);
All_Variables;
A.Z=0;
SP=0;
A.Z_des=0;
A.X_des_EF=0;
A.Y_des_EF=0;
Z_plot=zeros(1,30001);
X_plot=zeros(1,30001);
Y_plot=zeros(1,30001);
SET_POINT=zeros(1,30001);

A.phi_plot=zeros(1,30001);
A.psi_plot=zeros(1,30001);
A.theta_plot=zeros(1,30001);
A.time=zeros(1,3001);
A.Z_plot=zeros(1,30001);
A.X_plot=zeros(1,30001);
A.O1_plot=zeros(1,30001);
A.U1_plot=zeros(1,30001);
A.U2_plot=zeros(1,30001);

A.U3_plot=zeros(1,30001);
A.U4_plot=zeros(1,30001);


%camva([30,30])
%campos([0 0 0])
%camtarget([0 0 5])

% Number of points in the simulation.
N = numel(times);
Vel_X=0;
Vel_Y=0;
cont=0;
p_cont=0;

for t = times
    
    if (t==33)
        %A.q_des=0.5;
        %A.p_des=0.2;
        %A.Z_des=2.5;
        SP=20*0;
        A.theta_des=20*A.pi/180*0;
        A.phi_des=20*A.pi/180*0;
        A.psi_des=20*A.pi/180*0;
        A.Z_des=2*0;
        
        %      A.theta=15*A.pi/180;
        %A.U2_d=0.15;
        %A.U3_d=0.15;
        %   A.U1=16;
        %     A.phi=15*A.pi/180;
        %     A.psi=15*A.pi/180;
        %       A.X_des_EF=0;
        %       A.Y_des_EF=0;
        % A.Z_des=0;
        %  A.q=0;
        %  A.p=0;
        %  A.r=0;
        % A.phi_des=0.9;
    end
    if (t==66)
        SP=0;
        %A.q_des=0.5;
        %A.p_des=0.2;
        %A.Z_des=2.5;
        A.theta_des=0;
        A.phi_des=0;
        A.psi_des=0;
        A.Z_des=0;
        % A.X_des_EF=0;
        %A.Y_des_EF=0;
        %A.U2_d=0.15;
        %A.U3_d=0.15;
        %   A.U1=16;
        
        % A.Z_des=3;
        %  A.X=5;
        %  A.Y=3;
        %  A.Z=5;
        %  A.q=0;
        %  A.p=0;
        %  A.r=0;
        % A.phi_des=0.9;
    end
    
    
    %   if (t==50)
    %A.q_des=0.5;
    %A.p_des=0.2;
    %A.Z_des=2.5;
    %    A.theta_des=0;
    %A.U2_d=0.15;
    %A.U3_d=0.15;
    %   A.phi_des=0;
    % A.Z=5;
    %A.U1=13;
    %  A.q=0;
    %  A.p=0;
    %  A.r=0;
    % A.phi_des=0.9;
    %  end
    
    
    
    Z_meas;
    IMU;
    %Kalman_phi;
    %Kalman_psi;
    %Kalman_theta;
    %Kalman_Z;
    %PID_X;
    %PID_Y;
    %A.U1=11.75;
    %Se inicializan los controles PID
    %PID_Ex;
    %PID_Ey;
    %PID_Ez;
    %PIDpitch;
    %PIDroll;
    %PIDpsi;
    A.cont=A.cont+1;
    quaternion_calc;
    %A.psi_des=A.psi;
    quaterniondes_calc;
    ln_quaternion;
   % if (abs(A.phi)<A.pi/3 && abs(A.theta)<A.pi/3 )
        PID_Altura;
   % end
    %PIDpitch;
    %PIDroll;
    %PIDpsi;
    
    %PID_X;
    %PID_Y;
    
    
    %e=2.71828;
    %A.p_des=sign(A.Ex)*0.5*(1-e^(-abs(A.Ex)));
    %A.q_des=sign(A.Ey)*0.5*(1-e^(-abs(A.Ey)));
    %A.r_des=sign(A.Ez)*0.3*(1-e^(-abs(A.Ez)));
    Motors_Speed;    %Se actualizan los valores de la simulacion a traves de sus respectivas ecuaciones
    Forces;
    quadmodel;
    
    
    %plot_quad_3Ddin;
    
    
    %drawnow
    %Vel_Y=(A.Y_ddot+200*A.Y_error(A.counter))*A.Ts+Vel_Y;
    %Vel_X=(A.X_ddot+200*A.X_error(A.counter))*A.Ts+Vel_X;
    %A.theta_des=-(0.1876/12)*Vel_Y-0.15*(A.Y_ddot+200*A.Y_error(A.counter));
    %A.phi_des=-(0.1876/12)*Vel_X-0.15*(A.X_ddot+200*A.X_error(A.counter));
    
    
    %sl_control;
    A.Ts=dt;
    
    A.init=1;
    if (A.counter<30002)
        SET_POINT(A.counter)=SP;
        Z_plot(A.counter)=A.Z;
        X_plot(A.counter)=A.X;
        Y_plot(A.counter)=A.Y;
        A.phi_plot(A.counter)=A.phi*(180/A.pi);
        A.theta_plot(A.counter)=A.theta*(180/A.pi);
        A.psi_plot(A.counter)=A.psi*(180/A.pi);
        A.U1_plot(A.counter)=A.U1;
        A.O1_plot(A.counter)=A.O1;
        A.U2_plot(A.counter)=A.U2;
        %  A.U3_plot(A.counter)=A.U3;
        A.U4_plot(A.counter)=A.U4;
        A.time(A.counter)=t;
        
    end
end
 plot(times,Z_plot,'k','LineWidth',2.5)
%plot(times,A.phi_plot,'k',times,A.theta_plot,'r',times,A.psi_plot,'g','LineWidth',2.5)
hold on
plot(times,SET_POINT,'b','LineWidth',1.5)
%title('Step response of the Yaw, Pitch and Roll angles without noise')
xlabel('tiempo(s)','FontSize',16,'FontWeight','bold','FontName','Times New Roman') % x-axis label
%ylabel('Ángulos (grados)','FontSize',16,'FontWeight','bold','FontName','Times New Roman')% y-axis label
ylabel('Altura (m)','FontSize',16,'FontWeight','bold','FontName','Times New Roman')% y-axis label
%legend({'phi(Roll)','theta(Pitch)','psi(Yaw)','SP=0°'},'FontSize',16,'FontWeight','bold','FontName','Times New Roman')
%axis([-0.1,10,-0.05,0.5])
legend({'Z','SP=0 m'},'FontSize',16,'FontWeight','bold','FontName','Times New Roman')
grid on
axis([100 170 -0.5 0.5]);
%csvwrite('time',times);
%csvwrite('phi_plot_withnoise.dat',A.phi_plot);
%csvwrite('Z_plot_withnoise.dat',Z_plot);
%csvwrite('theta_plot_withnoise.dat',A.theta_plot);
%csvwrite('psi_plot_withnoise.dat',A.psi_plot);

