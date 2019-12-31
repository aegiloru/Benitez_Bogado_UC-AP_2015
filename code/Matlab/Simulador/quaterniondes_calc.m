function quaterniondes_calc
global A;
persistent psi_des;
if (A.init==0)
    psi_des=0;
end
psi_des=A.psi_meas;
A.q0_des=cos(A.phi_des/2)*cos(A.theta_des/2)*cos(psi_des/2)+sin(A.phi_des/2)*sin(A.theta_des/2)*sin(psi_des/2);
A.q1_des=sin(A.phi_des/2)*cos(A.theta_des/2)*cos(psi_des/2)-cos(A.phi_des/2)*sin(A.theta_des/2)*sin(psi_des/2);
A.q2_des=cos(A.phi_des/2)*sin(A.theta_des/2)*cos(psi_des/2)+sin(A.phi_des/2)*cos(A.theta_des/2)*sin(psi_des/2);
A.q3_des=cos(A.phi_des/2)*cos(A.theta_des/2)*sin(psi_des/2)-sin(A.phi_des/2)*sin(A.theta_des/2)*cos(psi_des/2);

    
end

