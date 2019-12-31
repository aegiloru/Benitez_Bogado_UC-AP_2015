function quaternion_calc
global A;
A.q0=cos(A.phi_meas/2)*cos(A.theta_meas/2)*cos(A.psi_meas/2)+sin(A.phi_meas/2)*sin(A.theta_meas/2)*sin(A.psi_meas/2);
A.q1=sin(A.phi_meas/2)*cos(A.theta_meas/2)*cos(A.psi_meas/2)-cos(A.phi_meas/2)*sin(A.theta_meas/2)*sin(A.psi_meas/2);
A.q2=cos(A.phi_meas/2)*sin(A.theta_meas/2)*cos(A.psi_meas/2)+sin(A.phi_meas/2)*cos(A.theta_meas/2)*sin(A.psi_meas/2);
A.q3=cos(A.phi_meas/2)*cos(A.theta_meas/2)*sin(A.psi_meas/2)-sin(A.phi_meas/2)*sin(A.theta_meas/2)*cos(A.psi_meas/2);

%A.q0=cos(A.phi/2)*cos(A.theta/2)*cos(A.psi/2)+sin(A.phi/2)*sin(A.theta/2)*sin(A.psi/2);
%A.q1=sin(A.phi/2)*cos(A.theta/2)*cos(A.psi/2)-cos(A.phi/2)*sin(A.theta/2)*sin(A.psi/2);
%A.q2=cos(A.phi/2)*sin(A.theta/2)*cos(A.psi/2)+sin(A.phi/2)*cos(A.theta/2)*sin(A.psi/2);
%A.q3=cos(A.phi/2)*cos(A.theta/2)*sin(A.psi/2)-sin(A.phi/2)*sin(A.theta/2)*cos(A.psi/2);


end

