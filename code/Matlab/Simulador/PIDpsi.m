function PIDpsi
global A;
  persistent uik1;
  persistent errork1;
  % Inicializacion de los valores persistentes
  if A.init==0, 
      uik1    = 0; 
      errork1 = 0;
  end
  
%Ecuacion del error
  error = A.psi_des - A.psi;
  
  %Control Proporcional
  up = A.psi_KP * error;
  
  % Control Integral
  ui = uik1 + A.psi_KI * A.Ts/2 * (error + errork1);
  
  % Control Derivativo
  ud = A.psi_KD*A.r;
  
  
  % Implementacion del Control
  A.r_des = up + ui + ud;

  % Actualizacion de las variables
  uik1    = ui; 
  errork1 = error;