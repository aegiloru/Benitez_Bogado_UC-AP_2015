function PIDroll
global A;
  persistent uik1;
  persistent errork1;
  % Se inicializan los valores persistentes en la simulacion
  if A.init==0, 
      uik1    = 0; 
      errork1 = 0;
  end
 
  % Ecuacion del error
  error = A.phi_des - A.phi;
 % error = A.phi_des - A.phi;

  % Control Proporcional
  up = A.phi_KP * error;
  
  % Control Integral
  ui = uik1 + A.phi_KI * A.Ts/2 * (error + errork1);
  
  % Control Derivativo
  ud = A.phi_KD*A.p;
  
  
  % Implementacion del PID
  A.U2 = (up + ui + ud);

  % update stored variables
  uik1    = ui; 
  errork1 = error;