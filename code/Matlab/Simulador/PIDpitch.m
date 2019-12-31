function PIDpitch
global A;
  persistent uik1;
  persistent errork1;
  % Se inicializan los valores persistentes de la simulacion
  if A.init==0, 
      uik1    = 0; 
      errork1 = 0;
  end
 
  % Ecuacion del Error

  error = A.theta_des - A.theta;

  % Control Proporcional
  up = A.theta_KP * error;
  
  % Control Integral
  ui = uik1 + A.theta_KI * A.Ts/2 * (error + errork1);
  
  % Control Derivativo
  ud = A.theta_KD*A.q;
  
  
  % Implementacion del PID
  A.U3 = up + ui + ud ;

  % Actualizacion de las variables
  uik1    = ui; 
  errork1 = error;
  
end