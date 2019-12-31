function [X,Y,Z]=rotateQUAD(X,Y,Z,q0,q1,q2,q3)
  % Define el quaternion de rotacion 
  q=[q0 q1 q2 q3];


  % Rota los vertices
  B=size(X);
  
  for i=1:B(2)*B(1)
  pts = [X(i), Y(i), Z(i)];
  rts=quatrotate(q,pts);
  
  X(i) = rts(:,1);
  Y(i) = rts(:,2);
  Z(i) = rts(:,3);
  end
end