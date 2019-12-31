function Forces
global A;
%Esta funcion calculara los torques y las fuerzas
% Dado la velocidad de cada Helice

% Inputs
% O1 = Frente(CCW)
% O2 = Derecha (CW)
% O3 = Retaguardia (CCW)
% O4 = Izquierda (CW)

% Outputs
% U1 = Sustentacion
% U2 = Roll
% U3 = Pitch 
% U4 = Yaw 

% Constantes
% l = Donde l [m] es la distancia entre el centro del cuadricoptero
% y el centro de las helices
% b = Constante de fuerza o sustentacion
% d= Constante de momento de arrastre

%A.U1 = A.b*(sign(A.O1)*A.O1^2 + sign(A.O2)*A.O2^2 + sign(A.O3)*A.O3^2 + sign(A.O4)*A.O4^2);


%A.U2 = A.b*A.l*(sign(A.O4)*A.O4^2 - sign(A.O2)*A.O2^2);

%A.U3 = A.b*A.l*(sign(A.O3)*A.O3^2 - sign(A.O1)*A.O1^2);
%A.U4 = A.d*(sign(A.O2)*A.O2^2 + sign(A.O4)*A.O4^2 - sign(A.O1)*A.O1^2 - sign(A.O3)*A.O3^2);

A.U1 = A.b*(A.O1^2 + A.O2^2 + A.O3^2 + A.O4^2);


A.U2 = A.b*A.l*(A.O4^2 + A.O3^2 - A.O2^2 - A.O1^2);

A.U3 = A.b*A.l*(A.O3^2 + A.O2^2 - A.O1^2 - A.O4^2);
A.U4 = A.d*(A.O2^2 + A.O4^2 - A.O1^2 - A.O3^2);


A.O = (-A.O1 + A.O2 - A.O3 + A.O4);


end