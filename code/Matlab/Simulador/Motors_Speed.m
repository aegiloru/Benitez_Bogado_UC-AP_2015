function Motors_Speed
global A;
%A.U1=A.U1+6.1323038;
%A.U2=A.U2-0.433771154;
%A.U3=A.U3+0.433771154;
%A.U4=A.U4-0.052;
%Rot_mat=[A.b A.b A.b A.b; -A.b*A.l -A.b*A.l A.b*A.l A.b*A.l;-A.b*A.l  A.b*A.l A.b*A.l -A.b*A.l; -A.d   A.d -A.d   A.d]
%BB_mat=inv(Rot_mat)*[A.U1 A.U2 A.U3 A.U4]'
%BB1=BB_mat(1,1);
%BB2=BB_mat(2,1);
%BB3=BB_mat(3,1);
%BB4=BB_mat(4,1);


%BB1 = A.U1/(4*A.b) - A.U3/(2*A.b*A.l) - A.U4/(4*A.d); %Ecuacion de velocidad de los motores
%BB2 = A.U1/(4*A.b) - A.U2/(2*A.b*A.l) + A.U4/(4*A.d);
%BB3 = A.U1/(4*A.b) + A.U3/(2*A.b*A.l) - A.U4/(4*A.d);
%BB4 = A.U1/(4*A.b) + A.U2/(2*A.b*A.l) + A.U4/(4*A.d);
BB1 = A.U1/(4*A.b) - A.U2/(4*A.b*A.l) - A.U3/(4*A.b*A.l) - A.U4/(4*A.d);
BB2 = A.U1/(4*A.b) - A.U2/(4*A.b*A.l) + A.U3/(4*A.b*A.l) + A.U4/(4*A.d);
BB3 = A.U1/(4*A.b) + A.U2/(4*A.b*A.l) + A.U3/(4*A.b*A.l) - A.U4/(4*A.d);
BB4 = A.U1/(4*A.b) + A.U2/(4*A.b*A.l) - A.U3/(4*A.b*A.l) + A.U4/(4*A.d);

if abs(BB1)>A.Motors_limit
    BB1 = sign(BB1)*A.Motors_limit;
end



if abs(BB2)>A.Motors_limit
    BB2 = sign(BB2)*A.Motors_limit;
end

if abs(BB3)>A.Motors_limit
    BB3 = sign(BB3)*A.Motors_limit;
end

if abs(BB4)>A.Motors_limit
    BB4 = sign(BB4)*A.Motors_limit;
end
if (BB1<100^2)
    BB1=100^2;
end
if (BB2<100^2)
    BB2=100^2;
end
if (BB3<100^2)
    BB3=100^2;
end
if (BB4<100^2)
    BB4=100^2;
end
if (A.cont==1)
    A.cont=0;
A.O1 = sign(BB1)*sqrt(abs(BB1))+ A.M1_error(A.counter);    % Frente M
A.O2 = sign(BB2)*sqrt(abs(BB2))+ A.M2_error(A.counter);    % Derecha M
A.O3 = sign(BB3)*sqrt(abs(BB3))+ A.M3_error(A.counter);    % Atras M
A.O4 = sign(BB4)*sqrt(abs(BB4))+ A.M4_error(A.counter);    % Izquierda M
end

end