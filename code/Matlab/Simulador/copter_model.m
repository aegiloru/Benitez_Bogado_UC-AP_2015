global A
%Tamaño de los tres ejes del cuadricoptero
x1=-.27;
x2=.27;
y1=-.01;
y2=.01;
z1=-.01;
z2=.01;
%Vertice del Cuadricoptero 1er Palito
acon=[x1 y1 z1; 
   x2 y1 z1;
   x2 y2 z1;
   x1 y2 z1;
   x1 y1 z2;
   x2 y1 z2;
   x2 y2 z2;
   x1 y2 z2];

%Define la cantidad de caras y sus puntos
bcon=[1 2 6 5;              
    2 3 7 6;
    3 4 8 7;
    4 1 5 8;
    1 2 3 4;
    5 6 7 8];
%view(0,20);
grid on; 
 body1=patch('faces',bcon,...
     'vertices',acon,...
     'facecolor',[.8 .8 .8],...
     'edgecolor',[0 0 0],'facecolor',[.2 .3 .5]);
 % segundo Palito
 
y1=-.27;
y2=.27;
x1=-.01;
x2=.01;
z1=-.01;
z2=.01;

acon=[x1 y1 z1; 
   x2 y1 z1;
   x2 y2 z1;
   x1 y2 z1;
   x1 y1 z2;
   x2 y1 z2;
   x2 y2 z2;
   x1 y2 z2];

 body2=patch('faces',bcon,...
     'vertices',acon,...
     'facecolor',[.8 .8 .8],...
     'edgecolor',[0 0 0],'facecolor',[.2 .3 .5]);
 %Dibujo de los rotores Ecuacion de una cia trasladando los ejes X e Y
 t=0:pi/15:2*pi;
 X=.13*cos(t);
 Y=.13*sin(t);
 Z=zeros(size(t))+.014;
 C=zeros(size(t));
 A.C = C;

 RotorF = patch(X+.27,Y,Z,C,'facealpha',.3,'facecolor','g');
 RotorB = patch(X-.27,Y,Z,C,'facealpha',.3,'facecolor','g');
 RotorL = patch(X,Y+.27,Z,C,'facealpha',.3,'facecolor','g');
 RotorR = patch(X,Y-.27,Z,C,'facealpha',.3,'facecolor','g');
 % Tomando los datos en una variable global
 A.Body1X = get(body1,'xdata');
 A.Body1Y = get(body1,'ydata');
 A.Body1Z = get(body1,'zdata');
 
 A.Body2X = get(body2,'xdata');
 A.Body2Y = get(body2,'ydata');
 A.Body2Z = get(body2,'zdata');
  
 A.RotorFX = get(RotorF,'xdata');
 A.RotorFY = get(RotorF,'ydata');
 A.RotorFZ = get(RotorF,'zdata');
  
 A.RotorBX = get(RotorB,'xdata');
 A.RotorBY = get(RotorB,'ydata');
 A.RotorBZ = get(RotorB,'zdata');
  
 A.RotorRX = get(RotorR,'xdata');
 A.RotorRY = get(RotorR,'ydata');
 A.RotorRZ = get(RotorR,'zdata');
  
 A.RotorLX = get(RotorL,'xdata');
 A.RotorLY = get(RotorL,'ydata');
 A.RotorLZ = get(RotorL,'zdata');
 A.phi=0;
 A.theta=0;
 A.psi=0.25*A.pi;
 [A.Body1X,A.Body1Y,A.Body1Z]=rotateXYZ2(A.Body1X,A.Body1Y,A.Body1Z,A.phi,A.theta,A.psi); 
 [A.Body2X,A.Body2Y,A.Body2Z]=rotateXYZ2(A.Body2X,A.Body2Y,A.Body2Z,A.phi,A.theta,A.psi);
 [A.RotorFX,A.RotorFY,A.RotorFZ]=rotateXYZ2(A.RotorFX,A.RotorFY,A.RotorFZ,A.phi,A.theta,A.psi);
 [A.RotorBX,A.RotorBY,A.RotorBZ]=rotateXYZ2(A.RotorBX,A.RotorBY,A.RotorBZ,A.phi,A.theta,A.psi);
 [A.RotorRX,A.RotorRY,A.RotorRZ]=rotateXYZ2(A.RotorRX,A.RotorRY,A.RotorRZ,A.phi,A.theta,A.psi);
 [A.RotorLX,A.RotorLY,A.RotorLZ]=rotateXYZ2(A.RotorLX,A.RotorLY,A.RotorLZ,A.phi,A.theta,A.psi);


 xlabel('x')
 grid on
 axis([-1 1 -1 1 -1 1]);
 axis equal
 save Cuadricoptero.mat
  
