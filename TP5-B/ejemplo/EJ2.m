% Ejercicio 2 - TP5B - Cinematica Inversa - Corazza - Masi - Suarez.
close all; clear all; clc;
%
d1=1;
a2=1;
d4=1;
d6=1;
dh = [0 d1 0  -pi/2  0;
      0 0  a2 0      0;
      0 0  0  -pi/2  0;
      0 d4 0  pi/2   0;
      0 0  0  pi/2  0;
      0 d6 0  0      0];
R = SerialLink(dh,'name','Ejercicio 2');
q =[35,-70,-35,35,-110,35]*pi/180;
T = R.fkine(q).double; % Matriz T dato.
R.qlim(1,1:2) = [-180,180]*pi/180; 
R.qlim(2,1:2) = [-180,180]*pi/180; 
R.qlim(3,1:2) = [-180,180]*pi/180;
R.qlim(4,1:2) = [-360,360]*pi/180; 
R.qlim(5,1:2) = [-360,360]*pi/180;  
R.qlim(6,1:2) = [-360,360]*pi/180; 
%
%---C�lculo del centro de la mu�eca---
%
p = T(1:3,4) - dh(6,2) * T(1:3,3);
%
qsol=zeros(8,6); % Vector articular, hay cuatro soluciones posibles.
%
%---C�lculo de tita1---
%
qsol(1,1) = atan2(p(2),p(1));
qsol(2,1) = qsol(1,1);
if qsol(1,1) > 0
    qsol(3,1) = qsol(1,1) - pi;
    qsol(4,1) = qsol(3,1);
end
if qsol(1,1) <= 0
    qsol(3,1) = qsol(1,1) + pi;
    qsol(4,1) = qsol(3,1);
end
%
%---C�lculo de tita2---
%
for i=1:1:2
    T1 = R.links(1).A(qsol(2*i,1)).double;
    pc1 = invHomog(T1) * [p;1];
    r = sqrt(pc1(1)^2+pc1(2)^2);
    beta_2 = atan2(pc1(2),pc1(1));
    L2 = R.links(2).a;
    L3 = R.links(4).d;
    alfa_2 = acos((-L2^2+L3^2-r^2)/(-2*L2*r));
    qsol(2*i-1,2) = beta_2 - alfa_2;
    qsol(2*i,2) = beta_2 + alfa_2;
end
%
%---C�lculo de tita3---
%
for i=1:1:4
    T1 = R.links(1).A(qsol(i,1)).double;
    pc1 = invHomog(T1) * [p;1];
    T2 = R.links(2).A(qsol(i,2)).double;
    pc2 = invHomog(T2) * pc1;
    qsol(i,3) = atan2(pc2(2),pc2(1))- pi/2;
end
%
%---Problema de la orientaci�n---
%
qsol(5:8,1:3) = qsol(1:4,1:3);
%
%---C�lculo de tita4---
%
for i=1:1:4
    T1 = R.links(1).A(qsol(i,1)).double;
    T2 = R.links(2).A(qsol(i,2)).double;
	T3 = R.links(3).A(qsol(i,3)).double;
    T36 = invHomog(T3) * invHomog(T2) * invHomog(T1) * T;
    qsol(i,4) = atan2(T36(2,3),T36(1,3));
    if qsol(i,4) > 0
        qsol(i+4,4) = qsol(i,4) - pi;
    else
        qsol(i+4,4) = qsol(i,4) + pi;
    end
end
%
%---C�lculo de tita5---
%
for i=1:1:8
    T1 = R.links(1).A(qsol(i,1)).double;
    T2 = R.links(2).A(qsol(i,2)).double;
	T3 = R.links(3).A(qsol(i,3)).double;
    T4 = R.links(4).A(qsol(i,4)).double;
    T46 = invHomog(T4) * invHomog(T3) * invHomog(T2) * invHomog(T1) * T;
    qsol(i,5) = atan2(T46(2,3), T46(1,3)) + pi/2;
    T5 = R.links(5).A(qsol(i,5)).double;
    T56 = invHomog(T5) * T46;
    qsol(i,6) = atan2(T56(2,1), T56(1,1));
end
%
figure(1)
R.plot(qsol(8,:),'workspace',[-4 4 -4 4 -4 4],'scale',0.5,'jointdiam',0.5) % Ploteo del modelo.
hold on
plot3(p(1),p(2),p(3),'*k','Color','r') % Punto de la mu�eca.
plot3(T(1,4),T(2,4),T(3,4),'*k','Color','b') % Ploteo del punto.
grid on
