% Ejercicio 1 - TP5B - Cinematica Inversa - Corazza - Masi - Suarez.
close all; clear all; clc;
%
d1=1;
a2=1;
a3=1;
dh = [0 d1 0  pi/2 0;
      0 0  a2 0    0;
      0 0  a3 0    0];
R = SerialLink(dh,'name','Ejercicio 1');
q = [0,0,0]; % Vector inicial de valores articulares. Correspondiete a inciso 7.
R.qlim(1,1:2) = [-180,180]*pi/180; 
R.qlim(2,1:2) = [-180,180]*pi/180; 
R.qlim(3,1:2) = [-180,180]*pi/180;
%
%---Inciso 7---
%
q1 = [45,-45,90]*pi/180; % Configuraci�n articular propuesta.
T = Transf_Sistemas_Total(dh,q1);
xc = T(1,4); 
yc = T(2,4); 
zc = T(3,4);
%
pc0 = [xc;yc;zc;1]; % Vector homogeno de posici�n de C con respecto a {0}.
%
flag=0; % Flag para saber si se grafica o no.
%
qsol=zeros(4,3); % Vector articular, hay cuatro soluciones posibles.
%
%---C�lculo de tita1---
%
qsol(1,1) = atan2(yc,xc);
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
pc1 = zeros(4,2);
for i=1:1:2
    T1 = Transf_Sistemas(dh(1,:),qsol(2*i,1));
    pc1(:,i) = T1*pc0;
    r = sqrt(pc1(1,i)^2+pc1(2,i)^2);
    beta_2 = atan2(pc1(2,i),pc1(1,i));
    alfa_2 = acos((a3^2-a2^2-r^2)/(-2*a2*r));
    qsol(2*i-1,2) = beta_2 - alfa_2;
    qsol(2*i,2) = beta_2 + alfa_2;
end
pc1 = [pc1(:,1) pc1(:,1) pc1(:,2) pc1(:,2)];
%
%---C�lculo de tita3---
%
for i=1:1:4
    T12 = Transf_Sistemas(dh(2,:),qsol(i,2));
    pc2 = inv(T12)*pc1(:,i);
    qsol(i,3) = atan2(pc2(2),pc2(1));
end
%
%---Ploteo---
%
qs = qsol(1,:); % Configuraci�n de soluci�n.
for i=1:1:3
    if (qs(i)>pi || qs(i)<-pi)
        flag = 1;
    end
end
if flag==0
    figure(1)
    R.plot(qsol(1,:),'workspace',[-2 2 -2 2 -2 2],'scale',0.5,'jointdiam',0.5) % Ploteo del modelo.
    hold on
    plot3(xc,yc,zc,'*k','Color','b') % Ploteo del punto.
    grid on
else 
    disp('Soluci�n fuera de rango articular.')
    disp(qs*180/pi);
end
disp('Todas las soluciones:')
disp(qsol*180/pi);
%
%---Inciso 7---
%
