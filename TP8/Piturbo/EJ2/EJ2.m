clc
clear
close all

%Definimos el robot �FANUC Paint Mate 200iA� a partir de los resultados
%obtenidos en el TP4
dh = [
    0      0.45   0.075 -pi/2  0;
    0      0      0.3    0     0;
    0      0      0.075 -pi/2  0;
    0      0.32   0      pi/2  0;
    0      0      0     -pi/2  0;
    0      0.008  0      0     0];
R = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');
%Aplicamos cinem�tica directa, obtenemos una matriz de transformaci�n
%homog�nea y extraemos la submatriz de rotaci�n
qq = [0 -pi/2 -pi/4 0 pi/4 0];
T = R.fkine(qq).double()
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformaci�n homog�neas, una para la posici�n
%inicial y otra para la posici�n final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

q1 = R.ikine(T1, qq)
q2 = R.ikine(T2, qq)

%Con q1 y q2, interpolamos en el espacio articular utilizando la funci�n
%jtraj
m = 100; %Cantidad de puntos de discretizaci�n
q = jtraj(q1, q2, m);

%Finalmente realizamos una simulaci�n con el robot FANUC Paint Mate 200iA
figure();
R.plot(q);
%Graficamos la evoluci�n de las variables articulares
figure();
qplot(q);
xlabel('')