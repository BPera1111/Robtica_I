<<<<<<< HEAD
clc;
clear;
close all;
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
T = R.fkine(qq).double();
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformaci�n homog�neas, una para la posici�n
%inicial y otra para la posici�n final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

%Interpolamos en el espacio cartesiano utilizando la funci�n ctraj y 100
%puntos de discretizaci�n. Obtenemos un conjunto de matrices de
%transformaci�n para cada punto intermedio de la trayectoria
m=100;
tc = ctraj(T1, T2, m)
%Aplicamos cinem�tica inversa y hallamos los valores de las variables
%articulares para cada matriz de transformaci�n obtenida.
q = R.ikine(tc, qq);
%Realizamos una animaci�n cinem�tica del robot y graficamos la evoluci�n
%temporal de las variables articulares
figure()
R.plot(q)

figure()
qplot(q)
=======
clc;
clear;
close all;
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
T = R.fkine(qq).double();
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformaci�n homog�neas, una para la posici�n
%inicial y otra para la posici�n final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

%Interpolamos en el espacio cartesiano utilizando la funci�n ctraj y 100
%puntos de discretizaci�n. Obtenemos un conjunto de matrices de
%transformaci�n para cada punto intermedio de la trayectoria
m=100;
tc = ctraj(T1, T2, m)
%Aplicamos cinem�tica inversa y hallamos los valores de las variables
%articulares para cada matriz de transformaci�n obtenida.
q = R.ikine(tc, qq);
%Realizamos una animaci�n cinem�tica del robot y graficamos la evoluci�n
%temporal de las variables articulares
figure()
R.plot(q)

figure()
qplot(q)
>>>>>>> 5d5cfbc (por favor funciona)
xlabel('')