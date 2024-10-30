clc
clear
close all

%Inciso 1
q0 = [0 -pi/2 0 0 0 0];
qf = [-pi/3 pi/10 -pi/5 pi/2 pi/4 0];
t = 0:0.1:3;

%Vector de q interpolados, cada columna corresponde a una variable articular
q = jtraj(q0,qf,t) 

%Inciso 2
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
%Animaci�n del robot
figure()
R.plot(q)

%Inciso 3
%Graficamos los q en funci�n del tiempo
figure()
qplot(t, q)

