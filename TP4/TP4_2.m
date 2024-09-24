close all; clc; clear;
DH = [
 0.000 0.195  0.300 0.000 0;
 0.000 0.000  0.250 0.000 0;
 0.000 0.000  0.000 pi    1;
 0.000 0.000  0.000 0.000 0];
DH_nueva = [
 0.000, 0.262,  0.200, 0.000, 0;
 0.000, 0.000,  0.250, 0.000, 0;
 0.000, 0.000,  0.000, pi,    0;
 0.000, 0.000,  0.000, 0.000, 1];
 R = SerialLink(DH);
 q = [0,0,0,0];
 T = double(R.fkine(q));
 disp('Matriz de transformación homogénea con parámetros dados:')
 disp(T)
 R_n = SerialLink(DH_nueva);
 q_n = [0,0,0,0];
 T_nueva = double(R_n.fkine(q_n));
 disp('Matriz de transformación homogénea con parámetros obtenidos:')
 disp(T_nueva)