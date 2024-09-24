close all; clc; clear;
% Parametros DH LBR  iiwa  7  R800  (KUKA)
DH = [
  0 0.34  0 pi/2  0;
  0 0     0 pi/2  0;
  0 0.4   0 pi/2  0;
  0 0     0 pi/2  0;
  0 0.4   0 pi/2  0;
  0 0     0 pi/2     0;
  0 0.126 0 0     0];
q =[0,0,0,0,0,0,0];
erro_art = (0.1 * pi/180) * ones(1,7);
q_error = q + erro_art;
R = SerialLink(DH);
R1.qlim(1,1:2) = [-360, 360]*pi/180;
R1.qlim(2,1:2) = [-360, 360]*pi/180;
R1.qlim(3,1:2) = [-360, 360]*pi/180;
R1.qlim(4,1:2) = [-360, 360]*pi/180;
R1.qlim(5,1:2) = [-360, 360]*pi/180;
R1.qlim(6,1:2) = [-360, 360]*pi/180;
R1.qlim(7,1:2) = [-360, 360]*pi/180;

T = double(R.fkine(q));
T_error = double(R.fkine(q_error));
disp('Matriz de transformación homogénea con parámetros dados:')
disp(T)
disp('Matriz de transformación homogénea con parámetros dados + error:')
disp(T_error)

A = (T_error - T);
error_abs = sum(abs(A(1:3,4)));
disp('Error absoluto:')
disp(error_abs)




