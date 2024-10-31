clc; clear; close all;

% Cargar los robots guardados en kuka_16.mat y kuka_16_2.mat
load('kuka_16.mat', 'R', 'path', 'workspace', 'q_kuka_16');
load('kuka_16_2.mat', 'R2', 'path_2', 'workspace_2', 'q_kuka_16_2');
R2.name='Kuka 16 2';
% Crear la figura y definir el workspace común
figure();
W = [-3 3 -3 3 -3 3];

hold on


% Graficar el primer robot usando plot3d
R.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1, 'workspace', W);

% Graficar el segundo robot también usando plot3d
R2.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1, 'workspace', W);

hold off;
