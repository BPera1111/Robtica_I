
clc; clear; close all;


% Cargar el robot guardado en kuka_16.mat

R=RobotCI();
path = fullfile(pwd,'..','STL','KR16_2');


%Parámetros de la circunferencia
radio = 0.2; % Radio de la circunferencia
centro_y = 1.5; % Centro de la circunferencia en el eje Y
centro_z = 1; % Centro de la circunferencia en el eje Z
num_puntos = 100; % Número de puntos en la circunferencia

% Generar puntos de la circunferencia en el plano YZ
theta = linspace(pi, pi*3/2, num_puntos); % Ángulos de la circunferencia
trayectoria_y = centro_y + radio * cos(theta); % Coordenada Y
trayectoria_z = centro_z + radio * sin(theta); % Coordenada Z
trayectoria_x = zeros(1, num_puntos); % Coordenada X fija (plano YZ)


% Graficar la trayectoria en el plano YZ
figure();
plot3(trayectoria_x, trayectoria_y, trayectoria_z, 'b', 'LineWidth', 2); 
xlabel('Y'); ylabel('Z');
title('Trayectoria circular en el plano YZ');
axis ([-2 2 -1 4 -1 3]);


T={};
T{1} = transl(0,1,1);

for i=1:num_puntos
    T{i+1} = transl(trayectoria_x(i),trayectoria_y(i),trayectoria_z(i)); % Posición inicial
end

q=[180 0 0 0 0 0]*pi/180; % Posición inicial de las articulaciones

T_trayectoria = zeros(4,4,1);

for i=1:num_puntos
    T_trayectoria = concatenar_matriz(T_trayectoria, ctraj(T{i}, T{i+1}, 3));

end

q_trayectoria = []; % Trayectoria completa de articulaciones

for i=1:num_puntos*3
    [q_i qq]=TP5B_EjercicioTF(T_trayectoria(:,:,i), R, q, 1);
    q_i = q_i';
    q_trayectoria = [q_trayectoria; q_i];
end

% Mover el robot a la posición calculada

R.plot3d(q_trayectoria, 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);


% Graficar el segundo robot también usando plot3d
%R2.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);


function [T_traj] = concatenar_matriz(T_traj1,T_traj2)
    T_traj = zeros(4,4,size(T_traj1,3)+size(T_traj2,3));
    T_traj(:,:,1:size(T_traj1,3)) = T_traj1;
    T_traj(:,:,size(T_traj1,3)+1:end) = T_traj2;
end
