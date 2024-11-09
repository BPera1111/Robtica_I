clc; clear; close all;

% Definir puntos de trayectoria en el espacio cartesiano con orientación (X, Y, Z, roll, pitch, yaw)
% Las orientaciones están en radianes.
Q = [
    0, 0, 0, 0, 0, 0;           % Punto inicial
    0.5, 0.5, 0.5, pi/4, 0, 0;  % Punto intermedio con rotación en roll
    1.0, 0, 0.5, pi/4, pi/6, pi/3 % Punto final con roll, pitch, yaw
];

% Parámetros de la trayectoria
qdmax = [0.1, 0.1, 0.1, 0.05, 0.05, 0.05];  % Velocidad máxima por dimensión
dt = 0.1;                                   % Intervalo de tiempo de muestreo
tacc = 0.05;                                % Tiempo de aceleración
q0 = Q(1, :);                               % Punto de inicio de la trayectoria

% Generar la trayectoria con mstraj
traj_xyzrpy = mstraj(Q, qdmax, [], q0, dt, tacc);

% Mostrar la trayectoria generada
disp('Trayectoria generada (X, Y, Z, roll, pitch, yaw):');
disp(traj_xyzrpy);

% Graficar la trayectoria (en términos de X, Y, Z)
figure;
plot3(traj_xyzrpy(:, 1), traj_xyzrpy(:, 2), traj_xyzrpy(:, 3), '-o');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trayectoria en el espacio cartesiano con mstraj');
grid on;
