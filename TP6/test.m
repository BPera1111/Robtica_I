% Definición del robot planar de 2RR utilizando la Robotics Toolbox de Peter Corke

% Limpieza del entorno
clc;
clear all;
close all;

% Definición de los parámetros de los eslabones (Denavit-Hartenberg)
L1 = 1; % Longitud del eslabón 1 en metros
L2 = 0.5; % Longitud del eslabón 2 en metros

% Definición de los eslabones utilizando parámetros DH estándar
% theta d a alpha
% En este caso, d y alpha son cero ya que es un robot planar

% Importar la biblioteca
% Si la biblioteca está en tu ruta de MATLAB, puedes omitir esta línea
% addpath('ruta/a/la/robotics-toolbox'); 

% Definición de los eslabones
% Utilizamos 'Revolute' para definir articulaciones rotacionales
Link1 = Revolute('d', 0, 'a', L1, 'alpha', 0);
Link2 = Revolute('d', 0, 'a', L2, 'alpha', 0);

% Creación del modelo del robot
robot = SerialLink([Link1, Link2], 'name', 'Robot Planar 2RR');

% Visualización del robot en la posición cero
q = [0, 0]; % Ángulos articulares iniciales
figure;
robot.plot(q);
title('Configuración Inicial del Robot');

% Cinemática Directa
% Definimos un conjunto de ángulos articulares
theta1 = deg2rad(30); % Convertir grados a radianes
theta2 = deg2rad(45);

q = [theta1, theta2];

% Calculamos la posición del efector final
T = robot.fkine(q); % Matriz de transformación homogénea
disp('Posición del efector final (Cinemática Directa):');
disp(T.t); % Mostrar la posición (x, y, z)

% Visualización del robot en la nueva configuración
figure;
robot.plot(q);
title('Configuración del Robot para θ1=30°, θ2=45°');

% Cálculo del Jacobiano
J = robot.jacob0(q); % Jacobiano en el marco base
disp('Jacobiano del Robot:');
disp(J);

% Verificación de Singularidades
% Calculamos el determinante del Jacobiano
det_J = det(J(1:2,1:2)); % Usamos solo las primeras dos filas y columnas para un robot planar
disp('Determinante del Jacobiano:');
disp(det_J);

if abs(det_J) < 1e-6
    disp('El robot está en una configuración singular.');
else
    disp('El robot no está en una configuración singular.');
end

% Exploración de Singularidades
% Variamos theta2 para encontrar una configuración singular
theta2_sing = 0; % Cuando los eslabones están alineados
q_sing = [theta1, theta2_sing];

% Calculamos el Jacobiano en la configuración singular
J_sing = robot.jacob0(q_sing);
det_J_sing = det(J_sing(1:2,1:2));

disp('Determinante del Jacobiano en la configuración singular:');
disp(det_J_sing);

% Visualización del robot en la configuración singular
figure;
robot.plot(q_sing);
title('Configuración Singular del Robot (Eslabones Alineados)');

% Gráfica del espacio de trabajo
% Definimos un rango de valores para theta1 y theta2
theta1_range = linspace(-pi, pi, 100);
theta2_range = linspace(-pi, pi, 100);

% Inicializamos matrices para almacenar las posiciones
[X, Y] = meshgrid(zeros(length(theta1_range), length(theta2_range)));

for i = 1:length(theta1_range)
    for j = 1:length(theta2_range)
        q_temp = [theta1_range(i), theta2_range(j)];
        T_temp = robot.fkine(q_temp);
        pos = T_temp.t;
        X(i,j) = pos(1);
        Y(i,j) = pos(2);
    end
end

% Gráfica del espacio de trabajo
figure;
plot(X(:), Y(:), '.');
xlabel('X (m)');
ylabel('Y (m)');
title('Espacio de Trabajo del Robot Planar 2RR');
axis equal;

