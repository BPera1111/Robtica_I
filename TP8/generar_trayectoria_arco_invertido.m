
clc; clear; close all;

% Cargar el robot guardado en kuka_16.mat

R=RobotCI();
path = fullfile(pwd,'STL','KR16_2');

%Parámetros de la circunferencia
radio = 0.1; % Radio de la circunferencia
centro_y = 1.25; % Centro de la circunferencia en el eje Y
centro_z = 1; % Centro de la circunferencia en el eje Z
num_puntos = 30; % Número de puntos en la circunferencia

% Generar puntos de la circunferencia en el plano YZ
theta = linspace(0, 2*pi, num_puntos); % Ángulos de la circunferencia
trayectoria_y = centro_y + radio * cos(theta); % Coordenada Y
trayectoria_z = centro_z + radio * sin(theta); % Coordenada Z
trayectoria_x = zeros(1, num_puntos); % Coordenada X fija (plano YZ)

% Graficar la trayectoria en el plano YZ
figure();
plot3(trayectoria_x, trayectoria_y, trayectoria_z, 'b', 'LineWidth', 2); 
xlabel('Y'); ylabel('Z');
title('Trayectoria circular en el plano YZ');
axis ([-2 2 -1 4 -1 3]);

T_rot=eye(3); % Matriz de rotación

T={};

for i=1:num_puntos
    T{i} = [T_rot [trayectoria_x(i); trayectoria_y(i); trayectoria_z(i)]; 0 0 0 1]; % Posición inicial
end

q1=[0 0 0 0 pi/2 0];
q2=[0 0 0 0 0 0]; % Posición inicial de las articulaciones
q3=[0 pi/4 -pi/2 0 0 0]; % Posición intermedia de las articulaciones
q4=[0 pi*3/4 -pi/2 0 0 0]; % Posición intermedia de las articulaciones

q={};

% Dividir la trayectoria en 4 segmentos
secciones = {T(1:8), T(8:15), T(15:22), T(22:end)};
q_trayectoria = []; % Trayectoria completa de articulaciones
q_actual = zeros(1, 6); % Estado inicial del robot

for s = 1:length(secciones)
    % Obtener posiciones finales del efector
    Ts = secciones{s};
    
    % Inicializar trayectoria parcial de articulaciones para esta sección
    q_seccion = [];
    
    for i = 1:length(Ts)
        % Calcular posición de articulaciones mediante cinemática inversa
        q_i = R.ikine(Ts{i}, 'q0', q_actual, 'mask', [1 1 1 0 0 0]);
        
        % Suavizar la transición en el primer punto de cada nueva sección
        if i == 1 && ~isempty(q_trayectoria)
            % Generar una transición suave entre la última posición anterior y la actual
            transicion = jtraj(q_trayectoria(end,:), q_i, 5);
            q_seccion = [q_seccion; transicion];
        else
            q_seccion = [q_seccion; q_i]
        end
        
        % Actualizar posición actual
        q_actual = q_i;
    end
    
    % Concatenar la trayectoria parcial de la sección a la trayectoria completa
    q_trayectoria = [q_trayectoria; q_seccion];
end
% Mover el robot a la posición calculada

% Graficar el primer robot usando plot3d
R.plot3d(q_trayectoria, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);

% Graficar el segundo robot también usando plot3d
%R2.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);
