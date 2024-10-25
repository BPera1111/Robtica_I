<<<<<<< HEAD
% Cargar el objeto R desde el archivo .mat y no ejecutar robot cada vez que se quiera probar el robot
% Requiere que se ejecute primero una vez robot.m

function herramienta; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS,*NASGU>
    load('kuka_16.mat', 'R','q_kuka_16','path','workspace');

    vector_q=[0,0,0,0,0,0]; %posicion de las articulaciones
    vector_s=[1,1,1,0,0,0,0];%1 si se quiere mostrar el sistema de la articulacion, 0 si no
    figure('name', 'Kuka 16 con stl');
    R.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);
    R.base = trotz(0);
    % Parámetros de la circunferencia
    radio = 0.1; % Radio de la circunferencia
    centro_y = 0.75; % Centro de la circunferencia en el eje Y
    centro_z = 1.25; % Centro de la circunferencia en el eje Z
    num_puntos = 10; % Número de puntos en la circunferencia

    % Generar puntos de la circunferencia en el plano YZ
    theta = linspace(0, 2*pi, num_puntos); % Ángulos de la circunferencia
    trayectoria_y = centro_y + radio * cos(theta); % Coordenada Y
    trayectoria_z = centro_z + radio * sin(theta); % Coordenada Z
    trayectoria_x = zeros(1, num_puntos); % Coordenada X fija (plano YZ)

    % Graficar la trayectoria en el plano YZ
    hold on
    plot3(trayectoria_z, trayectoria_x, trayectoria_y, 'b', 'LineWidth', 2);
    xlabel('Y'); ylabel('Z');
    title('Trayectoria circular en el plano YZ');
    R.teach('scale',0.001,'trail', {'r', 'LineWidth', 2}, 'workspace', workspace);
end



=======
% Cargar el objeto R desde el archivo .mat y no ejecutar robot cada vez que se quiera probar el robot
% Requiere que se ejecute primero una vez robot.m

function herramienta; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS,*NASGU>
    load('kuka_16.mat', 'R','q_kuka_16','path','workspace');

    vector_q=[0,0,0,0,0,0]; %posicion de las articulaciones
    vector_s=[1,1,1,0,0,0,0];%1 si se quiere mostrar el sistema de la articulacion, 0 si no
    figure('name', 'Kuka 16 con stl');
    R.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);
    R.base = trotz(0);
    % Parámetros de la circunferencia
    radio = 0.1; % Radio de la circunferencia
    centro_y = 1.1; % Centro de la circunferencia en el eje Y
    centro_z = 1; % Centro de la circunferencia en el eje Z
    num_puntos = 5; % Número de puntos en la circunferencia

    % Generar puntos de la circunferencia en el plano YZ
    theta = linspace(0, 2*pi, num_puntos); % Ángulos de la circunferencia
    trayectoria_y = centro_y + radio * cos(theta); % Coordenada Y
    trayectoria_z = centro_z + radio * sin(theta); % Coordenada Z
    trayectoria_x = zeros(1, num_puntos); % Coordenada X fija (plano YZ)

    % Graficar la trayectoria en el plano YZ
    hold on
    plot3(trayectoria_z, trayectoria_x, trayectoria_y, 'b', 'LineWidth', 2);
    xlabel('Y'); ylabel('Z');
    title('Trayectoria circular en el plano YZ');
    R.teach('scale',0.001,'trail', {'r', 'LineWidth', 2}, 'workspace', workspace);
end



>>>>>>> 5d5cfbc (por favor funciona)
