clc; clear; close all;


% Cargar el robot guardado en kuka_16.mat

Ri=RobotCI();
Ri.base =  transl(0,-1.43,0);

Rd=RobotCI();
Rd.base = transl(0,1.43,0);
% global path;
% path = fullfile(pwd,'..','STL','KR16_2');

[trayectoria_x trayectoria_y trayectoria_z] = graf_trayectoria();

%% Tubo izq y der
disp('Tubo izq')
[tx_i ty_i tz_i data]=tray_izq();
disp('Tubo der')
[tx_d ty_d tz_d data]=tray_der();

%% Calculo de las trayectorias izq y der
% disp('trayectoria izq')
% tra_i = ejecutar_trayectoria_soldadura_alineada(Ri,3,tx_i,ty_i,tz_i,data);
% disp('trayectoria der')
% tra_d = ejecutar_trayectoria_soldadura_alineada(Rd,3,tx_d,ty_d,tz_d,data);

%% Guardar las trayectorias para no tener que calcularlas todo el tiempo
% save('trayectorias.mat', 'tra_i', 'tra_d')
% disp('trayectorias guardadas')

%% Cargar las trayectorias guardadas
load('trayectorias.mat', 'tra_i', 'tra_d')

% for i=1:data(4)
%     plot3(trayectoria_x, trayectoria_y, trayectoria_z, 'b', 'LineWidth', 0.1); 
%     path = fullfile(pwd,'..','STL','KR16_2');
%     Rd.plot3d(tra_d(i, :), 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
%     Ri.plot3d(tra_i(i, :), 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
%     % pause(0.05); % Controla la velocidad de la simulación

% end

mostrar_doble_trayectoria(Ri, Rd, tra_i, tra_d, trayectoria_x, trayectoria_y, trayectoria_z)


function [tx ty tz]=graf_trayectoria()
    %Parámetros de la circunferencia
    radio = 0.15; % Radio de la circunferencia
    centro_y = 0; % Centro de la circunferencia en el eje Y
    centro_z = 1.1; % Centro de la circunferencia en el eje Z
    num_puntos = 100; % Número de puntos en la circunferencia

    % Generar puntos de la circunferencia en el plano YZ
    theta = linspace(0, 2*pi, num_puntos); % Ángulos de la circunferencia
    ty = centro_y + radio * cos(theta); % Coordenada Y
    tz = centro_z + radio * sin(theta); % Coordenada Z
    tx = zeros(1, num_puntos); % Coordenada X fija (plano YZ)


    % Graficar la trayectoria en el plano YZ
    % figure();
    % plot3(trayectoria_x, trayectoria_y, trayectoria_z, 'b', 'LineWidth', 0.5); 
    % % xlabel('Y'); ylabel('Z');
    % xlabel('x'); ylabel('y'); zlabel('z')
    % title('Trayectoria circular en el plano YZ');
    % axis ([-2 2 -1 4 -1 3]);
end

function [tx_i ty_i tz_i data]=tray_izq()
    % Parámetros de la trayectoria circular
    radio = 0.2; % Radio del tubo
    centro_y = 0; % Centro en el eje Y
    centro_z = 1.1; % Centro en el eje Z
    num_puntos = 100; % Número de puntos en la trayectoria
    data = [radio centro_y centro_z num_puntos];
    
    % Generación de puntos de la trayectoria circular en el plano YZ
    % theta = linspace(91*pi/180,271*pi/180 , num_puntos); % Ángulos de la circunferencia
    theta = linspace(270*pi/180,90*pi/180,num_puntos); %Ángulos de la circunferencia
    ty_i = centro_y + radio * cos(theta); % Coordenada Y
    tz_i = centro_z + radio * sin(theta); % Coordenada Z
    tx_i = zeros(1, num_puntos); % Coordenada X fija (plano YZ)
end

function [tx_d ty_d tz_d data]=tray_der()
    % Parámetros de la trayectoria circular
    radio = 0.2; % Radio del tubo
    centro_y = 0; % Centro en el eje Y
    centro_z = 1.1; % Centro en el eje Z
    num_puntos = 100; % Número de puntos en la trayectoria
    data = [radio centro_y centro_z num_puntos];
    
    % Generación de puntos de la trayectoria circular en el plano YZ
    % theta = linspace(91*pi/180,271*pi/180 , num_puntos); % Ángulos de la circunferencia
    theta = linspace(269*pi/180,89*pi/180,num_puntos); %Ángulos de la circunferencia
    ty_d = centro_y - radio * cos(theta); % Coordenada Y
    tz_d = centro_z - radio * sin(theta); % Coordenada Z
    tx_d = zeros(1, num_puntos); % Coordenada X fija (plano YZ)
end



function q_traj = ejecutar_trayectoria_soldadura_alineada(R, Cin_Inv,trayectoria_x,trayectoria_y,trayectoria_z,data)
    % % Parámetros de la trayectoria circular
    % radio = 0.2; % Radio del tubo
    % centro_y = 0; % Centro en el eje Y
    % centro_z = 1.1; % Centro en el eje Z
    % num_puntos = 100; % Número de puntos en la trayectoria
    
    % % Generación de puntos de la trayectoria circular en el plano YZ
    % % theta = linspace(91*pi/180,271*pi/180 , num_puntos); % Ángulos de la circunferencia
    % theta = linspace(ti,tf,num_puntos); %Ángulos de la circunferencia
    % trayectoria_y = centro_y + radio * cos(theta); % Coordenada Y
    % trayectoria_z = centro_z + radio * sin(theta); % Coordenada Z
    % trayectoria_x = zeros(1, num_puntos); % Coordenada X fija (plano YZ)

    radio = data(1); % Radio del tubo
    centro_y = data(2); % Centro en el eje Y
    centro_z = data(3); % Centro en el eje Z
    num_puntos = data(4); % Número de puntos en la trayectoria
    q_traj = zeros(num_puntos, R.n); % Almacenar todas las configuraciones articulares
    % Iterar sobre cada punto de la trayectoria
    for i = 1:num_puntos
        % Calcula el vector de dirección hacia el centro del tubo
        direccion = [trayectoria_x(i), centro_y - trayectoria_y(i), centro_z - trayectoria_z(i)];
        direccion = direccion / norm(direccion); % Normalizar el vector
        
        % Calcular la matriz de rotación deseada
        z_axis = direccion; % El eje Z del efector debe apuntar hacia el centro del tubo
        x_axis = cross([0 0 1], z_axis); % Calcula el eje X ortogonal
        x_axis = x_axis / norm(x_axis);
        y_axis = cross(z_axis, x_axis); % Calcula el eje Y ortogonal
        
        % Construir la matriz de rotación deseada para el efector
        R_desired = [x_axis; y_axis; z_axis]';
        
        % Matriz de transformación deseada en el punto actual
        Td = [R_desired, [trayectoria_x(i); trayectoria_y(i); trayectoria_z(i)]; 0 0 0 1];
        
        % Selección de la función de cinemática inversa
        switch Cin_Inv
            case 1 % Función personalizada
                q = TP5B_EjercicioTF(Td, R, zeros(6,1), true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
            case 2 % Función de Robotics Toolbox 'ikine'
                q = R.ikine(Td, 'mask', [1 1 1 0 0 0]); % Evita la rotación en Td
            case 3 % Función 'ikcon'
                q = R.ikcon(Td); % Puede ser más precisa pero más lenta
            otherwise
                error('Valor de Cin_Inv no válido. Use 1, 2 o 3.')
        end
        q_traj(i, :) = q; % Almacenar la configuración articular
    end
    
    % Mover el robot a través de todas las configuraciones articulares
    % for i = 1:num_puntos
    %     plot3(trayectoria_x, trayectoria_y, trayectoria_z, 'b', 'LineWidth', 0.1); 
    %     path = fullfile(pwd,'..','STL','KR16_2');
    %     R.plot3d(q_traj(i, :), 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
    %     pause(0.05); % Controla la velocidad de la simulación
    % end
end


function mostrar_doble_trayectoria(Ri, Rd, traj_i, traj_d, tray_x, tray_y, tray_z)
    % Crear la figura y establecer el espacio de trabajo
    figure;
    plot3(tray_x, tray_y, tray_z, 'b', 'LineWidth', 1.5); % Trayectoria del tubo
    hold on;
    
    % Configurar el espacio de trabajo de los ejes
    axis([-1 1 -2 2 0 2]); % Ajusta estos valores según el tamaño del espacio necesario
    axis equal;
    
    % Etiquetas y título
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Simulación de Doble Robot con Trayectoria Circular');
    
    % Path para el modelo 3D de los robots
    path = fullfile(pwd, '..', 'STL', 'KR16_2');
    
    % Número de puntos en la trayectoria
    num_puntos = size(traj_i, 1);
    
    % Animar ambos robots
    for i = 1:num_puntos
        % Dibujar los robots en sus configuraciones actuales
        Ri.plot3d(traj_i(i, :), 'path', path, 'notiles', 'nowrist', 'view', [90, 0], 'scale', 0.1);
        Rd.plot3d(traj_d(i, :), 'path', path, 'notiles', 'nowrist', 'view', [90, 0], 'scale', 0.1);
        
        % Pausar para controlar la velocidad de la animación
        pause(0.05);
    end
    hold off;
end

