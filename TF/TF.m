clc; clear; close all;%#ok<*NCOMMA,*ASGLU,*NASGU,*DEFNU,*INUSD,*AGROW>


% Cargar el robot guardado en kuka_16.mat

Ri=RobotCI();
Ri.base =  transl(0,-1.35,0);

Rd=RobotCI();
Rd.base = transl(0,1.35,0);
% global path;
% path = fullfile(pwd,'..','STL','KR16_2');

[trayectoria_x trayectoria_y trayectoria_z] = graf_trayectoria(); 

%% Tubo izq y der
disp('Tubo izq')
[tx_i ty_i tz_i data]=tray_izq(); 
disp('Tubo der')
[tx_d ty_d tz_d data]=tray_der();

%% Calculo de las trayectorias izq y der

qi_i = [0 0 0 0 0 0]*pi/180;
qi_d = [0 0 0 0 0 0]*pi/180;

disp('trayectoria izq')
[tra_i vms_i] = ejecutar_trayectoria_soldadura_alineada_izq(Ri,1,tx_i,ty_i,tz_i,data,qi_i);
% disp(tra_i*180/pi)
disp('trayectoria der')
[tra_d vms_d] = ejecutar_trayectoria_soldadura_alineada_izq(Rd,1,tx_d,ty_d,tz_d,data,qi_d);
% disp(tra_d*180/pi)

% primer matriz
% disp(fkine(Ri,tra_i(1,:)))

%% interpolación articular izq y der
q_td = []; q_vd = []; q_ad = []; points = 2;
for i = 1:size(tra_d, 1)-1
    [q_d, qd_d, qdd_d] = jtraj(tra_d(i,:), tra_d(i+1, :), points);
    q_td = [q_td; q_d];
    q_vd = [q_vd; qd_d];
    q_ad = [q_ad; qdd_d];
end
q_ti = []; q_vi = []; q_ai = [];
for i = 1:size(tra_i, 1)-1
    [q_i, qd_i, qdd_i] = jtraj(tra_i(i,:), tra_i(i+1, :), points);
    q_ti = [q_ti; q_i];
    q_vi = [q_vi; qd_i];
    q_ai = [q_ai; qdd_i];
end

%% interpolación cartesiana izq y der con mstraj
% Parámetros de la trayectoria cartesiana
% vel = 0.1; % Velocidad máxima (puede ser un vector si se requiere diferente velocidad por dimensión)
% dt = 1; % Intervalo de tiempo entre puntos
% accel = 0.05; % Aceleración máxima
% % q0 = [90.0000   43.1370  -65.4540   90.0000 -108.3170  -90.0000]*pi/180; % Posición inicial
% % q0 = TP5B_EjercicioTF(homogenousTransform(vms_i(1, 1), vms_i(1, 2), vms_i(1, 3), vms_i(1, 4), vms_i(1, 5), vms_i(1, 6)), Ri,  [0 0 0 0 0 0]*pi/180, true)
% q0 = vms_i(1, :);

% % Generar la trayectoria
% traj_xyzrpy = mstraj(vms_i, vel, [], q0, dt, accel);

% % disp('vms_i')
% % disp(vms_i(1,:))
% % disp('traj_xyzrpy')
% % disp(traj_xyzrpy(1,:))
% % disp( homogenousTransform(traj_xyzrpy(1, 1), traj_xyzrpy(1, 2), traj_xyzrpy(1, 3), traj_xyzrpy(1, 4), traj_xyzrpy(1, 5), traj_xyzrpy(1, 6)) )

% traj_articular = zeros(size(traj_xyzrpy, 1), Ri.n);
% for i = 1:size(traj_xyzrpy, 1)
%     T = homogenousTransform(traj_xyzrpy(i, 1), traj_xyzrpy(i, 2), traj_xyzrpy(i, 3), traj_xyzrpy(i, 4), traj_xyzrpy(i, 5), traj_xyzrpy(i, 6));
%     % q = Ri.ikine(T, 'mask', [1 1 1 0 0 0]);
%     if i == 1
%         q = TP5B_EjercicioTF(T, Ri, [0 0 0 -90 0 0]*pi/180, true);
%     end
%     if i > 1
%         q = TP5B_EjercicioTF(T, Ri, traj_articular(i-1, :), true);
%     end
%     traj_articular(i, :) = q;
% end
% traj_articular
% % q_ti
% T_pre = homogenousTransform(traj_xyzrpy(1, 1), traj_xyzrpy(1, 2), traj_xyzrpy(1, 3), traj_xyzrpy(1, 4), traj_xyzrpy(1, 5), traj_xyzrpy(1, 6))
% T_post = Ri.fkine(traj_articular(1, :))

traj_articular_i = inter_mstraj(Ri, vms_i);
traj_articular_d = inter_mstraj(Rd, vms_d);

plotada(Ri,traj_articular_i,q_vi,q_ai,trayectoria_x,trayectoria_y,trayectoria_z)
plotada(Rd,traj_articular_d,q_vd,q_ad,trayectoria_x,trayectoria_y,trayectoria_z)

% figure;
% plot3(traj_xyzrpy(:, 1), traj_xyzrpy(:, 2), traj_xyzrpy(:, 3), 'r');
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Trayectoria en el espacio cartesiano');
% grid on;


function traj_articular = inter_mstraj(R, vms)
    %% interpolación cartesiana izq y der con mstraj
    % Parámetros de la trayectoria cartesiana
    vel = 0.1; % Velocidad máxima (puede ser un vector si se requiere diferente velocidad por dimensión)
    dt = 1; % Intervalo de tiempo entre puntos
    accel = 0.05; % Aceleración máxima
    % q0 = [90.0000   43.1370  -65.4540   90.0000 -108.3170  -90.0000]*pi/180; % Posición inicial
    % q0 = TP5B_EjercicioTF(homogenousTransform(vms_i(1, 1), vms_i(1, 2), vms_i(1, 3), vms_i(1, 4), vms_i(1, 5), vms_i(1, 6)), Ri,  [0 0 0 0 0 0]*pi/180, true)
    q0 = vms(1, :);

    % Generar la trayectoria
    traj_xyzrpy = mstraj(vms, vel, [], q0, dt, accel);

    % disp('vms_i')
    % disp(vms_i(1,:))
    % disp('traj_xyzrpy')
    % disp(traj_xyzrpy(1,:))
    % disp( homogenousTransform(traj_xyzrpy(1, 1), traj_xyzrpy(1, 2), traj_xyzrpy(1, 3), traj_xyzrpy(1, 4), traj_xyzrpy(1, 5), traj_xyzrpy(1, 6)) )

    traj_articular = zeros(size(traj_xyzrpy, 1), R.n);
    for i = 1:size(traj_xyzrpy, 1)
        T = homogenousTransform(traj_xyzrpy(i, 1), traj_xyzrpy(i, 2), traj_xyzrpy(i, 3), traj_xyzrpy(i, 4), traj_xyzrpy(i, 5), traj_xyzrpy(i, 6));
        % q = Ri.ikine(T, 'mask', [1 1 1 0 0 0]);
        if i == 1
            q = TP5B_EjercicioTF(T, R, [0 0 0 -90 0 0]*pi/180, true);
        end
        if i > 1
            q = TP5B_EjercicioTF(T, R, traj_articular(i-1, :), true);
        end
        traj_articular(i, :) = q;
    end

end


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
    theta = linspace(90*pi/180,270*pi/180,num_puntos); %Ángulos de la circunferencia
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
    theta = linspace(89*pi/180,269*pi/180,num_puntos); %Ángulos de la circunferencia
    ty_d = centro_y - radio * cos(theta); % Coordenada Y
    tz_d = centro_z - radio * sin(theta); % Coordenada Z
    tx_d = zeros(1, num_puntos); % Coordenada X fija (plano YZ)
end

function [q_traj vms] = ejecutar_trayectoria_soldadura_alineada_izq(R, Cin_Inv,trayectoria_x,trayectoria_y,trayectoria_z,data,q_inicial)

    sing = false; 

    radio = data(1); % Radio del tubo
    centro_y = data(2); % Centro en el eje Y
    centro_z = data(3); % Centro en el eje Z
    num_puntos = data(4); % Número de puntos en la trayectoria
    q_traj = zeros(num_puntos, R.n); % Almacenar todas las configuraciones articulares
    vms = zeros(num_puntos, 6); 
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
        rpy=tr2rpy(Td);
        vms(i,:) = [trayectoria_x(i), trayectoria_y(i), trayectoria_z(i), rpy(1), rpy(2), rpy(3)];
        
        % Selección de la función de cinemática inversa
        switch Cin_Inv
            case 1 % Función personalizada
                if i == 1
                    q = TP5B_EjercicioTF(Td, R, [0 0 0 0 0 0]*pi/180, true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
                end
                if i > 1
                    q = TP5B_EjercicioTF(Td, R, q_traj(i-1, :), true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
                end
            case 2 % Función de Robotics Toolbox 'ikine'
                q = R.ikine(Td, 'mask', [1 1 1 0 0 0]); % Evita la rotación en Td
            case 3 % Función 'ikcon'
                q = R.ikcon(Td); % Puede ser más precisa pero más lenta
            otherwise
                error('Valor de Cin_Inv no válido. Use 1, 2 o 3.')
        end
        q_traj(i, :) = q; % Almacenar la configuración articular
    end
end

function q_traj = ejecutar_trayectoria_soldadura_alineada_der(R, Cin_Inv,trayectoria_x,trayectoria_y,trayectoria_z,data,q_inicial) 
    sing = false;

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
                if i == 1
                    q = TP5B_EjercicioTF(Td, R, [0 0 0 0 0 0]*pi/180, true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
                end
                if i > 1
                    q = TP5B_EjercicioTF(Td, R, q_traj(i-1, :), true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
                end
            case 2 % Función de Robotics Toolbox 'ikine'
                q = R.ikine(Td, 'mask', [1 1 1 0 0 0]); % Evita la rotación en Td
            case 3 % Función 'ikcon'
                q = R.ikcon(Td); % Puede ser más precisa pero más lenta
            otherwise
                error('Valor de Cin_Inv no válido. Use 1, 2 o 3.')
        end
        q_traj(i, :) = q; % Almacenar la configuración articular

    end
    
end

function mostrar_doble_trayectoria(Ri, Rd, traj_i, traj_d, tray_x, tray_y, tray_z)
    % Crear la figura y establecer el espacio de trabajo
    figure;
    plot3(tray_x, tray_y, tray_z, 'b', 'LineWidth', 1.5); % Trayectoria del tubo
    hold on;
    
    % Configurar el espacio de trabajo de los ejes
    axis([-2 2 -2 2 0 2]); % Ajusta estos valores según el tamaño del espacio necesario
    % axis equal;
    
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
        % pause(0.05);
    end
    hold off;
end

function mostrar_simple_trayectoria(R,traj,tray_x,tray_y,tray_z)
    for i=1:size(traj,1)
        plot3(tray_x, tray_y, tray_z, 'b', 'LineWidth', 0.1); 
        path = fullfile(pwd,'..','STL','KR16_2');
        R.plot3d(traj(i, :), 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
        % pause(0.05); % Controla la velocidad de la simulación
    end
end

function plotada(R,q,v,a,cx,cy,cz)

    figure();
    title('Trayectoria circular en el plano YZ');
    plot3(cx,cy,cz,'b','LineWidth',1.5);
    hold on;
    axis([-2 2 -2 2 0 2]);
    path = fullfile(pwd,'..','STL','KR16_2');
    R.plot3d(q, 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
    hold off;

    figure();
    title('Posición')
    qplot(q)
    ylabel('Posición (rad)')
    xlabel('Tiempo (s)')
    grid on


    % figure();
    % title('Velocidad')
    % qplot(v)
    % ylabel('Velocidad (rad/s)')
    % xlabel('Tiempo (s)')
    % grid on

    % figure();
    % title('Aceleración')
    % qplot(a)
    % ylabel('Aceleración (rad/s^2)')
    % xlabel('Tiempo (s)')
    % grid on
end

function T = homogenousTransform(X, Y, Z, roll, pitch, yaw)
    % Matrices de rotación
    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];
      
    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];
      
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];
    
    % Matriz de rotación total
    R = Rz * Ry * Rx;
    
    % Matriz de transformación homogénea
    T = [R, [X; Y; Z];
         0, 0, 0, 1];
end
