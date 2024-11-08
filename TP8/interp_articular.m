clc; clear; close all;


Ri=RobotCI();
Ri.base =  transl(0,-1.35,0);

points = 2;%numero de puntos de interpolacion, a mayor numero menor la velocidad de animacion


[trayectoria_x, trayectoria_y, trayectoria_z] = graf_trayectoria();

%% Tubo izq 
disp('Tubo izq')
[tx_i, ty_i, tz_i, data]=tray_izq(); 


%% Calculo de las trayectorias izq 

qi_i = [0 0 0 0 0 0]*pi/180;

disp('trayectoria izq')
tra_i = ejecutar_trayectoria_soldadura_alineada_izq(Ri,1,tx_i,ty_i,tz_i,data,qi_i);


%% interpolación articular izq 
q_ti = []; q_vi = []; q_ai = []; 
for i = 1:size(tra_i, 1)-1
    [q_i, qd_i, qdd_i] = jtraj(tra_i(i,:), tra_i(i+1, :), points);
    q_ti = [q_ti; q_i];
    q_vi = [q_vi; qd_i];
    q_ai = [q_ai; qdd_i];
end



plotada(Ri,q_ti,q_vi,q_ai,trayectoria_x,trayectoria_y,trayectoria_z)

graficas_efector(Ri, q_ti, 0.1)



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


function q_traj = ejecutar_trayectoria_soldadura_alineada_izq(R, CinInv,trayectoria_x,trayectoria_y,trayectoria_z,data,q_inicial) %#ok<*INUSD>

    sing = false; %#ok<*NASGU>

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
        switch CinInv
            case 1 % Función personalizada
                if i == 1
                    q = Cin_Inv(Td, R, [0 0 0 0 0 0]*pi/180, true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
                end
                if i > 1
                    q = Cin_Inv(Td, R, q_traj(i-1, :), true); % Modificar 'zeros(6,1)' si deseas un q inicial específico
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

function plotada(R,q,v,a,cx,cy,cz) %#ok<DEFNU>

    figure();
    title('Trayectoria circular en el plano YZ');
    plot3(cx,cy,cz,'b','LineWidth',1.5);
    hold on;
    axis([-2 2 -2 2 0 2]);
    path = fullfile(pwd,'STL','KR16_2');
    R.plot3d(q, 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
    hold off;

    figure();
    title('Posición articular')
    qplot(q)
    ylabel('Posición (rad)')
    xlabel('Tiempo (s)')
    grid on


    figure();
    title('Velocidad articular')
    qplot(v)
    ylabel('Velocidad (rad/s)')
    xlabel('Tiempo (s)')
    grid on

    figure();
    title('Aceleración articular')
    qplot(a)
    ylabel('Aceleración (rad/s^2)')
    xlabel('Tiempo (s)')
    grid on
end

function graficas_efector(R, q_traj, dt)
    T_traj_art = R.fkine(q_traj).double;
    x_art = zeros(1,size(T_traj_art,3));
    y_art = zeros(1,size(T_traj_art,3));
    z_art = zeros(1,size(T_traj_art,3));

    %Posicion del efector
    for i=1:size(T_traj_art,3)
        x_art(i) = T_traj_art(1,4,i);
        y_art(i) = T_traj_art(2,4,i);
        z_art(i) = T_traj_art(3,4,i);
    end

    %Calculo de la velocidad del efector mediante derivacion numerica
    [row, col] = size(x_art);
    v_x = zeros(size(x_art));
    v_y = zeros(size(x_art));
    v_z = zeros(size(x_art));
    for c = 1:col-1
        v_x(1,c) = (x_art(1,c+1) - x_art(1,c))/dt; 
        v_y(1,c) = (y_art(1,c+1) - y_art(1,c))/dt;
        v_z(1,c) = (z_art(1,c+1) - z_art(1,c))/dt; 
    end


    %Calculo de la aceleracion del efector mediante derivacion numerica
    a_x = zeros(size(x_art));
    a_y = zeros(size(x_art));
    a_z = zeros(size(x_art));
    for c = 1:col-1
        a_x(1,c) = (v_x(1,c+1) - v_x(1,c))/dt; 
        a_y(1,c) = (v_y(1,c+1) - v_y(1,c))/dt;
        a_z(1,c) = (v_z(1,c+1) - v_z(1,c))/dt; 
    end



    figure()
    plot(x_art)
    hold on
    plot(y_art)
    hold on
    plot(z_art)
    title('Posicion del efector')
    legend('x', 'y', 'z')
    grid on
    ylabel('Posicion [m]')

    figure()
    plot(v_x)
    hold on
    plot(v_y)
    hold on
    plot(v_z)
    title('Velocidad del efector')
    legend('Vx', 'Vy', 'Vz')
    grid on
    ylabel('Velocidad [m/s]')

    figure()
    plot(a_x)
    hold on
    plot(a_y)
    hold on
    plot(a_z)
    title('Aceleracion del efector')
    legend('Ax', 'Ay', 'Az')
    grid on
    ylabel('Aceleracion [m/s^2]')

end