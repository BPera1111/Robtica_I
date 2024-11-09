clc; clear; close all;


Ri=RobotCI();
Ri.base =  transl(0,-1.35,0);


[trayectoria_x trayectoria_y trayectoria_z] = graf_trayectoria();

%% Tubo izq 
disp('Tubo izq')
[tx_i ty_i tz_i data]=tray_izq(); 

%% Calculo de las trayectorias izq 

qi_i = [0 0 0 0 0 0]*pi/180;

disp('trayectoria izq')
q_tra_i = ejecutar_trayectoria_soldadura_alineada_izq(Ri,tx_i,ty_i,tz_i,data,qi_i);




plotada(Ri,q_tra_i,trayectoria_x,trayectoria_y,trayectoria_z)

graficas_efector(Ri, q_tra_i, 0.1)
graficas_articulares(q_tra_i, 0.1)




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


function q_traj = ejecutar_trayectoria_soldadura_alineada_izq(R,trayectoria_x,trayectoria_y,trayectoria_z,data,q_inicial) 

    sing = false; %#ok<*NASGU>
    Td={};
    radio = data(1); % Radio del tubo
    centro_y = data(2); % Centro en el eje Y
    centro_z = data(3); % Centro en el eje Z
    num_puntos = data(4); % Número de puntos en la trayectoria

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
        Td{i} = [R_desired, [trayectoria_x(i); trayectoria_y(i); trayectoria_z(i)]; 0 0 0 1];
    end

    T_trayectoria = zeros(4,4,1);
    
    for i=1:num_puntos-1
        T_trayectoria = concatenar_matriz(T_trayectoria, ctraj(Td{i}, Td{i+1}, 3));
    end

    q_traj = []; % Trayectoria completa de articulaciones

    for i=1:num_puntos*3-2
        if i == 1
            [q_i qq]=Cin_Inv(T_trayectoria(:,:,i), R, q_inicial, 1);
        end
        if i > 1
            [q_i qq]=Cin_Inv(T_trayectoria(:,:,i), R, q_traj(i-1, :), 1);
        end
        q_traj(i, :) = q_i;
    end
end

function [T_traj] = concatenar_matriz(T_traj1,T_traj2)
    T_traj = zeros(4,4,size(T_traj1,3)+size(T_traj2,3));
    T_traj(:,:,1:size(T_traj1,3)) = T_traj1;
    T_traj(:,:,size(T_traj1,3)+1:end) = T_traj2;
end


function plotada(R,q,cx,cy,cz) 

    figure();
    title('Trayectoria circular en el plano YZ');
    plot3(cx,cy,cz,'b','LineWidth',1.5);
    hold on;
    axis([-2 2 -2 2 0 2]);
    path = fullfile(pwd,'STL','KR16_2');
    R.plot3d(q, 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);
    hold off;
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

function graficas_articulares(q_cart, dt)
    %C�lculo de la velocidad articular
    [row, col] = size(q_cart);
    qd_cart = zeros(size(q_cart));
    qdd_cart = zeros(size(q_cart));
    for c = 1:col
        for r = 2:row-1
        qd_cart(r,c) = (q_cart(r+1,c) - q_cart(r,c))/dt; 
        end
    end

    for c = 1:col
        for r = 2:row-1
        qdd_cart(r,c) = (qd_cart(r+1,c) - qd_cart(r,c))/dt; 
        end
    end

    figure()
    title('Posicion articular')
    qplot(q_cart)
    ylabel('Posicion articular [rad]')
    xlabel('')
    grid on

    figure()
    title('Velocidad articular')
    qplot(qd_cart)
    ylabel('Velocidad articular [rad/s]')
    xlabel('')
    grid on

    figure()
    title('Aceleracion articular')
    qplot(qdd_cart)
    ylabel('Aceleracion articular [rad/s^2]')
    xlabel('')
    grid on

end