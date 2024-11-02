
clc; clear; close all;
dt=0.1; % Intervalo de tiempo

% Cargar el robot guardado en kuka_16.mat

R=RobotCI();
path = fullfile(pwd,'..','STL','KR16_2');


%Parámetros de la circunferencia
radio = 0.2; % Radio de la circunferencia
centro_y = 1.5; % Centro de la circunferencia en el eje Y
centro_z = 1; % Centro de la circunferencia en el eje Z
num_puntos = 10; % Número de puntos en la circunferencia

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

for i=1:num_puntos
    T{i} = transl(trayectoria_x(i),trayectoria_y(i),trayectoria_z(i)); % Posición inicial
end

q=[180 0 0 0 0 0]*pi/180; % Posición inicial de las articulaciones


q_trayectoria = []; % Trayectoria completa de articulaciones

for i=1:num_puntos
    q_i=TP5B_EjercicioTF(T{i}, R, q, 1);
    q_i = q_i';
    q_trayectoria = [q_trayectoria; q_i];
end
q_final=[];
q_finald=[];
q_finaldd=[];
for i=1:num_puntos-1
    [q_traj,qd,qdd]=jtraj(q_trayectoria(i,:), q_trayectoria(i+1,:), 10);
    q_final=[q_final;q_traj];
    q_finald=[q_finald;qd];
    q_finaldd=[q_finaldd;qdd];
end

% Mover el robot a la posición calculada

%R.plot3d(q_final, 'path',path,'notiles', 'nowrist','view',[90,0], 'scale', 0.1);


% Graficar el segundo robot también usando plot3d
%R2.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);

T_art = R.fkine(q_trayectoria).double;
x_art = zeros(1,size(T_art,3));
y_art = zeros(1,size(T_art,3));
z_art = zeros(1,size(T_art,3));

%Posici�n del efector
for i=1:size(T_art,3)
    x_art(i) = T_art(1,4,i);
    y_art(i) = T_art(2,4,i);
    z_art(i) = T_art(3,4,i);
end

%C�lculo de la velocidad del efector mediante derivaci�n num�rica
[row, col] = size(x_art);
v_x = zeros(size(x_art));
v_y = zeros(size(x_art));
v_z = zeros(size(x_art));
v = zeros(size(x_art));
for c = 1:col-1
    v_x(1,c) = (x_art(1,c+1) - x_art(1,c))/dt; 
    v_y(1,c) = (y_art(1,c+1) - y_art(1,c))/dt;
    v_z(1,c) = (z_art(1,c+1) - z_art(1,c))/dt; 
end
%C�lculo del m�dulo de la velocidad
for c = 1:col
    v(1,c) = sqrt(v_x(1,c)^2 + v_y(1,c)^2 + v_z(1,c)^2);
end

%C�lculo de la aceleraci�n del efector mediante derivaci�n num�rica
a_x = zeros(size(x_art));
a_y = zeros(size(x_art));
a_z = zeros(size(x_art));
a = zeros(size(x_art));
for c = 1:col-1
    a_x(1,c) = (v_x(1,c+1) - v_x(1,c))/dt; 
    a_y(1,c) = (v_y(1,c+1) - v_y(1,c))/dt;
    a_z(1,c) = (v_z(1,c+1) - v_z(1,c))/dt; 
end
%C�lculo del m�dulo de la aceleraci�n
for c = 1:col
    a(1,c) = sqrt(a_x(1,c)^2 + a_y(1,c)^2 + a_z(1,c)^2);
end


%Gr�ficas ROBOT 
figure()
title('Posici�n articular')
qplot(q_final)
xlabel('')
grid on

figure()
title('Velocidad articular')
qplot(q_finald)
ylabel('Velocidad articular [rad/s]')
xlabel('')
grid on

figure()
title('Aceleraci�n articular')
qplot(q_finaldd)
ylabel('Aceleraci�n articular [rad/s^2]')
xlabel('')
grid on

figure()
plot(x_art)
hold on
plot(y_art)
hold on
plot(z_art)
title('Posici�n del efector ')
legend('x', 'y', 'z')
grid on
ylabel('Posici�n [m]')

figure()
plot(v)
title('Velocidad del efector')
grid on
ylabel('Velocidad [m/s]')

figure()
plot(a)
title('Aceleraci�n del efector')
grid on
ylabel('Aceleraci�n [m/s^2]')