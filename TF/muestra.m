clc; clear; close all;%#ok<*NCOMMA,*ASGLU,*NASGU,*DEFNU,*INUSD,*AGROW>

load('trayectoria_izq.mat','t_xyzrpy_i','tra_i','vms_i','t_a_i','v_a_i','a_a_i','Ri')
load('trayectoria_der.mat','t_xyzrpy_d','tra_d','vms_d','t_a_d','v_a_d','a_a_d','Rd')

Ri.tool = transl(0,0,0.01);
Rd.tool = transl(0,0,0.01);


pos_R1=[1,0,1.1];
pos_R2=[-1,0,1.1];
midpoint=(pos_R1+pos_R2)/2;

height=1;

radius=0.15;
[x,y,z] = cylinder(radius);

z1 = z*height+0.05;
z2 = -(z*height+0.05);

direction=pos_R2-pos_R1;
direction=direction/norm(direction);



% Calcular ángulos para rotar el cilindro
theta = atan2(direction(2), direction(1));  % Rotación en el plano XY
phi = acos(direction(3));  % Rotación respecto al eje Z

% Crear la matriz de rotación manualmente usando los ángulos
Rz = [cos(theta), -sin(theta), 0;
      sin(theta),  cos(theta), 0;
      0,          0,           1];
      
Ry = [cos(phi), 0, sin(phi);
      0,        1, 0;
     -sin(phi), 0, cos(phi)];

% Rotación final aplicando Rz primero y luego Ry
R_align = Rz * Ry;

% Aplicar la rotación y la traslación para posicionar el cilindro correctamente
cylinder_points_1 = R_align * [x(:)'; y(:)'; z1(:)'];
x_rot_1 = reshape(cylinder_points_1(1, :) + midpoint(1), size(x));
y_rot_1 = reshape(cylinder_points_1(2, :) + midpoint(2), size(y));
z_rot_1 = reshape(cylinder_points_1(3, :) + midpoint(3), size(z1));

cylinder_points_2 = R_align * [x(:)'; y(:)'; z2(:)'];
x_rot_2 = reshape(cylinder_points_2(1, :) + midpoint(1), size(x));
y_rot_2 = reshape(cylinder_points_2(2, :) + midpoint(2), size(y));
z_rot_2 = reshape(cylinder_points_2(3, :) + midpoint(3), size(z2));



% x1=x_rot(1,:);
% y1=y_rot(1,:);
% z1=z_rot(1,:);

% x2=x_rot(:,11:20);
% y2=y_rot(:,11:20);
% z2=z_rot(:,11:20);

% Configurar la figura y los cilindros
figure();
hold on;

% Dibujar los cilindros
surf(x_rot_1, y_rot_1, z_rot_1, 'FaceColor', [0 0 1], 'EdgeColor', 'none');
surf(x_rot_2, y_rot_2, z_rot_2, 'FaceColor', [0 1 0], 'EdgeColor', 'none');

% Ajustar los límites del gráfico y otras configuraciones
axis([-2 2 -2 2 0 2]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Simulación de Robots KUKA KR16');

% Cargar los modelos STL
path = fullfile(pwd, '..', 'STL', 'KR16_2');



% Realizar la animación sincronizada de ambos robots

Ri.plot3d(t_a_i, 'path', path, 'notiles', 'nowrist', 'view', [80, 10], 'scale', 0.1);
Rd.plot3d(t_a_d, 'path', path, 'notiles', 'nowrist', 'view', [100, 10], 'scale', 0.1);

% Ri.plot(t_a_i, 'trail', {'r', 'LineWidth', 1.5},scale=0.7);
% Rd.plot(t_a_d, 'trail', {'r', 'LineWidth', 1.5},scale=0.7);

plotada(Ri,t_a_i,v_a_i,a_a_i,'Robot Izquierdo');
plotada(Rd,t_a_d,v_a_d,a_a_d,'Robot Derecho');

hold off;

function plotada(R,q,v,a,robot)


      figure(); % Crear una sola figura
  
      % Gráfico de posición
      subplot(3, 1, 1); % Primer gráfico en una disposición de 3 filas y 1 columna
      title(['Posición ' robot]);
      qplot(q);
      ylabel('Posición (rad)');
      xlabel('Tiempo (s)');
      grid on;
      
      % Gráfico de velocidad
      subplot(3, 1, 2); % Segundo gráfico
      title(['Velocidad ' robot]);
      qplot(v);
      ylabel('Velocidad (rad/s)');
      xlabel('Tiempo (s)');
      grid on;
      
      % Gráfico de aceleración
      subplot(3, 1, 3); % Tercer gráfico
      title(['Aceleración ' robot]);
      qplot(a);
      ylabel('Aceleración (rad/s^2)');
      xlabel('Tiempo (s)');
      grid on;
  end