clc; clear ; close all; 

load('kuka_16.mat','R','path','q_kuka_16');
load('kuka_16_2.mat','R2','path_2','q_kuka_16_2');

R2.name = 'Kuka 16_2';


y_offset1 = -1.3; % Desplazamiento deseado en x
y_offset2 = 1.3; % Desplazamiento deseado en y

pos_R1=[1,0,1.1];
pos_R2=[-1,0,1.1];
midpoint=(pos_R1+pos_R2)/2;

% Crear una transformación homogénea de base para R2


R1.base = transl(0, y_offset1, 0)*trotz(pi/2);
R2.base = transl(0, y_offset2, 0)*trotz(-pi/2);


% Graficar los robots
R.plot3d(q_kuka_16, 'path', path, 'nowrist', 'noarrow', 'view', [90 0], 'delay', 0.01,'notiles');
R2.plot3d(q_kuka_16_2, 'path', path_2, 'nowrist', 'noarrow', 'view', [90 0], 'delay', 0.01,'notiles');

height=1;

radius=0.1;
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

% Graficar el cilindro en la posición deseada
hold on
surf(x_rot_1, y_rot_1, z_rot_1, 'FaceColor', [0 0 1], 'EdgeColor', 'none');
surf(x_rot_2, y_rot_2, z_rot_2, 'FaceColor', [0 1 0], 'EdgeColor', 'none');

hold off