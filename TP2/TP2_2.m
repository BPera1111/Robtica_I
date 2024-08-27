%% Ejercicio 2
T0 = eye(4);
a = [1;0.5;0;1]; %vector a con respecto a M homogeneo (le agrego el 1 para que quede de 4x1)
b = [0;0;1;1];
c = [1;0.5;0.3;1];
%Crear la matriz de rotación utilizando trotz con el ángulo en radianes
Ma = trotz(343*pi/180);
Mb = trotx(35*pi/180);
Mc = troty(90*pi/180);
ka = Ma * a; % Vector con respecto a O homogeneo.
kb = Mb * b; % Vector con respecto a O homogeneo.
kc = Mc * c; % Vector con respecto a O homogeneo.
kfa = ka(1:3, 1); % Vector con respecto a 0
kfb = kb(1:3, 1); % Vector con respecto a 0
kfc = kc(1:3, 1); % Vector con respecto a 0
figure 
trplot(T0, 'color', 'b', 'frame', 'O', 'length', 1.8) % Sistema de referencia.
hold on
trplot(Ma, 'color', 'r', 'frame', 'M', 'length', 1.8) % Sistema rotado según matriz de rotación.
grid on
rotate3d on
axis([-2 2 -2 2 -2 2])
view(0, 90)
plot3(kfa(1), kfa(2), kfa(3), '*k', 'markersize', 20, 'LineWidth', 1) % Ploteamos el punto
disp(kfa(1:3,1))
hold off
figure
trplot(T0, 'color', 'b', 'frame', 'O', 'length', 1.8) % Sistema de referencia.
hold on
trplot(Mb, 'color', 'r', 'frame', 'M', 'length', 1.8) % Sistema rotado según matriz de rotación.
grid on
rotate3d on
axis([-2 2 -2 2 -2 2])
view(0, 90)
plot3(kfb(1), kfb(2), kfb(3), '*k', 'markersize', 20, 'LineWidth', 1) % Ploteamos el punto
disp(kfb(1:3,1))
hold off
figure
trplot(T0, 'color', 'b', 'frame', 'O', 'length', 1.8) % Sistema de referencia.
hold on
trplot(Mc, 'color', 'r', 'frame', 'M', 'length', 1.8) % Sistema rotado según matriz de rotación.
grid on
rotate3d on
axis([-2 2 -2 2 -2 2])
view(0, 90)
plot3(kfc(1), kfc(2), kfc(3), '*k', 'markersize', 20, 'LineWidth', 1) % Ploteamos el punto
disp(kfc(1:3,1))
hold off
