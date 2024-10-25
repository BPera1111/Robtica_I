<<<<<<< HEAD
clc
clear
close all

%Definimos el robot “FANUC Paint Mate 200iA” a partir de los resultados
%obtenidos en el TP4
dh = [
    0      0.45   0.075 -pi/2  0;
    0      0      0.3    0     0;
    0      0      0.075 -pi/2  0;
    0      0.32   0      pi/2  0;
    0      0      0     -pi/2  0;
    0      0.008  0      0     0];
R = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');
%Aplicamos cinemática directa, obtenemos una matriz de transformación
%homogénea y extraemos la submatriz de rotación
qq = [0 -pi/2 -pi/4 0 pi/4 0];
T = R.fkine(qq).double();
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformación homogéneas, una para la posición
%inicial y otra para la posición final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

q1 = R.ikine(T1, qq);
q2 = R.ikine(T2, qq);

%Con q1 y q2, interpolamos en el espacio articular utilizando la función
%jtraj
m = 100; %Cantidad de puntos de discretización
[q,qd,qdd] = jtraj(q1, q2, m); %Interpolamos en el espacio articular y obtenemos la posición, la velocidad y la aceleración articulares

Tc = ctraj(T1, T2, m); %Interpolamos en el espacio cartesiano

%Aplicamos cinemática inversa y hallamos los valores de las variables
%articulares para cada matriz de transformación obtenida.
q_ctraj = R.ikine(Tc, qq);

%Aplicamos la derivada numérica hacia adelante a la posición articular
[row, col] = size(q_ctraj);
qd_num = zeros(size(q_ctraj));
qdd_num = zeros(size(q_ctraj));
for c = 1:col
    for r = 2:row-1
       qd_num(r,c) = (q_ctraj(r+1,c) - q_ctraj(r,c))*m; 
    end
end

for c = 1:col
    for r = 2:row-1
       qdd_num(r,c) = (qd_num(r+1,c) - qd_num(r,c))*m; 
    end
end

%Aplicamos cinemática directa a los vectores q obtenidos de jtraj para así
%hallar las sucesivas matrices de transformación homogéneas y luego poder
%comparar las sucesivas posiciones que va tomando el efector

T_jtraj = R.fkine(q).double();
[x_jtraj, y_jtraj, z_jtraj] = transl(T_jtraj);

[x_ctraj, y_ctraj, z_ctraj] = transl(Tc);

%-------------------------------------------------------------------------
%--------------------------------INICIO PLOTS-----------------------------
%-------------------------------------------------------------------------
%Inciso 1
figure()
title('Posición articular')
qplot(q)
xlabel('')

figure()
title('Velocidad articular')
qplot(qd)
ylabel('Velocidad articular [rad/s]')
xlabel('')

figure()
title('Aceleración articular')
qplot(qdd)
ylabel('Aceleración articular [rad/s^2]')
xlabel('')

%Inciso 2
figure()
hold on
%Posicion articular
for i=1:col
    subplot(3,2,i)
    plot(q(:,i))
    hold on
    plot(q_ctraj(:,i))
    grid on
    title(strcat('Posición articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ')
end


figure()
hold on
%Velocidad articular
for i=1:col
    subplot(3,2,i)
    plot(qd(:,i))
    hold on
    plot(qd_num(:,i))
    grid on
    title(strcat('Velocidad articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ + Der.NUM')
end

figure()
hold on
%Aceleración articular
for i=1:col
    subplot(3,2,i)
    plot(qdd(:,i))
    hold on
    plot(qdd_num(:,i))
    grid on
    title(strcat('Aceleración articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ + Der.NUM')
end

%Inciso 3
figure()
hold on
subplot(3,1,1)
plot(x_ctraj)
hold on
plot(x_jtraj)
title('Posición X Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,2)
plot(y_ctraj)
hold on
plot(y_jtraj)
title('Posición Y Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,3)
plot(z_ctraj)
hold on
plot(z_jtraj)
title('Posición Z Efector');
grid on
legend('CTRAJ', 'JTRAJ');

%Inciso 4
figure()
plot(x_ctraj, z_ctraj)
hold on
plot(x_jtraj, z_jtraj)
grid on
legend('CTRAJ', 'JTRAJ')
title('Z vs X')
xlabel('X [m]')
ylabel('Z [m]')
=======
clc
clear
close all

%Definimos el robot “FANUC Paint Mate 200iA” a partir de los resultados
%obtenidos en el TP4
dh = [
    0      0.45   0.075 -pi/2  0;
    0      0      0.3    0     0;
    0      0      0.075 -pi/2  0;
    0      0.32   0      pi/2  0;
    0      0      0     -pi/2  0;
    0      0.008  0      0     0];
R = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');
%Aplicamos cinemática directa, obtenemos una matriz de transformación
%homogénea y extraemos la submatriz de rotación
qq = [0 -pi/2 -pi/4 0 pi/4 0];
T = R.fkine(qq).double();
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformación homogéneas, una para la posición
%inicial y otra para la posición final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

q1 = R.ikine(T1, qq);
q2 = R.ikine(T2, qq);

%Con q1 y q2, interpolamos en el espacio articular utilizando la función
%jtraj
m = 100; %Cantidad de puntos de discretización
[q,qd,qdd] = jtraj(q1, q2, m); %Interpolamos en el espacio articular y obtenemos la posición, la velocidad y la aceleración articulares

Tc = ctraj(T1, T2, m); %Interpolamos en el espacio cartesiano

%Aplicamos cinemática inversa y hallamos los valores de las variables
%articulares para cada matriz de transformación obtenida.
q_ctraj = R.ikine(Tc, qq);

%Aplicamos la derivada numérica hacia adelante a la posición articular
[row, col] = size(q_ctraj);
qd_num = zeros(size(q_ctraj));
qdd_num = zeros(size(q_ctraj));
for c = 1:col
    for r = 2:row-1
       qd_num(r,c) = (q_ctraj(r+1,c) - q_ctraj(r,c))*m; 
    end
end

for c = 1:col
    for r = 2:row-1
       qdd_num(r,c) = (qd_num(r+1,c) - qd_num(r,c))*m; 
    end
end

%Aplicamos cinemática directa a los vectores q obtenidos de jtraj para así
%hallar las sucesivas matrices de transformación homogéneas y luego poder
%comparar las sucesivas posiciones que va tomando el efector

T_jtraj = R.fkine(q).double();
[x_jtraj, y_jtraj, z_jtraj] = transl(T_jtraj);

[x_ctraj, y_ctraj, z_ctraj] = transl(Tc);

%-------------------------------------------------------------------------
%--------------------------------INICIO PLOTS-----------------------------
%-------------------------------------------------------------------------
%Inciso 1
figure()
title('Posición articular')
qplot(q)
xlabel('')

figure()
title('Velocidad articular')
qplot(qd)
ylabel('Velocidad articular [rad/s]')
xlabel('')

figure()
title('Aceleración articular')
qplot(qdd)
ylabel('Aceleración articular [rad/s^2]')
xlabel('')

%Inciso 2
figure()
hold on
%Posicion articular
for i=1:col
    subplot(3,2,i)
    plot(q(:,i))
    hold on
    plot(q_ctraj(:,i))
    grid on
    title(strcat('Posición articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ')
end


figure()
hold on
%Velocidad articular
for i=1:col
    subplot(3,2,i)
    plot(qd(:,i))
    hold on
    plot(qd_num(:,i))
    grid on
    title(strcat('Velocidad articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ + Der.NUM')
end

figure()
hold on
%Aceleración articular
for i=1:col
    subplot(3,2,i)
    plot(qdd(:,i))
    hold on
    plot(qdd_num(:,i))
    grid on
    title(strcat('Aceleración articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ + Der.NUM')
end

%Inciso 3
figure()
hold on
subplot(3,1,1)
plot(x_ctraj)
hold on
plot(x_jtraj)
title('Posición X Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,2)
plot(y_ctraj)
hold on
plot(y_jtraj)
title('Posición Y Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,3)
plot(z_ctraj)
hold on
plot(z_jtraj)
title('Posición Z Efector');
grid on
legend('CTRAJ', 'JTRAJ');

%Inciso 4
figure()
plot(x_ctraj, z_ctraj)
hold on
plot(x_jtraj, z_jtraj)
grid on
legend('CTRAJ', 'JTRAJ')
title('Z vs X')
xlabel('X [m]')
ylabel('Z [m]')
>>>>>>> 5d5cfbc (por favor funciona)
