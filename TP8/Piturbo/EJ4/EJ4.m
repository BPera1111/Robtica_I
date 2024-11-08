clc
clear
close all

%Definimos el robot �FANUC Paint Mate 200iA� a partir de los resultados
%obtenidos en el TP4
dh = [
    0      0.45   0.075 -pi/2  0;
    0      0      0.3    0     0;
    0      0      0.075 -pi/2  0;
    0      0.32   0      pi/2  0;
    0      0      0     -pi/2  0;
    0      0.008  0      0     0];
R = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');
%Aplicamos cinem�tica directa, obtenemos una matriz de transformaci�n
%homog�nea y extraemos la submatriz de rotaci�n
qq = [0 -pi/2 -pi/4 0 pi/4 0];
T = R.fkine(qq).double();
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformaci�n homog�neas, una para la posici�n
%inicial y otra para la posici�n final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

q1 = R.ikine(T1, qq);
q2 = R.ikine(T2, qq);

%Con q1 y q2, interpolamos en el espacio articular utilizando la funci�n
%jtraj
m = 100; %Cantidad de puntos de discretizaci�n
[q,qd,qdd] = jtraj(q1, q2, m); %Interpolamos en el espacio articular y obtenemos la posici�n, la velocidad y la aceleraci�n articulares

Tc = ctraj(T1, T2, m); %Interpolamos en el espacio cartesiano

%Aplicamos cinem�tica inversa y hallamos los valores de las variables
%articulares para cada matriz de transformaci�n obtenida.
q_ctraj = R.ikine(Tc, qq);

%Aplicamos la derivada num�rica hacia adelante a la posici�n articular
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

%Aplicamos cinem�tica directa a los vectores q obtenidos de jtraj para as�
%hallar las sucesivas matrices de transformaci�n homog�neas y luego poder
%comparar las sucesivas posiciones que va tomando el efector

T_jtraj = R.fkine(q).double();
[x_jtraj, y_jtraj, z_jtraj] = transl(T_jtraj);

[x_ctraj, y_ctraj, z_ctraj] = transl(Tc);

%-------------------------------------------------------------------------
%--------------------------------INICIO PLOTS-----------------------------
%-------------------------------------------------------------------------
%Inciso 1
figure()
title('Posici�n articular')
qplot(q)
xlabel('')

figure()
title('Velocidad articular')
qplot(qd)
ylabel('Velocidad articular [rad/s]')
xlabel('')

figure()
title('Aceleraci�n articular')
qplot(qdd)
ylabel('Aceleraci�n articular [rad/s^2]')
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
    title(strcat('Posici�n articular q',num2str(i)))
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
%Aceleraci�n articular
for i=1:col
    subplot(3,2,i)
    plot(qdd(:,i))
    hold on
    plot(qdd_num(:,i))
    grid on
    title(strcat('Aceleraci�n articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ + Der.NUM')
end

%Inciso 3
figure()
hold on
subplot(3,1,1)
plot(x_ctraj)
hold on
plot(x_jtraj)
title('Posici�n X Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,2)
plot(y_ctraj)
hold on
plot(y_jtraj)
title('Posici�n Y Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,3)
plot(z_ctraj)
hold on
plot(z_jtraj)
title('Posici�n Z Efector');
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
clc
clear
close all

%Definimos el robot �FANUC Paint Mate 200iA� a partir de los resultados
%obtenidos en el TP4
dh = [
    0      0.45   0.075 -pi/2  0;
    0      0      0.3    0     0;
    0      0      0.075 -pi/2  0;
    0      0.32   0      pi/2  0;
    0      0      0     -pi/2  0;
    0      0.008  0      0     0];
R = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');
%Aplicamos cinem�tica directa, obtenemos una matriz de transformaci�n
%homog�nea y extraemos la submatriz de rotaci�n
qq = [0 -pi/2 -pi/4 0 pi/4 0];
T = R.fkine(qq).double();
T_rot = T(1:3,1:3);

%Armamos dos matrices de transformaci�n homog�neas, una para la posici�n
%inicial y otra para la posici�n final
P1 = [0; 0; 0.95];
P2 = [0.4; 0; 0.95];

T1 = [T_rot P1;
      0 0 0 1];
T2 = [T_rot P2;
      0 0 0 1];

q1 = R.ikine(T1, qq);
q2 = R.ikine(T2, qq);

%Con q1 y q2, interpolamos en el espacio articular utilizando la funci�n
%jtraj
m = 100; %Cantidad de puntos de discretizaci�n
[q,qd,qdd] = jtraj(q1, q2, m); %Interpolamos en el espacio articular y obtenemos la posici�n, la velocidad y la aceleraci�n articulares

Tc = ctraj(T1, T2, m); %Interpolamos en el espacio cartesiano

%Aplicamos cinem�tica inversa y hallamos los valores de las variables
%articulares para cada matriz de transformaci�n obtenida.
q_ctraj = R.ikine(Tc, qq);

%Aplicamos la derivada num�rica hacia adelante a la posici�n articular
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

%Aplicamos cinem�tica directa a los vectores q obtenidos de jtraj para as�
%hallar las sucesivas matrices de transformaci�n homog�neas y luego poder
%comparar las sucesivas posiciones que va tomando el efector

T_jtraj = R.fkine(q).double();
[x_jtraj, y_jtraj, z_jtraj] = transl(T_jtraj);

[x_ctraj, y_ctraj, z_ctraj] = transl(Tc);

%-------------------------------------------------------------------------
%--------------------------------INICIO PLOTS-----------------------------
%-------------------------------------------------------------------------
%Inciso 1
figure()
title('Posici�n articular')
qplot(q)
xlabel('')

figure()
title('Velocidad articular')
qplot(qd)
ylabel('Velocidad articular [rad/s]')
xlabel('')

figure()
title('Aceleraci�n articular')
qplot(qdd)
ylabel('Aceleraci�n articular [rad/s^2]')
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
    title(strcat('Posici�n articular q',num2str(i)))
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
%Aceleraci�n articular
for i=1:col
    subplot(3,2,i)
    plot(qdd(:,i))
    hold on
    plot(qdd_num(:,i))
    grid on
    title(strcat('Aceleraci�n articular q',num2str(i)))
    legend('JTRAJ', 'CTRAJ + Der.NUM')
end

%Inciso 3
figure()
hold on
subplot(3,1,1)
plot(x_ctraj)
hold on
plot(x_jtraj)
title('Posici�n X Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,2)
plot(y_ctraj)
hold on
plot(y_jtraj)
title('Posici�n Y Efector');
grid on
legend('CTRAJ', 'JTRAJ');

subplot(3,1,3)
plot(z_ctraj)
hold on
plot(z_jtraj)
title('Posici�n Z Efector');
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
