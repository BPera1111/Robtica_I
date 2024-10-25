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

q1 = [0 0 0 0 0 0];
q2 = [0 -pi/2 pi/4 -pi/5 -pi/10 0];
q3 = [pi -pi/4 -pi/4 -2*pi/5 -3*pi/5 -pi/2];
q4 = [pi/5 0 -pi/5 -pi/3 -2*pi/5 -pi];

via_points = [q1
              q2;
              q3;
              q4]; %Vector de puntos de paso
qd_max = [1, 1, 1, 1, 1, 1]; %Vector de velocidades máximas (arbitrario)

%Con estos vectores articulares, interpolamos entre ellos utilizando mstraj
dt = 0.05;
q = mstraj(via_points, qd_max, [], q1, dt, 1);

%Aplicamos derivación numérica para hallar la velocidad y aceleración
%articular
[row, col] = size(q);
qd_num = zeros(size(q));
qdd_num = zeros(size(q));
for c = 1:col
    for r = 2:row-1
       qd_num(r,c) = (q(r+1,c) - q(r,c))/dt; 
    end
end

for c = 1:col
    for r = 2:row-1
       qdd_num(r,c) = (qd_num(r+1,c) - qd_num(r,c))/dt; 
    end
end
figure()
R.plot(q)


figure()
title('Posición articular')
qplot(q)
xlabel('')

figure()
title('Velocidad articular')
qplot(qd_num)
ylabel('Velocidad articular [rad/s]')
xlabel('')

figure()
title('Aceleración articular')
qplot(qdd_num)
ylabel('Aceleración articular [rad/s^2]')
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

q1 = [0 0 0 0 0 0];
q2 = [0 -pi/2 pi/4 -pi/5 -pi/10 0];
q3 = [pi -pi/4 -pi/4 -2*pi/5 -3*pi/5 -pi/2];
q4 = [pi/5 0 -pi/5 -pi/3 -2*pi/5 -pi];

via_points = [q1
              q2;
              q3;
              q4]; %Vector de puntos de paso
qd_max = [1, 1, 1, 1, 1, 1]; %Vector de velocidades máximas (arbitrario)

%Con estos vectores articulares, interpolamos entre ellos utilizando mstraj
dt = 0.05;
q = mstraj(via_points, qd_max, [], q1, dt, 1);

%Aplicamos derivación numérica para hallar la velocidad y aceleración
%articular
[row, col] = size(q);
qd_num = zeros(size(q));
qdd_num = zeros(size(q));
for c = 1:col
    for r = 2:row-1
       qd_num(r,c) = (q(r+1,c) - q(r,c))/dt; 
    end
end

for c = 1:col
    for r = 2:row-1
       qdd_num(r,c) = (qd_num(r+1,c) - qd_num(r,c))/dt; 
    end
end
figure()
R.plot(q)


figure()
title('Posición articular')
qplot(q)
xlabel('')

figure()
title('Velocidad articular')
qplot(qd_num)
ylabel('Velocidad articular [rad/s]')
xlabel('')

figure()
title('Aceleración articular')
qplot(qdd_num)
ylabel('Aceleración articular [rad/s^2]')
>>>>>>> 5d5cfbc (por favor funciona)
xlabel('')