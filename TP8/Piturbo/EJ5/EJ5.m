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

%Con estos vectores articulares, interpolamos entre ellos con 100 puntos de
%discretización
m = 100;
[q_traj_1, qd1, qdd1] = jtraj(q1,q2,m);
[q_traj_2, qd2, qdd2] = jtraj(q2,q3,m);
[q_traj_3, qd3, qdd3] = jtraj(q3,q4,m);

q = [q_traj_1;
    q_traj_2;
    q_traj_3];

qd = [qd1;
      qd2;
      qd3];

qdd = [qdd1;
      qdd2;
      qdd3];
%Finalmente realizamos una simulación con el robot FANUC Paint Mate 200iA
figure();
R.plot(q);

%Ploteamos la evolución temporal de la posición, velocidad y aceleración
%articular
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