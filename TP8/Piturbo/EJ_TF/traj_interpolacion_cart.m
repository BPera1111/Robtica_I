puntos_trayectoria

dt = 0.1;

% %Interpolaci�n cartesiana - ROBOT 1
% Ttraj_1 = ctraj(T0,T1,10);
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T1, T0, 10));
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T0, T2, 50));
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T2, T3, 10));
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T3, T4, 10));
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T4, T3, 10));
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T3, T2, 10));
% Ttraj_1 = concatenar_matriz(Ttraj_1, ctraj(T2, T0, 20));
% 
% x1_cart = zeros(1,size(Ttraj_1,3));
% y1_cart = zeros(1,size(Ttraj_1,3));
% z1_cart = zeros(1,size(Ttraj_1,3));
% q1_cart = zeros(size(Ttraj_1,3),6);
% %Posici�n del efector y posicion articular
% for i=1:size(Ttraj_1,3)
%     x1_cart(i) = Ttraj_1(1,4,i);
%     y1_cart(i) = Ttraj_1(2,4,i);
%     z1_cart(i) = Ttraj_1(3,4,i);
%     q1_cart(i,:) = CinInv(Ttraj_1(:,:,i), R1, q_ref, 1);
% end
% 
% %C�lculo de la velocidad articular
% [row, col] = size(q1_cart);
% qd1_cart = zeros(size(q1_cart));
% qdd1_cart = zeros(size(q1_cart));
% for c = 1:col
%     for r = 2:row-1
%        qd1_cart(r,c) = (q1_cart(r+1,c) - q1_cart(r,c))/dt; 
%     end
% end
% 
% for c = 1:col
%     for r = 2:row-1
%        qdd1_cart(r,c) = (qd1_cart(r+1,c) - qd1_cart(r,c))/dt; 
%     end
% end
% 
% %C�lculo de la velocidad del efector mediante derivaci�n num�rica
% [row, col] = size(x1_cart);
% v1_x = zeros(size(x1_cart));
% v1_y = zeros(size(x1_cart));
% v1_z = zeros(size(x1_cart));
% v1 = zeros(size(x1_cart));
% for c = 1:col-1
%     v1_x(1,c) = (x1_cart(1,c+1) - x1_cart(1,c))/dt; 
%     v1_y(1,c) = (y1_cart(1,c+1) - y1_cart(1,c))/dt;
%     v1_z(1,c) = (z1_cart(1,c+1) - z1_cart(1,c))/dt; 
% end
% %C�lculo del m�dulo de la velocidad
% for c = 1:col
%     v1(1,c) = sqrt(v1_x(1,c)^2 + v1_y(1,c)^2 + v1_z(1,c)^2);
% end
% 
% %C�lculo de la aceleraci�n del efector mediante derivaci�n num�rica
% a1_x = zeros(size(x1_cart));
% a1_y = zeros(size(x1_cart));
% a1_z = zeros(size(x1_cart));
% a1 = zeros(size(x1_cart));
% for c = 1:col-1
%     a1_x(1,c) = (v1_x(1,c+1) - v1_x(1,c))/dt; 
%     a1_y(1,c) = (v1_y(1,c+1) - v1_y(1,c))/dt;
%     a1_z(1,c) = (v1_z(1,c+1) - v1_z(1,c))/dt; 
% end
% %C�lculo del m�dulo de la velocidad
% for c = 1:col
%     a1(1,c) = sqrt(a1_x(1,c)^2 + a1_y(1,c)^2 + a1_z(1,c)^2);
% end

% %Gr�ficas ROBOT 1
% figure()
% title('Posici�n articular - ROBOT 1')
% qplot(q1_cart)
% xlabel('')
% grid on
% 
% figure()
% title('Velocidad articular - ROBOT 1')
% qplot(qd1_cart)
% ylabel('Velocidad articular [rad/s]')
% xlabel('')
% grid on
% 
% figure()
% title('Aceleraci�n articular - ROBOT 1')
% qplot(qdd1_cart)
% ylabel('Aceleraci�n articular [rad/s^2]')
% xlabel('')
% grid on
% 
% figure()
% plot(x1_cart)
% hold on
% plot(y1_cart)
% hold on
% plot(z1_cart)
% title('Posici�n del efector - ROBOT 1')
% legend('x', 'y', 'z')
% grid on
% ylabel('Posici�n [m]')
% 
% figure()
% plot(v1)
% title('Velocidad del efector - ROBOT 1')
% grid on
% ylabel('Velocidad [m/s]')
% 
% figure()
% plot(a1)
% title('Aceleraci�n del efector - ROBOT 1')
% grid on
% ylabel('Aceleraci�n [m/s^2]')
% 

%Interpolaci�n cartesiana - ROBOT 2
% Ttraj_2 = ctraj(T2_0,T2_1,10);
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_1, T2_2, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_2, T2_1, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_1, T2_0, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_0, T2_3, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_3, T2_4, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_4, T2_5, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_5, T2_6, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_6, T2_5, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_5, T2_4, 10));
% Ttraj_2 = concatenar_matriz(Ttraj_2, ctraj(T2_4, T2_0, 10));
% 
% x2_cart = zeros(1,size(Ttraj_2,3));
% y2_cart = zeros(1,size(Ttraj_2,3));
% z2_cart = zeros(1,size(Ttraj_2,3));
% q2_cart = zeros(size(Ttraj_2,3),6);
% %Posici�n del efector y posicion articular
% for i=1:size(Ttraj_2,3)
%     x2_cart(i) = Ttraj_2(1,4,i);
%     y2_cart(i) = Ttraj_2(2,4,i);
%     z2_cart(i) = Ttraj_2(3,4,i);
%     q2_cart(i,:) = CinInv(Ttraj_2(:,:,i), R2, q_ref, 1);
% end
% 
% %C�lculo de la velocidad articular
% [row, col] = size(q2_cart);
% qd2_cart = zeros(size(q2_cart));
% qdd2_cart = zeros(size(q2_cart));
% for c = 1:col
%     for r = 2:row-1
%        qd2_cart(r,c) = (q2_cart(r+1,c) - q2_cart(r,c))/dt; 
%     end
% end
% 
% for c = 1:col
%     for r = 2:row-1
%        qdd2_cart(r,c) = (qd2_cart(r+1,c) - qd2_cart(r,c))/dt; 
%     end
% end
% 
% %C�lculo de la velocidad del efector mediante derivaci�n num�rica
% [row, col] = size(x2_cart);
% v2_x = zeros(size(x2_cart));
% v2_y = zeros(size(x2_cart));
% v2_z = zeros(size(x2_cart));
% v2 = zeros(size(x2_cart));
% for c = 1:col-1
%     v2_x(1,c) = (x2_cart(1,c+1) - x2_cart(1,c))/dt; 
%     v2_y(1,c) = (y2_cart(1,c+1) - y2_cart(1,c))/dt;
%     v2_z(1,c) = (z2_cart(1,c+1) - z2_cart(1,c))/dt; 
% end
% %C�lculo del m�dulo de la velocidad
% for c = 1:col
%     v2(1,c) = sqrt(v2_x(1,c)^2 + v2_y(1,c)^2 + v2_z(1,c)^2);
% end
% 
% %C�lculo de la aceleraci�n del efector mediante derivaci�n num�rica
% a2_x = zeros(size(x2_cart));
% a2_y = zeros(size(x2_cart));
% a2_z = zeros(size(x2_cart));
% a2 = zeros(size(x2_cart));
% for c = 1:col-1
%     a2_x(1,c) = (v2_x(1,c+1) - v2_x(1,c))/dt; 
%     a2_y(1,c) = (v2_y(1,c+1) - v2_y(1,c))/dt;
%     a2_z(1,c) = (v2_z(1,c+1) - v2_z(1,c))/dt; 
% end
% %C�lculo del m�dulo de la velocidad
% for c = 1:col
%     a2(1,c) = sqrt(a2_x(1,c)^2 + a2_y(1,c)^2 + a2_z(1,c)^2);
% end
% 
% %Gr�ficas ROBOT 2
% figure()
% title('Posici�n articular - ROBOT 2')
% qplot(q2_cart)
% xlabel('')
% grid on
% 
% figure()
% title('Velocidad articular - ROBOT 2')
% qplot(qd2_cart)
% ylabel('Velocidad articular [rad/s]')
% xlabel('')
% grid on
% 
% figure()
% title('Aceleraci�n articular - ROBOT 2')
% qplot(qdd2_cart)
% ylabel('Aceleraci�n articular [rad/s^2]')
% xlabel('')
% grid on
% 
% figure()
% plot(x2_cart)
% hold on
% plot(y2_cart)
% hold on
% plot(z2_cart)
% title('Posici�n del efector - ROBOT 2')
% legend('x', 'y', 'z')
% grid on
% ylabel('Posici�n [m]')
% 
% figure()
% plot(v2)
% title('Velocidad del efector - ROBOT 2')
% grid on
% ylabel('Velocidad [m/s]')
% 
% figure()
% plot(a2)
% title('Aceleraci�n del efector - ROBOT 2')
% grid on
% ylabel('Aceleraci�n [m/s^2]')


%Interpolaci�n cartesiana - ROBOT 3
Ttraj_3 = ctraj(T3_0,T3_1,10);
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_1, T3_2, 10));
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_2, T3_1, 10));
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_1, T3_0, 10));
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_0, T3_3, 10));
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_3, T3_4, 10));
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_4, T3_3, 10));
Ttraj_3 = concatenar_matriz(Ttraj_3, ctraj(T3_3, T3_0, 10));


x3_cart = zeros(1,size(Ttraj_3,3));
y3_cart = zeros(1,size(Ttraj_3,3));
z3_cart = zeros(1,size(Ttraj_3,3));
q3_cart = zeros(size(Ttraj_3,3),6);
%Posici�n del efector y posicion articular
for i=1:size(Ttraj_3,3)
    x3_cart(i) = Ttraj_3(1,4,i);
    y3_cart(i) = Ttraj_3(2,4,i);
    z3_cart(i) = Ttraj_3(3,4,i);
    q3_cart(i,:) = CinInv(Ttraj_3(:,:,i), R3, q_ref, 1);
end

%C�lculo de la velocidad articular
[row, col] = size(q3_cart);
qd3_cart = zeros(size(q3_cart));
qdd3_cart = zeros(size(q3_cart));
for c = 1:col
    for r = 2:row-1
       qd3_cart(r,c) = (q3_cart(r+1,c) - q3_cart(r,c))/dt; 
    end
end

for c = 1:col
    for r = 2:row-1
       qdd3_cart(r,c) = (qd3_cart(r+1,c) - qd3_cart(r,c))/dt; 
    end
end

%C�lculo de la velocidad del efector mediante derivaci�n num�rica
[row, col] = size(x3_cart);
v3_x = zeros(size(x3_cart));
v3_y = zeros(size(x3_cart));
v3_z = zeros(size(x3_cart));
v3 = zeros(size(x3_cart));
for c = 1:col-1
    v3_x(1,c) = (x3_cart(1,c+1) - x3_cart(1,c))/dt; 
    v3_y(1,c) = (y3_cart(1,c+1) - y3_cart(1,c))/dt;
    v3_z(1,c) = (z3_cart(1,c+1) - z3_cart(1,c))/dt; 
end
%C�lculo del m�dulo de la velocidad
for c = 1:col
    v3(1,c) = sqrt(v3_x(1,c)^2 + v3_y(1,c)^2 + v3_z(1,c)^2);
end

%C�lculo de la aceleraci�n del efector mediante derivaci�n num�rica
a3_x = zeros(size(x3_cart));
a3_y = zeros(size(x3_cart));
a3_z = zeros(size(x3_cart));
a3 = zeros(size(x3_cart));
for c = 1:col-1
    a3_x(1,c) = (v3_x(1,c+1) - v3_x(1,c))/dt; 
    a3_y(1,c) = (v3_y(1,c+1) - v3_y(1,c))/dt;
    a3_z(1,c) = (v3_z(1,c+1) - v3_z(1,c))/dt; 
end
%C�lculo del m�dulo de la velocidad
for c = 1:col
    a3(1,c) = sqrt(a3_x(1,c)^2 + a3_y(1,c)^2 + a3_z(1,c)^2);
end

%Gr�ficas ROBOT 2
figure()
title('Posici�n articular - ROBOT 3')
qplot(q3_cart)
xlabel('')
grid on

figure()
title('Velocidad articular - ROBOT 3')
qplot(qd3_cart)
ylabel('Velocidad articular [rad/s]')
xlabel('')
grid on

figure()
title('Aceleraci�n articular - ROBOT 3')
qplot(qdd3_cart)
ylabel('Aceleraci�n articular [rad/s^2]')
xlabel('')
grid on

figure()
plot(x3_cart)
hold on
plot(y3_cart)
hold on
plot(z3_cart)
title('Posici�n del efector - ROBOT 3')
legend('x', 'y', 'z')
grid on
ylabel('Posici�n [m]')

figure()
plot(v3)
title('Velocidad del efector - ROBOT 3')
grid on
ylabel('Velocidad [m/s]')

figure()
plot(a3)
title('Aceleraci�n del efector - ROBOT 3')
grid on
ylabel('Aceleraci�n [m/s^2]')

% W = [-3 5 -3 10 -3 5];
% figure()
% R3.plot(q3_cart,'workspace',W,'scale',0.5,'jointdiam',0.5,'notiles','noname')
