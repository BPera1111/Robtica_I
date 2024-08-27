% Resolución Trabajo Práctico N°2.
% Babolene

clc, clear, close all
% %% Ejercicio 1
% % Grafique el sistema {𝑀} respecto de {𝑂} para cada una de las siguientes matrices de rotación:
% Ta=[0.5 -0.866 0 0
%     0.866 0.5 0 0
%     0 0 1 0
%     0 0 0 1];
% Tb=[0 0 1 0
%     -1 0 0 0
%     0 -1 0 0
%     0 0 0 1];
% Tc=[0.5 -0.75 -0.433 0
%     0.866 0.433 0.250 0
%     0 -0.5 0.866 0
%     0 0 0 1];
% % Ploteo de los sistemas rotados.
% T0=eye(4); %Matriz identidad de 4x4
% Tk = input('Ingrese la matriz que desea usar Ta Tb o Tc: ');
% if Tk==Tc
%     trplot(T0,'color','b','frame','M','length',1.8) % Sistema de referencia.
%     hold on
%     trplot(Tk,'color','r','frame','0','length',1) % Sistema rotado según matriz de rotación.
%     grid on
%     rotate3d on
%     axis([-1 2 -1 2 -1 2]) %limites de cada eje
%     view(179,20) %angulo de vista inicial
% else
%     trplot(T0,'color','b','frame','0','length',1.8) % Sistema de referencia.
%     hold on
%     trplot(Tk,'color','r','frame','M','length',1) % Sistema rotado según matriz de rotación.
%     grid on
%     rotate3d on
%     axis([-1 2 -1 2 -1 2]) %limites de cada eje
%     view(179,20) %angulo de vista inicial
% end
% %% Ejercicio 2
% T0 = eye(4);
% a = [1;0.5;0;1]; %vector a con respecto a M homogeneo (le agrego el 1 para que quede de 4x1)
% b = [0;0;1;1];
% c = [1;0.5;0.3;1];
% %Crear la matriz de rotación utilizando trotz con el ángulo en radianes
% Ma = trotz(343);
% k0 = Ma * a; % Vector con respecto a O homogeneo.
% kf = k0(1:3, 1); % Vector con respecto a 0
% trplot(T0, 'color', 'b', 'frame', 'O', 'length', 1.8) % Sistema de referencia.
% hold on
% trplot(Ma, 'color', 'r', 'frame', 'M', 'length', 1.8) % Sistema rotado según matriz de rotación.
% grid on
% rotate3d on
% axis([-2 2 -2 2 -2 2])
% view(0, 90)
% plot3(kf(1), kf(2), kf(3), '*k', 'markersize', 20, 'LineWidth', 1) % Ploteamos el punto
% disp(kf(1:3,1))
%% Ejercicio 4
% T0=eye(4);
% T=[0.894 0.447 0 7
%      -0.447 0.894 0 4
%      0 0 1 0
%      0 0 0 1];
% aM=[2
%     1
%     0
%     1];
% a0=T*aM;
% trplot(T0, 'color', 'b', 'frame', 'O', 'length', 7) % Sistema de referencia.
% hold on
% trplot(T, 'color', 'r', 'frame', 'M', 'length', 2) % Sistema rotado según matriz de rotación.
% grid on
% rotate3d on
% axis([-10 10 -10 10 -10 10])
% view(0, 90)
% plot3(a0(1), a0(2), a0(3), '*k', 'markersize', 20, 'LineWidth', 1) % Ploteamos el punto
% disp('El vector a respecto de 0 es:')
% disp(a0(1:3,1))
%% Ejercicio 5 y 6
% T0=eye(4);
% Trot = troty(45);
% Ttras = [1 0 0 0
%          0 1 0 0
%          0 0 1 1
%          0 0 0 1];
% T = Ttras * Trot;%Matriz de transformación del sistema 0 respecto de M
% a=[0.5
%     0
%     1
%     1];%vector a respecto de 0
% Tinv=inv(T);%Matriz de transformacion del sistema M respecto de 0
% aM=Tinv*a;
% % Sistema de referencia.
% trplot(T0,'color','b','frame','O','length',1.8)
% hold on
% % % Sistema rotado según matriz de rotación.
% trplot(Ttras,'color','r','frame','M','length',1.8)
% % Sistema rotado y trasladado según matriz de rotación*traslacion.
% trplot(T,'color','g','frame','M*','length',1.8)
% grid on
% rotate3d on
% axis([-2 2 -2 2 -2 2])
% view(130,30)
% plot3(aM(1), aM(2), aM(3), '*k', 'markersize', 20, 'LineWidth', 0.5) % Ploteamos el punto
% disp('El vector a respecto de 0 es:')
% disp(aM(1:3,1))
% %%
% %% Ejercicio 5 y 6
% T0=eye(4);
% Trot1 = troty(45);
% Ttras1 = [1 0 0 0
%          0 1 0 0
%          0 0 1 1
%          0 0 0 1];
% T1 = Trot1 * Ttras1;
% a1=[0.5
%     0
%     1
%     1];
% Tinv=inv(T1);
% aM1=Tinv*a1;
% % Sistema de referencia.
% figure(2)
% trplot(T0,'color','b','frame','O','length',1.8)
% hold on
% % % Sistema rotado según matriz de rotación.
% trplot(Trot1,'color','r','frame','M','length',1.8)
% % Sistema rotado y trasladado según matriz de rotación*traslacion.
% trplot(T1,'color','g','frame','M*','length',1.8)
% grid on
% rotate3d on
% axis([-2 2 -2 2 -2 2])
% view(130,30)
% plot3(aM1(1), aM1(2), aM1(3), '*k', 'markersize', 20, 'LineWidth', 0.5) % Ploteamos el punto
% disp('El vector a respecto de 0 es:')
% disp(aM1(1:3,1))






