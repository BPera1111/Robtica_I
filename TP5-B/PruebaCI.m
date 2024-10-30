%Robotica 1
% Prueba de CinInv
close all; clc; clear all;

R = RobotCI();
%disp(class(R)); % Debería mostrar 'SerialLink'
% Verificación del objeto SerialLink

% q = [40, -60, -30, 30, -40, 40] * pi/180; % Se propone este vector articular a modo de ejemplo.
q = semillas(R); % Se propone este vector articular a modo de ejemplo.
T = R.fkine(q); % Matriz T dato.

q0 = [0,0,0,0,0,0]*pi/180;

disp('Comparación de resultados:')
disp('q semilla:');
disp(q*180/pi);
% Cinemática inversa
qsol = TP5B_EjercicioTF(T, R, q0, 1);
%qsol = TP5B_EjercicioTF(T, R, q0, 0);

% Intentar de nuevo la cinemática inversa con un mayor número de iteraciones y tolerancia ajustada
% qinv = R.ikine(T, 'q0', q, 'tol', 1e-6, 'ilimit', 500);
qinv = R.ikcon(T);
disp('Solucion calculada con el metodo ikine:')
disp(qinv'*180/pi);

% Corroboración
%qalt = [ -2.5307, -0.5243, 0.0598, -0.8308, -0.4617, -1.2344];
Tp = R.fkine(qsol).double;
%Tpb = R.fkine(qalt).double;

disp('Matriz T calculada con fkine para el q semilla:')
disp(T)
disp('Matriz T calculada con fkine para la mejor solución de mi metodo de CInv:')
disp(Tp)
%disp('Matriz T calculada con fkine para la primer solución de mi metodo de CInv:')
%disp(Tpb)
mat_dif = double(T) - double(Tp);
disp('Diferencia entre las matrices T y Tp:')
disp(mat_dif)

qqq = [q' qsol qinv'];
graficar(R, qqq);





function q_ran = semillas(R)
    qlim_1 = R.qlim(1, 1:2);
    qlim_2 = R.qlim(2, 1:2);
    qlim_3 = R.qlim(3, 1:2);
    qlim_4 = R.qlim(4, 1:2);
    qlim_5 = R.qlim(5, 1:2);
    qlim_6 = R.qlim(6, 1:2);
    
    q1 = qlim_1(1) + (qlim_1(2)-qlim_1(1))*rand;
    q2 = qlim_2(1)+(qlim_2(2)-qlim_2(1))*rand;
    q3 = qlim_3(1)+(qlim_3(2)-qlim_3(1))*rand;
    % q3 = -pi/2;
    q4 = qlim_4(1)+(qlim_4(2)-qlim_4(1))*rand;
    q5 = qlim_5(1)+(qlim_5(2)-qlim_5(1))*rand;
    % q5 = 0;
    q6 = qlim_6(1)+(qlim_6(2)-qlim_6(1))*rand;
    
    q_ran = [q1,q2,q3,q4,q5,q6];

end

function graficar(R, q)
    figure;
    for i = 1:size(q, 2)
        clf; % Limpia la figura para mostrar la siguiente configuración
        % title(['Singularidad Muñeca ', num2str(i)]);
        if i == 1
            title('Semilla');
        elseif i == 2
            title('Cinemática Inversa');
        else
            title('Solución ikcon');
        end
        path = fullfile(pwd,'STL','KR16_2');
        % Visualizar la configuración actual
        % R.plot(q(:, i)', 'notiles', 'nowrist', 'view', [30, 30], 'scale', 0.1);
        R.plot3d(q(:, i)', 'path', path, 'notiles', 'nowrist', 'view', [30, 30], 'scale', 0.1);
        % Crear botón para avanzar a la siguiente configuración
        uicontrol('Style', 'pushbutton', 'String', 'Siguiente', ...
                  'Position', [20 20 80 40], ...
                  'Callback', 'uiresume(gcbf)');
        
        % Esperar hasta que el usuario presione el botón
        uiwait(gcf);
    end
end