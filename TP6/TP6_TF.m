function TP6_TF; clc; clear ; close all; %#ok<*NOPRT,*CLEAR0ARGS,*NOPTS,*NASGU,*MINV>

    load('kuka_16.mat', 'R','q_kuka_16','path','workspace');

    % Visualización del robot en la posición cero
    % figure;
    % R.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);
    disp('Configuración Inicial del Robot');
    syms q1 q2 q3 q4 q5 q6 rea
    q = [q1 q2 q3 q4 q5 q6];
    dh = [0.000 0.675 0.260  -pi/2  0;
          0.000 0.000 0.680   0     0;
          0.000 0.000 -0.035   -pi/2  0;
          0.000 0.670 0.000  pi/2  0;
          0.000 0.000 0.000   -pi/2  0;
          0.000 0.115 0.000   0     0]
    R_sym = SerialLink(dh, 'name', 'Kuka 16_sym');
    J = simplify(R_sym.jacob0(q));
    disp('Jacobiano del Robot:');
    disp(vpa(J, 3));
    
    disp("Determinante simbolico");
    disp(simplify(det(J)));


    % disp('Determinante del Jacobiano:');
    % disp("q_kuka_16");
    % disp(q_kuka_16);
    % j = R.jacob0(q_kuka_16);
    % disp("Determinante del Jacobiano");
    % disp(det(j(1:6,1:6)));

    


end