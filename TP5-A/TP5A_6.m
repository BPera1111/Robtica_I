function TP5A_6; clc; clear ; close all; %#ok<*NOPRT,*CLEAR0ARGS,*NOPTS,*NASGU,*MINV>

    % LBRIIWA7R800 (KUKA)

    dh = [0 0.340 0      pi/2 0; % Matriz de parámetros de DH.
          0 0     0      pi/2 0;
          0 0.400 0      pi/2 0;
          0 0     0      pi/2 0;
          0 0.400 0      pi/2 0;
          0 0     0      pi/2 0;
          0 0.126 0      0    0];

    R = SerialLink(dh,'name','LBRIIWA7R800');
    q = [0,0,0,0,0,0,0]; % Valor inicial de las articulaciones.

    % Rangos de los movimientos.
    R.qlim(1,1:2) = [-170,  170]*pi/180;
    R.qlim(2,1:2) = [-120,  120]*pi/180;
    R.qlim(3,1:2) = [-170,  170]*pi/180;
    R.qlim(4,1:2) = [-120,  120]*pi/180;
    R.qlim(5,1:2) = [-170,  170]*pi/180;
    R.qlim(6,1:2) = [-120,  120]*pi/180;
    R.qlim(7,1:2) = [-175,  175]*pi/180;

    R.offset = [0 pi 0 pi 0 pi 0]; % Offset de las articulaciones.

    % Objetivo
    disp("Objetivo");
    T = [1 0 0 0.23;
         0 1 0 0.70;
         0 0 1 0.60;
         0 0 0    1]

    % Semillas
    Q1 = [pi/2 pi/2 0 0 0 0 0] ;      % Vector inicial 1.
    Q2 = [pi/2 -pi/2 0 -pi/2 0 0 0];  % Vector inicial 2.
    Q3 = [pi -pi/2 pi/2 -pi/2 0 0 0]; % Vector inicial 3.
    Semillas = [Q1; Q2; Q3]; % Matriz de semillas.
    

   

    for i = 1:3
        solucion_num = i; % Para numerar las soluciones correctamente
        disp(["Solución Cinemática Inversa ", num2str(solucion_num)]);Semillas(i,:)
        [q, T_ver, error] = C_INV_DIR(R,T,Semillas(i,:));
        disp(["Solución Cinemática Inversa ", num2str(solucion_num)]); q
        disp(["Transformación Verificada ", num2str(solucion_num)]); T_ver
        disp(["Error ", num2str(solucion_num)]); error
    end


end


function [qsol,T_ver,error] = C_INV_DIR(Robot,Objetivo,Semilla)

    qsol = Robot.ikine(Objetivo,'q0',Semilla,'ilimit',1500); %Vector de soluciones
    T_ver = Robot.fkine(qsol).T; %Matriz de transformación verificada
    error = Objetivo - T_ver; %Error de la transformación
    
end