function TP5A_5; clc; clear ; close all; %#ok<*NOPRT,*CLEAR0ARGS,*NOPTS,*NASGU,*MINV>

    a1 = 0.5;
    a2 = 0.4;
    a3 = 0.3;
    dh = [0 0 a1 0 0;
          0 0 a2 0 0;
          0 0 a3 0 0];
    R = SerialLink(dh,'name','Ejemplo 2.1');
    q = [0,0,0]; % Vector inicial de valores articulares.
    R.qlim = [-180, 180; -180, 180; -180, 180] * pi/180;

    % Valores de x, y y gamma
    x = 0.575;
    y = 0.62;
    g = 75*pi/180; % Ángulo gamma en radianes.

    qsol = C_INV(a1,a2,a3,dh,x,y,g) 

end

function qsol = C_INV(a1,a2,a3,dh,x,y,g)

    qsol=zeros(2,3); % Vector articular, hay dos soluciones posibles.
    % Posición del sistema 2 con respecto al sistema 0
    % P_2^0 = P_3^0 - a3 * x_3^0
    x2=x-a3*cos(g);
    y2=y-a3*sin(g);

    % Cálculo de q1
    r=sqrt(x2^2+y2^2);
    beta_1=atan2(y2,x2);
    alfa_1=acos((a2^2-a1^2-r^2)/(-2*a1*r));

    qsol(1,1)=beta_1+alfa_1; % Solución 1
    qsol(2,1)=beta_1-alfa_1; % Solución 2

    % Cálculo de q2

    for i=1:2
        T1=Transf_Sistemas(dh(1,:),qsol(i,1));
        P2_1 = inv(T1)*[x2;y2;0;1];
        qsol(i,2)=atan2(P2_1(2),P2_1(1));
    end

    % Cálculo de q3

    for i=1:2
        T1=Transf_Sistemas(dh(1,:),qsol(i,1));
        T2= T1 * Transf_Sistemas(dh(2,:),qsol(i,2));
        P3_2 = inv(T2)*[x;y;0;1];
        qsol(i,3)=atan2(P3_2(2),P3_2(1));
    end
    
end

function T = Transf_Sistemas(fila_dh, q)
    T = trotz(q)*transl(0,0,fila_dh(2))*transl(fila_dh(3),0,0)*trotx(fila_dh(4));
  end