function[R1,R2] = robot()
    % Suárez - Corazza - Masi.

    %--------------------------------
    % KR 120 R2100 nano F exclusive.
    %--------------------------------
    dh = [0 0.570 0.300 -pi/2 0;   % Matriz de parámetros de DH.
          0 0     0.900  0    0;
          0 0     0     -pi/2 0;
          0 0.900 0     -pi/2 0;
          0 0     0      pi/2 0;
          0 0.240 0      0    0];
 
    R1 = SerialLink(dh,'name','KR 120 R2100 nano F exclusive - 1'); % Robot 1.

    R2 = SerialLink(dh,'name','KR 120 R2100 nano F exclusive - 2'); % Robot 2.

    % Vector de booleanos en donde se indique qué sistemas de referencia se desean visualizar.

    % Rangos de los movimientos.
    R1.qlim(1,1:2) = [-165,    165]*pi/180; 
    R1.qlim(2,1:2) = [90-135,90+45]*pi/180; 
    R1.qlim(3,1:2) = [-155,     65]*pi/180;
    R1.qlim(4,1:2) = [-350,    350]*pi/180;
    R1.qlim(5,1:2) = [-125,    125]*pi/180;
    R1.qlim(6,1:2) = [-350,    350]*pi/180;

    R2.qlim(1,1:2) = [-165,    165]*pi/180; 
    R2.qlim(2,1:2) = [90-135,90+45]*pi/180; 
    R2.qlim(3,1:2) = [-155,     65]*pi/180;
    R2.qlim(4,1:2) = [-350,    350]*pi/180;
    R2.qlim(5,1:2) = [-125,    125]*pi/180;
    R2.qlim(6,1:2) = [-350,    350]*pi/180;

    R1.base = transl(-2,2,0)*trotz(-pi/4);
    R1.tool = transl(0,0,0.5);
    R1.offset = [0 -pi/2 0 0 0 0];

    R2.base = transl(2,-2,0)*trotz(3*pi/4);
    R2.tool = transl(0,0,0.5);
    R2.offset = [0 -pi/2 0 0 0 0];
end

