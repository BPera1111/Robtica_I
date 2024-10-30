function R = RobotCI()
    % Define el robot usando la biblioteca Robotics Toolbox para MATLAB
    %Revisar
    dh = [0.000 0.675 0.260  -pi/2  0;
        0.000 0.000 0.680   0     0;
        0.000 0.000 0.035   -pi/2  0;
        0.000 0.670 0.000  pi/2  0;
        0.000 0.000 0.000   -pi/2  0;
        0.000 0.115 0.000   0     0];

    R = SerialLink(dh, 'name', 'KR16');

    % Configuración adicional del robot
    R.qlim(1,1:2) = [-185, 185]*pi/180;
    R.qlim(2,1:2) = [-65, 125]*pi/180;
    R.qlim(3,1:2) = [-220, 64]*pi/180;
    R.qlim(4,1:2) = [-350, 350]*pi/180;
    R.qlim(5,1:2) = [-130, 130]*pi/180;
    R.qlim(6,1:2) = [-350, 350]*pi/180;

    % R.base = transl(-1,1,0) * trotz(-45);
    % R.tool = transl(0,0,.2) * trotz(pi/2);
    R.offset = [0,-pi/2,0,0,0,0];
    % R.plot([0,0,0,0,0,0],'scale', 0.1)
    % R.teach()
end



    
