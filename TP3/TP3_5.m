function TP3_5; clc ; close all %#ok<*NOPRT>
    %#ok<*NASGU>

    dh_SCARA=[0.000 0.195 0.200 0.000 0;
              0.000 0.000 0.250 0.000 0;
              0.000 0.000 0.000 pi    1;
              0.000 0.000 0.000 0.000 0];
    q_SCARA=[0,0,0,0];
    qlim_SCARA=[-140,  140; 
                -150,  150; 
                 0.0, 0.18*180/pi; 
                -400,  400]*pi/180;

    dh_paint=[0.000 0.450 0.075   pi/2  0;
              0.000 0.000 0.300   0     0;
              0.000 0.000 0.075   pi/2  0;
              0.000 0.320 0.000  -pi/2  0;
              0.000 0.000 0.000   pi/2  0;
              0.000 0.000 0.000   0     0];
    q_paint=[0,0,0,0,0,0];
    qlim_paint=[-170, 170; 
                -100, 100; 
                -194, 194; 
                -190, 190; 
                -120, 120; 
                -360, 360]*pi/180;
    
    dh_iiwa=[0.000 0.340 0.000 -pi/2 0;
             0.000 0.000 0.000  pi/2 0;
             0.000 0.400 0.000 -pi/2 0;
             0.000 0.000 0.000  pi/2 0;
             0.000 0.400 0.000 -pi/2 0;
             0.000 0.000 0.000  pi/2 0;
             0.000 0.126 0.000 0     0];
    q_iiwa=[0,0,0,0,0,0,0];
    qlim_iiwa=[-170, 170; 
               -120, 120; 
               -170, 170; 
               -120, 120; 
               -170, 170; 
               -120, 120; 
               -175, 175]*pi/180;

    figure('name', 'SCARA');
    create_robot(dh_SCARA, 'SCARA', q_SCARA, qlim_SCARA, [0,0,0,0]);

    figure('name', 'Paint');

    create_robot(dh_paint, 'Paint', q_paint, qlim_paint, [0,pi/2,0,0,0,0]);
    
    figure('name', 'iiwa');
    create_robot(dh_iiwa, 'iiwa', q_iiwa, qlim_iiwa, [0,0,0,0,0,0,0]);

end

function create_robot(dh, name, q, qlim, offset)
    R = SerialLink(dh, 'name', name);
    R.offset = offset;
    R.qlim = qlim;
    R.plot(q, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
    R.teach();
end