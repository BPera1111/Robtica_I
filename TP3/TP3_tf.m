function TP3_tf; clc; clear; close all; %#ok<*NOPTS>
    
    dh_kuka_16=[0.000 0.675 0.260   pi/2  0;
                0.000 0.000 0.680   0     0;
                0.000 0.000 0.035   pi/2  0;
                0.000 0.670 0.000  -pi/2  0;
                0.000 0.000 0.000   pi/2  0;
                0.000 0.000 0.000   0     0];

    q_kuka_16=[0,0,0,0,0,0];

    qlim_kuka_16=[-92.5, 92.5; 
                  -155, 35; 
                  -130, 154; 
                  -175, 175; 
                  -65, 65; 
                  -175, 175]*pi/180;

    figure('name', 'Kuka 16');
    create_robot(dh_kuka_16, 'Kuka 16', q_kuka_16, qlim_kuka_16, [0,pi/2,0,0,0,0]);
end

function create_robot(dh, name, q, qlim, offset)
    R = SerialLink(dh, 'name', name);
    R.offset = offset;
    R.qlim = qlim;
    R.plot(q, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
    R.teach();
end