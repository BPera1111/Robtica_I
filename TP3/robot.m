function robot; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS>
    
    dh_kuka_16=[0.000 0.675 0.260   pi/2  0;
                0.000 0.000 0.680   0     0;
                0.000 0.000 0.035   pi/2  0;
                0.000 0.670 0.000  -pi/2  0;
                0.000 0.000 0.000   pi/2  0;
                0.000 0.158 0.000   0     0];

    q_kuka_16=[0,0,0,0,0,0];

    qlim_kuka_16=[-185, 185; 
                  -155  ,35; 
                  -130, 154; 
                  -350, 350; 
                  -130, 130; 
                  -350, 350]*pi/180;
    offet = [0,pi/2,0,0,0,0];

    base = transl(-1,1,0) * trotz(-45);
    path = fullfile(pwd,'STL','KR16_arc_HW');
    % figure('name', 'Kuka 16');
    R=create_robot(dh_kuka_16, 'Kuka 16', q_kuka_16, qlim_kuka_16, offet, base,path);
    save('kuka_16.mat', 'R',"q_kuka_16","qlim_kuka_16","offet","base","path");

    
    
end

function R =create_robot(dh, name, q, qlim, offset,base,path)
    R = SerialLink(dh, 'name', name);
    R.base = base;
    % R.tool = transl(.2, .2, 0);
    R.offset = offset;
    R.qlim = qlim;
    % R.plot(q, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2});
    % R.teach();
    % R.plot3d(q, 'path',path, 'nowrist', 'noarrow', 'view', [-30 30], 'delay', 0.01);
end