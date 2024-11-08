function robot2; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS,*INUSD>
    
    dh_kuka_16_2=[0.000 0.675 0.260  -pi/2  0;
                0.000 0.000 0.680   0     0;
                0.000 0.000 0.035   -pi/2  0;
                0.000 0.670 0.000  pi/2  0;
                0.000 0.000 0.000   -pi/2  0;
                0.000 0.115 0.000   0     0];

    q_kuka_16_2=[0,0,0,0,0,0];

    qlim_kuka_16_2=[-185, 185; 
                  -65  ,125; 
                  -220, 64; 
                  -350, 350; 
                  -130, 130; 
                  -350, 350]*pi/180;
    offet_2 = [0,-pi/2,0,0,0,0];

    base_2 = transl(0,2.5,0)*trotz(-45);
    %base = eye(4);
    path_2 = fullfile(pwd,'STL','KR16_2');
    % figure('name', 'Kuka 16');
    workspace_2 = [-2 2 -1 4 -1 3];
    R2=create_robot(dh_kuka_16_2, 'Kuka 16', q_kuka_16_2, qlim_kuka_16_2, offet_2, base_2,path_2,workspace_2);
    save('kuka_16_2.mat', 'R2',"q_kuka_16_2","qlim_kuka_16_2","offet_2","base_2","path_2","workspace_2","dh_kuka_16_2");

    
    
end

function R =create_robot(dh, name, q, qlim, offset,base,path,workspace) 
    R = SerialLink(dh, 'name', name);
    R.base = base;
    %Agregar tool
    % R.tool = transl(.2, .2, 0);
    %R.tool = transl(0,0,.2)*trotz(pi/2);
    R.offset = offset;
    R.qlim = qlim;
    %R.plot(q, 'scale', 0.8, 'trail', {'r', 'LineWidth', 2}, 'workspace', workspace);
    % R.teach();
    % R.plot3d(q, 'path',path, 'nowrist', 'noarrow', 'view', [-30 30], 'delay', 0.01);
end