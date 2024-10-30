% Cargar el objeto R desde el archivo .mat y no ejecutar robot cada vez que se quiera probar el robot
% Requiere que se ejecute primero una vez robot.m

function herramienta; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS,*NASGU>
    load('kuka_16.mat', 'R','q_kuka_16','path','workspace');

    vector_q=[0,0,0,0,0,0]; %posicion de las articulaciones
    vector_s=[1,1,1,0,0,0,0];%1 si se quiere mostrar el sistema de la articulacion, 0 si no
    figure('name', 'Kuka 16 con stl');
    R.plot3d(q_kuka_16, 'path',path,'notiles', 'nowrist','view',[30,30], 'scale', 0.1);
    R.base = trotz(-45);
    

    
    % figure('name', 'Kuka 16 con teach');
    
    % R.plot(vector_q, 'scale', 0.4, 'trail', {'r', 'LineWidth', 2}, 'workspace', workspace);
    hold on
    [T,all]=R.fkine(q_kuka_16);
    z=eye(4)*trotz(-45);
    for i=1:length(vector_s)
        if vector_s(i)==1
            if i==1
                trplot(z,'length',0.5,'frame','0');
            else
                trplot(all(i-1),'length',0.5,'frame',num2str(i-1));
            end
        end
    end
    hold off
    R.teach('scale',0.001,'trail', {'r', 'LineWidth', 2}, 'workspace', workspace);
end

