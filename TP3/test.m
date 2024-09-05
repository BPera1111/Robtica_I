% Cargar el objeto R desde el archivo .mat

function test; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS>
    load('kuka_16.mat', 'R','q_kuka_16','path');


    % figure('name', 'Kuka 16 con stl');
    % R.plot3d(q_kuka_16, 'path',path, 'nowrist','view',[30,30], 'scale', 0.1);
    R.base = trotz(-45);

    figure('name', 'Kuka 16 con teach');
    R.plot(q_kuka_16, 'scale', 0.4, 'trail', {'r', 'LineWidth', 2});
    R.teach();
end

