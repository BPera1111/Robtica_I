% Robótica 1.
% Corazza - Masi - Suarez.
%
close all; clc; clear all;
[R1,R2] = robot();
%
q = [0,30,0,0,0,0]*pi/180; % Se propone este vector articular a modo de ejemplo.
T1 = R1.fkine(q).double; % Matriz T1 dato.
T2 = R2.fkine(q).double; % Matriz T2 dato.
%
q0 = [0,0,0,0,0,0]*pi/180;
%
qsol1 = CinInv(T1,R1,q0,0);
qsol2 = CinInv(T2,R2,q0,0);
%
% Corroboración
%
T1p = R1.fkine(qsol1).double;
disp(T1)
disp(T1p)
T2p = R2.fkine(qsol2).double;
disp(T2)
disp(T2p)