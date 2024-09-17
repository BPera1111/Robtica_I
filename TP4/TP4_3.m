function TP4_3; clc; clear ; close all; %#ok<*CLEAR0ARGS,*NOPTS,*NASGU>

    dh = [  0      0.45   0.075 -pi/2  0;
            0      0      0.3    0     0;
            0      0      0.075 -pi/2  0;
            0      0.32   0      pi/2  0;
            0      0      0     -pi/2  0;
            0      0.008  0      0     0];

    R = SerialLink(dh);

    q1 = [0,0,0,0,0,0];
    q2 = [pi/4,-pi/2,0,0,0,0];
    q3 = [pi/5,-2*pi/5,-pi/10,pi/2,3*pi/10,-pi/2];
    q4 = [-.61 -.15 -.3 1.4 1.9 -1.4];

    R.links(6).A(q1(6))




end