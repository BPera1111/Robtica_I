function esp_trabajo; clc; clear; close all;
    load('kuka_16.mat', 'R','q_kuka_16','path','workspace','dh_kuka_16');
    rango= R.qlim(1, 1:2)*180/pi;
    coord_xy = zeros(2, length(rango(1):5:rango(2))+2);
    cont = 2;
    for q = rango(1):5:rango(2)
        q_kuka_16(1)=q*pi/180;
        T = R.fkine(q_kuka_16);
        T = T.T;
        coord_xy(:,cont) = [T(1:2,4)];
        cont = cont + 1;
    end
    x=[0 3];
    figure()
    plot(x, zeros(length(x)), 'b--');
    hold on
    plot(coord_xy(1,:), coord_xy(2,:));
    xlim([-3 3]);
    ylim([-3 3]);
    axis equal
    grid on
    title('Espacio de trabajo - Vista superior')
    q_kuka_16(1) = 0;
    rango_q2 = R.qlim(2, 1:2)*180/pi;
    rango_q3 = R.qlim(3, 1:2)*180/pi;
    coord_xz = [0;0];
    for i = rango_q2(1):5:rango_q2(2)
        for j = rango_q3(1):5:rango_q3(2)
            q_kuka_16(2:3) = [i j]*pi/180;
            T = R.fkine(q_kuka_16);
            T = T.T;
            coord_xz(:,end+1) = T([1 3],4);
        end
    end
    figure()
    plot(coord_xz(1,:), coord_xz(2,:), 'r*')
    grid on
    xlim([-2 3]);
    ylim([-2 3]);
    axis equal
    title('Espacio de trabajo - Vista lateral')
end