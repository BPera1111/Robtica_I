function TP2_4 ; clc ; close all %#ok<*NOPRT>

    T0 = eye(4); % Sistema O
    aM = [2 ; 1 ; 0 ; 1]; % Punto a respecto de M
    tita = -atan2(aM(2),aM(1)) * 180/pi % Angúlo de rotacíon del sistema M respecto de O
    disp('La matriz de transformación homogénea del Sistema {M} respecto de {O} es:')
    T = transl([7,4,0])*trotz(tita) 
    a0 = T*aM; % Punto a respecto de O
    OtM(T0,T,a0)
end

function OtM(t0,t1,a0)

    title('M t O');
    hold on
    trplot(t0,'color','b','frame','O','length',1) % Sistema de referencia.
    trplot(t1,'color','r','frame','M','length',1) % Sistema rotado según matriz de rotación.
    grid on
    rotate3d on %rotar el grafico
    axis([-2 10 -2 6 -1 1]) %limites de cada eje
    view(0,90) %angulo de vista inicial
    plot3(a0(1),a0(2),a0(3), '*k', 'markersize', 20, 'LineWidth', 1)
    hold off

end