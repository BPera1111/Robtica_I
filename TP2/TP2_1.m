function TP2_1

    %% Ejercicio1: Grafique el sistema {𝑀} respecto de {𝑂} 
    %% para cada una de  las siguientes matrices de rotación:

    Ta= [ 0.5   -0.866 0 0 
          0.866  0.5   0 0
          0      0     1 0
          0      0     0 1 ]
    
    Tb= [   0   0   1   0 
            -1  0   0   0   
            0   -1  0   0   
            0   0   0   1    ]

    Tc= [   0.5    -0.75   -0.433   0
            0.866   0.433   0.25    0
            0      -0.5     0.866   0
            0       0       0       1 ]

    T0 = eye(4) %Matriz identidad de 4x4


    % Ploteo de los sistemas rotados.
    figure
    OtM(T0,Ta)
    figure
    OtM(T0,Tb)
    figure
    MtO(T0,Tc)



    
    

end


function OtM(t0,t1)

    title('O t M');
    hold on
    trplot(t0,'color','b','frame','O','length',1) % Sistema de referencia.
    trplot(t1,'color','r','frame','M','length',1) % Sistema rotado según matriz de rotación.
    grid on
    rotate3d on %rotar el grafico
    axis([-1 2 -1 2 -1 2]) %limites de cada eje
    view(179,20) %angulo de vista inicial
    hold off

end


function MtO(t0,t1)

    title('M t O');
    hold on
    trplot(t0,'color','b','frame','M','length',1) % Sistema de referencia.
    trplot(t1,'color','r','frame','O','length',1) % Sistema rotado según matriz de rotación.
    grid on
    rotate3d on %rotar el grafico
    axis([-1 2 -1 2 -1 2]) %limites de cada eje
    view(179,20) %angulo de vista inicial
    hold off

end