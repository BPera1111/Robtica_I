function TP2_5 ; clc ; close all; %#ok<*NOPRT>

    TO = eye(4); % Sistema O
 
    oTm_a = TO* transl([0,0,1])*troty(45)
    oTm_b = TO* troty(45)*transl([0,0,1])

    OtM(TO,oTm_a,oTm_b)


end

function OtM(t0,t1,t2)

    title('M t O');
    hold on
    trplot(t0,'color','b','frame','O','length',1) % Sistema de referencia.
    trplot(t1,'color','r','frame','M_a','length',1) % Sistema rotado según matriz de rotación.
    trplot(t2,'color','g','frame','M_b','length',1)
    grid on
    rotate3d on %rotar el grafico
    axis(determinar_limites(t0,t1,t2)) %limites de cada eje
    view(179,20) %angulo de vista inicial
    % plot3(a0(1),a0(2),a0(3), '*k', 'markersize', 20, 'LineWidth', 1)
    hold off

end

function limites = determinar_limites(varargin)
    % varargin permite pasar un número variable de argumentos
    % Cada argumento es una matriz 4x4 donde la submatriz 3x3 es la que contiene los datos
    
    % Inicializar los límites con valores extremos
    min_x = inf; max_x = -inf;
    min_y = inf; max_y = -inf;
    min_z = inf; max_z = -inf;
    
    % Iterar sobre cada conjunto de datos
    for i = 1:nargin
        % Extraer la submatriz de 3x3
        datos = varargin{i}(1:3, 1:3);
        % Convertir la submatriz en un vector de puntos
        puntos = reshape(datos, [], 3);
        min_x = min(min_x, min(puntos(:,1)))-0.5;
        max_x = max(max_x, max(puntos(:,1)))+0.5;
        min_y = min(min_y, min(puntos(:,2)))-0.5;
        max_y = max(max_y, max(puntos(:,2)))+0.5;
        min_z = min(min_z, min(puntos(:,3)))-0.5;
        max_z = max(max_z, max(puntos(:,3)))+0.5;
    end
    
    % Crear un vector con los límites
    limites = [min_x max_x min_y max_y min_z max_z];
end