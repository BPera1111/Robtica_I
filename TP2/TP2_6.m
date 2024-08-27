function TP2_6; clc ; close all; 
    %#ok<*NOPRT> 
    %#ok<*MINV>

    TO = eye(4); % Sistema O

    oTm_a = TO* transl([0,0,1])*troty(45) 
    oTm_b = TO* troty(45)*transl([0,0,1])

    a = [0.5 ; 0 ; 1 ; 1]; % Punto a respecto de M

    aM_a = inv(oTm_a)*a 
    aM_b = inv(oTm_b)*a 

    figure('Name', "6 a")
    OtM(TO,oTm_a,aM_a,'Punto a respecto de M a')

    figure('Name', "6 b")
    OtM(TO,oTm_b,aM_b,'Punto a respecto de M b')

    

end

function OtM(t0,t1,a,plotTitle)

    title(plotTitle);
    hold on
    trplot(t0,'color','b','frame','O','length',1) % Sistema de referencia.
    trplot(t1,'color','r','frame','M','length',1) % Sistema rotado según matriz de rotación.
    grid on
    rotate3d on %rotar el grafico
    axis(determinar_limites(t0,t1)) %limites de cada eje
    view(179,20) %angulo de vista inicial
    plot3(a(1),a(2),a(3), '*k', 'markersize', 20, 'LineWidth', 1)
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