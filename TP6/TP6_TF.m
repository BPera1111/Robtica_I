function TP6_TF; clc; clear ; close all; %#ok<*NOPRT,*CLEAR0ARGS,*NOPTS,*NASGU,*MINV>
    disp("Cargando el robot")
    load('kuka_16.mat', 'R','q_kuka_16','path','workspace');

    

    disp("Confirmando singularidades");
    
    % disp("Singularidades por Muñeca")
    % Muneca(R, 4,path);
    close all;
    disp("Singularidades por Codo")
    codo(R, 1000,path);
    close all;


end



function Muneca(R,iter,path)
    q = [];
    qlim_1 = R.qlim(1, 1:2);
    qlim_2 = R.qlim(2, 1:2);
    qlim_3 = R.qlim(3, 1:2);
    qlim_4 = R.qlim(4, 1:2);
    % qlim_5 = R.qlim(5, 1:2);
    qlim_6 = R.qlim(6, 1:2);
    i2=0;
    for i=1:1:iter
        rng('shuffle');
        q1 = qlim_1(1) + (qlim_1(2)-qlim_1(1))*rand;
        q2 = qlim_2(1)+(qlim_2(2)-qlim_2(1))*rand;
        q3 = qlim_3(1)+(qlim_3(2)-qlim_3(1))*rand;
        % q3 = -pi/2;
        q4 = qlim_4(1)+(qlim_4(2)-qlim_4(1))*rand;
        % q5 = qlim_5(1)+(qlim_5(2)-qlim_5(1))*rand;
        q5 = 0;
        q6 = qlim_6(1)+(qlim_6(2)-qlim_6(1))*rand;
        
        qi = [q1,q2,q3,q4,q5,q6];
        deter =det(R.jacob0(qi));
        if abs(deter)<1e-5
            i2 = i2 + 1;
            q(:,i2) = qi';
        end
    end 
    disp("Singularidades encontradas")
    disp(q)
    num_singularities = size(q, 2);
    disp(['Número de singularidades de muñeca encontradas: ', num2str(num_singularities)]);
    figure;
    for i = 1:num_singularities
        clf; % Limpia la figura para mostrar la siguiente configuración
        title(['Singularidad Muñeca ', num2str(i)]);
        
        % Visualizar la configuración actual
        R.plot3d(q(:, i)', 'path', path, 'notiles', 'nowrist', 'view', [30, 30], 'scale', 0.1);
        
        % Crear botón para avanzar a la siguiente configuración
        uicontrol('Style', 'pushbutton', 'String', 'Siguiente', ...
                  'Position', [20 20 80 40], ...
                  'Callback', 'uiresume(gcbf)');
        
        % Esperar hasta que el usuario presione el botón
        uiwait(gcf);
    end

end

function codo(R,iter,path)
    q = [];
    qlim_1 = R.qlim(1, 1:2);
    qlim_2 = R.qlim(2, 1:2);
    % qlim_3 = R.qlim(3, 1:2);
    qlim_4 = R.qlim(4, 1:2);
    qlim_5 = R.qlim(5, 1:2);
    qlim_6 = R.qlim(6, 1:2);
    i2=0;
    for i=1:1:iter
        rng('shuffle');
        q1 = qlim_1(1) + (qlim_1(2)-qlim_1(1))*rand;
        q2 = qlim_2(1)+(qlim_2(2)-qlim_2(1))*rand;
        % q3 = qlim_3(1)+(qlim_3(2)-qlim_3(1))*rand;
        q3 = -1.45+((-1.65)-(-1.45))*rand;
        % q3 = -pi/2;
        % q3 = -1.6229;
        q4 = qlim_4(1)+(qlim_4(2)-qlim_4(1))*rand;
        % q5 = qlim_5(1)+(qlim_5(2)-qlim_5(1))*rand;
        q5 = pi/4;
        q6 = qlim_6(1)+(qlim_6(2)-qlim_6(1))*rand;
        
        qi = [q1,q2,q3,q4,q5,q6];
        deter =det(R.jacob0(qi));
        if abs(deter)<1e-5
            i2 = i2 + 1;
            q(:,i2) = qi';
        end
    end 
    disp("Singularidades encontradas")
    disp(q)
    num_singularities = size(q, 2);
    disp(['Número de singularidades de codo encontradas: ', num2str(num_singularities)]);

    figure;
    for i = 1:num_singularities
        clf; % Limpia la figura para mostrar la siguiente configuración
        title(['Singularidad Codo ', num2str(i)]);
        
        % Visualizar la configuración actual
        R.plot3d(q(:, i)', 'path', path, 'notiles', 'nowrist', 'view', [30, 30], 'scale', 0.1);
        
        % Crear botón para avanzar a la siguiente configuración
        uicontrol('Style', 'pushbutton', 'String', 'Siguiente', ...
                  'Position', [20 20 80 40], ...
                  'Callback', 'uiresume(gcbf)');
        
        % Esperar hasta que el usuario presione el botón
        uiwait(gcf);
    end

end

function q = Singularidades(R,iter)
    q = [];
    qlim_1 = R.qlim(1, 1:2);
    qlim_2 = R.qlim(2, 1:2);
    qlim_3 = R.qlim(3, 1:2);
    qlim_4 = R.qlim(4, 1:2);
    qlim_5 = R.qlim(5, 1:2);
    qlim_6 = R.qlim(6, 1:2);
    i2=0;
    for i=1:1:iter
        q1 = qlim_1(1) + (qlim_1(2)-qlim_1(1))*rand;
        q2 = qlim_2(1)+(qlim_2(2)-qlim_2(1))*rand;
        q3 = qlim_3(1)+(qlim_3(2)-qlim_3(1))*rand;
        % q3 = -pi/2;
        q4 = qlim_4(1)+(qlim_4(2)-qlim_4(1))*rand;
        q5 = qlim_5(1)+(qlim_5(2)-qlim_5(1))*rand;
        %q5 = 0;
        q6 = qlim_6(1)+(qlim_6(2)-qlim_6(1))*rand;
        
        qi = [q1,q2,q3,q4,q5,q6];
        deter =det(R.jacob0(qi));
        if abs(deter)<1e-5
            i2 = i2 + 1;
            q(:,i2) = qi';
        end
    end 
end

function q_singulares = SingularidadesExhaustiva(R, step)
    % Inicializamos q_singulares como una matriz vacía
    q_singulares = [];

    % Extraemos los límites de las articulaciones
    qlim_1 = R.qlim(1, 1:2);
    qlim_2 = R.qlim(2, 1:2);
    qlim_3 = R.qlim(3, 1:2);
    qlim_4 = R.qlim(4, 1:2);
    qlim_5 = R.qlim(5, 1:2);
    qlim_6 = R.qlim(6, 1:2);

    % Generamos los rangos de valores con el paso definido
    q1_vals = qlim_1(1):step:qlim_1(2);
    q2_vals = qlim_2(1):step:qlim_2(2);
    q3_vals = qlim_3(1):step:qlim_3(2);
    q4_vals = qlim_4(1):step:qlim_4(2);
    q5_vals = qlim_5(1):step:qlim_5(2);
    q6_vals = qlim_6(1):step:qlim_6(2);

    % Contador para el número de singularidades encontradas
    i2 = 0;

    % Bucle anidado para iterar sobre todas las combinaciones de valores posibles
    for q2 = q2_vals
        for q3 = q3_vals
            for q4 = q4_vals
                for q5 = q5_vals
                    % Configuración actual
                    qi = [0, q2, q3, q4, q5, 0]

                    % Calculamos el determinante del Jacobiano en la configuración qi
                    deter = det(R.jacob0(qi));
                    % Si el determinante es cercano a cero, consideramos una singularidad
                    if abs(deter) < 1e-4
                        i2 = i2 + 1;
                        q_singulares(:, i2) = qi';  % Guardamos la configuración singular
                    end
                end
            end
        end
    end 
    % Mostrar la cantidad de singularidades encontradas
    disp(['Número de singularidades encontradas: ', num2str(i2)]);
end
