<<<<<<< HEAD

function generar_trayectoria_arco_invertido()
    % Cargar los datos del robot desde el archivo .mat
    load('kuka_16.mat','path','R');

    % Definir los puntos extremos del arco invertido
    % Punto inicial (más alejado del robot)
    q_inicio = [0, -pi/4, pi/4, 0, pi/4, 0]; 
    
    % Punto medio (más cercano al robot)
    q_medio = [0, -pi/2, pi/2, 0, pi/2, 0]; 
    
    % Punto final (más alejado del robot)
    q_final = [0, -pi/4, pi/4, 0, pi/4, 0]; 
    
    % Generar la trayectoria con jtraj
    t = linspace(0, 2, 100); % Tiempo de 0 a 2 segundos, 100 puntos
    [q_trayectoria, qd_trayectoria, qdd_trayectoria] = jtraj(q_inicio, q_medio, t/2);
    [q_trayectoria_2, qd_trayectoria_2, qdd_trayectoria_2] = jtraj(q_medio, q_final, t/2);
    
    q_trayectoria_total = [q_trayectoria; q_trayectoria_2]; % Trayectoria completa
    
    % Visualizar el robot y la trayectoria
    figure;
    R.plot3d(q_trayectoria_total, 'path', path, 'notiles', 'nowrist', 'view', [90, 0], 'scale', 0.1);

    title('Trayectoria en Arco Invertido');

end
=======

function generar_trayectoria_arco_invertido()
    % Cargar los datos del robot desde el archivo .mat
    load('kuka_16.mat','path','R');

    % Definir los puntos extremos del arco invertido
    % Punto inicial (más alejado del robot)
    q_inicio = [0, -pi/4, pi/4, 0, pi/4, 0]; 
    
    % Punto medio (más cercano al robot)
    q_medio = [0, -pi/2, pi/2, 0, pi/2, 0]; 
    
    % Punto final (más alejado del robot)
    q_final = [0, -pi/4, pi/4, 0, pi/4, 0]; 
    
    % Generar la trayectoria con jtraj
    t = linspace(0, 2, 100); % Tiempo de 0 a 2 segundos, 100 puntos
    [q_trayectoria, qd_trayectoria, qdd_trayectoria] = jtraj(q_inicio, q_medio, t/2);
    [q_trayectoria_2, qd_trayectoria_2, qdd_trayectoria_2] = jtraj(q_medio, q_final, t/2);
    
    q_trayectoria_total = [q_trayectoria; q_trayectoria_2]; % Trayectoria completa
    
    % Visualizar el robot y la trayectoria
    figure;
    R.plot3d(q_trayectoria_total, 'path', path, 'notiles', 'nowrist', 'view', [90, 0], 'scale', 0.1);

    title('Trayectoria en Arco Invertido');

end
>>>>>>> 5d5cfbc (por favor funciona)
