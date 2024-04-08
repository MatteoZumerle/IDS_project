function [distanza, indice_ostacolo] = trova_ostacolo(posizione_lidar_x, posizione_lidar_y, direzione_x, direzione_y, ostacoli_x, ostacoli_y)
    % Inizializza la distanza minima e l'indice dell'ostacolo
    distanza_minima = Inf;
    indice_ostacolo = [];
    
    % Ciclo su tutti gli ostacoli
    for i = 1:length(ostacoli_x)
        % Calcola il vettore dall'origine del LiDAR all'ostacolo
        vettore_ostacolo = [ostacoli_x(i) - posizione_lidar_x, ostacoli_y(i) - posizione_lidar_y];
        
        % Calcola la distanza dall'origine del LiDAR all'ostacolo
        distanza_ostacolo = norm(vettore_ostacolo);
        
        % Calcola l'angolo tra il vettore dall'origine del LiDAR all'ostacolo e la direzione di scansione del LiDAR
        angolo = atan2d(vettore_ostacolo(2), vettore_ostacolo(1)) - atan2d(direzione_y, direzione_x);
        
        % Se l'ostacolo è nel campo di vista del LiDAR e più vicino della distanza minima
        if abs(angolo) < 30 && distanza_ostacolo < distanza_minima
            distanza_minima = distanza_ostacolo;
            indice_ostacolo = i;
        end
    end
    
    % Restituisce la distanza minima e l'indice dell'ostacolo
    distanza = distanza_minima;
end
