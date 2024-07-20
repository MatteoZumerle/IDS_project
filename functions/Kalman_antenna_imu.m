function [state_fused,P_fused] = Kalman_antenna_imu(Drone,drone_name,count)
    
    % Dati di misura del sensore 1
    z1 = [Drone.(drone_name).RRT.all_positions(count,1); Drone.(drone_name).RRT.all_positions(count,2)]; % posizione misurata da Antenna (PID)
    R1 = [Drone.(drone_name).PID.antenna_noise, 0; 0, Drone.(drone_name).PID.antenna_noise]; % matrice di covarianza del sensore 1
    
    % Dati di misura del sensore 2
    z2 = [Drone.(drone_name).RRT.all_positions_drift(count,1); Drone.(drone_name).RRT.all_positions_drift(count,2)]; % posizione misurata dal sensore 2
    R2 = [Drone.(drone_name).RRT.imu_noise, 0; 0, Drone.(drone_name).RRT.imu_noise]; % matrice di covarianza del sensore 2
    
    % Inizializzazione delle variabili del filtro di Kalman
    state = [Drone.(drone_name).RRT.all_positions(count-1,1); Drone.(drone_name).RRT.all_positions(count-1,2); Drone.(drone_name).RRT.vel(1,1); Drone.(drone_name).RRT.vel(2,1)]; % stato iniziale [x; y; vx; vy]
    P = Drone.(drone_name).P_matrix; % matrice di covarianza iniziale
    Q = eye(4) * 0.01; % rumore di processo
    
    % Matrici di osservazione per i due sensori
    H1 = [1 0 0 0; 0 1 0 0]; % matrice di osservazione per il sensore 1
    H2 = [1 0 0 0; 0 1 0 0]; % matrice di osservazione per il sensore 2
    
    % Filtro di Kalman: Aggiornamento con il sensore 1
    K1 = P * H1' / (H1 * P * H1' + R1);
    state1 = state + K1 * (z1 - H1 * state);
    P1 = (eye(4) - K1 * H1) * P;
    
    % Filtro di Kalman: Aggiornamento con il sensore 2
    K2 = P * H2' / (H2 * P * H2' + R2);
    state2 = state + K2 * (z2 - H2 * state);
    P2 = (eye(4) - K2 * H2) * P;
    
    % Stato stimato fuso
    state_fused = (state1 + state2) / 2; % Media dei due stati stimati
    P_fused = (P1 + P2) / 2; % Media delle due matrici di covarianza
        
end