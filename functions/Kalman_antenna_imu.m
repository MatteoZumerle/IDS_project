function [state_fused,P_fused] = Kalman_antenna_imu(Drone,drone_name,count)
    
    
    z1 = [Drone.(drone_name).RRT.all_positions(count,1); Drone.(drone_name).RRT.all_positions(count,2)]; % Antenna measure (PID)
    R1 = [Drone.(drone_name).PID.antenna_noise, 0; 0, Drone.(drone_name).PID.antenna_noise]; % Antenna's covariance matrix
    
    z2 = [Drone.(drone_name).RRT.all_positions_drift(count,1); Drone.(drone_name).RRT.all_positions_drift(count,2)]; % Imu position measure
    R2 = [Drone.(drone_name).RRT.imu_noise, 0; 0, Drone.(drone_name).RRT.imu_noise]; % Imu covariance matrix
    
    state = [Drone.(drone_name).RRT.all_positions(count-1,1); Drone.(drone_name).RRT.all_positions(count-1,2);...
             Drone.(drone_name).RRT.vel(1,1); Drone.(drone_name).RRT.vel(2,1)]; % Initial state  [x; y; vx; vy]
    P = Drone.(drone_name).P_matrix; % Starting covariance matrix
  
    H1 = [1 0 0 0; 0 1 0 0]; % Antenna observation matrix
    H2 = [1 0 0 0; 0 1 0 0]; % IMU observation matrix
    
    % Updating Antenna state
    K1 = P * H1' / (H1 * P * H1' + R1);
    state1 = state + K1 * (z1 - H1 * state);
    P1 = (eye(4) - K1 * H1) * P;
    
    % Updating IMU state
    K2 = P * H2' / (H2 * P * H2' + R2);
    state2 = state + K2 * (z2 - H2 * state);
    P2 = (eye(4) - K2 * H2) * P;
    
    % New covariance matrices
    P_fused_inv = inv(P1) + inv(P2);
    P_fused = inv(P_fused_inv);
    
    % Weighted mean for the new state
    state_fused = P_fused * (inv(P1) * state1 + inv(P2) * state2);
        
end