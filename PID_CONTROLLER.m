function [positions, near_pos] = PID_CONTROLLER(points, initial_vel, max_speed, Kp, Ki, Kd, t, dt )
        error_sum=0;
        previous_error = 0;
        final_pos = points(:, length(points));
        % Velocità iniziale e finale desiderate
        
        % Simulazione del controllo PID
        
        positions = []; % registro delle posizioni per la visualizzazione
        near_pos = [];
        
        % Posizione iniziale del punto
        pos = points(:, 1);
        
        % Velocità iniziale del punto
        vel = initial_vel;
        
        % continua finché non si raggiunge la destinazione
        
        proximity_threshold = 0.1;
        figure();
        for i=2:length(points)
        
        % momentaneous target of the drone where it has to go
        final_pos = points(:, i);
        
        while norm(pos - final_pos) > proximity_threshold  
        % Calcolo dell'errore di posizione
        error_pos = final_pos - pos;
        
        % Componente proporzionale
        control_p = Kp * error_pos;
        
        % Componente integrale
        error_sum = error_sum + error_pos * dt;
        control_i = Ki * error_sum;
        
        % Componente derivativa
        derivative = (error_pos - previous_error) / dt;
        control_d = Kd * derivative;
        
        % Controllo finale
        control = control_p + control_i + control_d;
        
        % Limitazione della velocità
        control = min(max(control, -max_speed), max_speed);
        
        % Aggiornamento della velocità e della posizione
        vel = vel + control * dt;
        pos = pos + vel * dt;
        
        % Memorizzazione della posizione
        positions = [positions, pos];
        
        % Aggiornamento dell'errore precedente
        previous_error = error_pos;
        
        t = t + dt; % incremento del tempo
        end
        
        %Visualize the time requested and error
        % t
        % vel(length(vel))
        % pos(length(pos))
        % error_pos(length(error_pos))
        near_pos = [near_pos; positions(:, length(positions))];
        % plot(positions(1, length(positions)), positions(2, length(positions)), 'o', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
        % hold on;
        %plot the end point
        end
end

