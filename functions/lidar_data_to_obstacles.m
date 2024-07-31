function obstacles = lidar_data_to_obstacles(lidar_data, buffer_distance)
    % Inizializza la matrice per gli ostacoli
    num_points = size(lidar_data, 1);
    obstacles = zeros(num_points, 4); % Ogni riga conterrà le coordinate di due punti (basso sinistra e alto destro)

    % Calcola il buffer quadrato attorno a ciascun punto
    for i = 1:num_points
        point = lidar_data(i, :);
        square = calculate_square(point, buffer_distance);
        obstacles(i, :) = square; % Salva le coordinate dei due punti
    end
end

function square = calculate_square(point, buffer_distance)
    % Calcola le coordinate del quadrato attorno al punto
    x = point(1);
    y = point(2);
    half_side = buffer_distance / 2;

    % Calcola i punti basso sinistra e alto destro del quadrato
    bottom_left = [x - half_side, y - half_side];
    top_right = [x + half_side, y + half_side];

    % Salva le coordinate dei due punti
    square = [bottom_left, top_right];
end
