function obstacles = lidar_data_to_obstacles(lidar_data, buffer_distance)
    
% Initializing obstacle matrix
    num_points = size(lidar_data, 1);
    obstacles = zeros(num_points, 4); % Each row will keep left bottom and right top obstacle points

    % Calcolating rectangle region for obstacle
    for i = 1:num_points
        point = lidar_data(i, :);
        square = calculate_square(point, buffer_distance);
        obstacles(i, :) = square; % Saving coordinates in obstacle matrix
    end
end

function square = calculate_square(point, buffer_distance)
    % rectangle coordinates around scanned point
    x = point(1);
    y = point(2);
    half_side = buffer_distance / 2;

    % Calcolating left bottom and right top obstacle points
    bottom_left = [x - half_side, y - half_side];
    top_right = [x + half_side, y + half_side];

    square = [bottom_left, top_right];
end
