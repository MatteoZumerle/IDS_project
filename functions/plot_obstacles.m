function plot_obstacles(lidar_data, obstacles)

    % Lidar scanned points
    scatter(lidar_data(:, 1), lidar_data(:, 2), 'k', 'filled');
    hold on;

    % Plot square obstacles
    num_obstacles = size(obstacles, 1);
    for i = 1:num_obstacles
        % Obstacle coordinates extraction
        bottom_left = obstacles(i, 1:2);
        top_right = obstacles(i, 3:4);
        bottom_right = [top_right(1), bottom_left(2)];
        top_left = [bottom_left(1), top_right(2)];

        % Plot the squared obstacle
        square_vertices = [bottom_left; bottom_right; top_right; top_left; bottom_left];
        plot(square_vertices(:, 1), square_vertices(:, 2), 'r', 'LineWidth', 1);
        hold on;
    end

    hold off;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title('Lidar Points and Inflated Obstacles');
    grid on;
end


