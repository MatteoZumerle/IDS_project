function [] = PLOT_MAP(obstacles,inflated_obstacles, x_map_size, y_map_size)

    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i, :), 'FaceColor', '#0072BD');
    end
    
    %Taking in to account inflated obstacles
    hold on;
    for i = 1:size(inflated_obstacles, 1)
        rectangle('Position', inflated_obstacles(i, :), 'EdgeColor', 'k', 'LineStyle', '--');
    end

    hold off;

end

