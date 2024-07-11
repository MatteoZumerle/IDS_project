function [] = PLOT_MAP_LITE(obstacles)

    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i, :), 'FaceColor', '#0072BD');
    end
    
end

