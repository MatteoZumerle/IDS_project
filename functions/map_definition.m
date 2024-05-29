function [obstacles,inflated_obstacles] = map_definition()
%Define the map dimensions, walls and obstacles
% Define max map dimensions
%x_map_size = 10;
%y_map_size = 10;


% Define the walls and obstacles in the map
% Comment the unwanted map type
% Map 1
obstacles = [0, 0, 10, 0.1;     %
             0, 0, 0.1, 10;     % these are the walls
             10, 0, 0.1, 10.1;  %
             0, 10, 10, 0.1;    %
             2.3, 1.5, 0.1, 3;
             2.3, 4.5, 2, 0.1;
             4.3, 4.5, 0.1, 1.7;
             1.4, 6.2, 3, 0.1;
             1.4, 6.3, 0.1, 2.4;
             4, 1.5, 4.5, 0.1;
             5.2, 3, 2.8, 0.1;
             5, 8.5, 3, 0.1;
             7.9, 4, 0.1, 4.5;
             6, 4, 0.1, 3;];    % [x, y, larghezza, altezza]

% Iinflation factor applied to the obstacles
inflate = 0.2;
inflated_obstacles = zeros(size(obstacles));
for i = 1:height(obstacles)
    inflated_obstacles(i, 1) = obstacles(i, 1) - inflate; % x iniziale spostato a sinistra
    inflated_obstacles(i, 2) = obstacles(i, 2) - inflate; % y iniziale spostato in basso
    inflated_obstacles(i, 3) = obstacles(i, 3) + 2*inflate; % larghezza aumentata
    inflated_obstacles(i, 4) = obstacles(i, 4) + 2*inflate; % altezza aumentata
end

% Map 2
% ...
end

