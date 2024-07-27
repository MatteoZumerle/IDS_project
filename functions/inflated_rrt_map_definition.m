function [obstacles,inflated_obstacles] = inflated_rrt_map_definition(rrt_inflated)

% Define the walls and obstacles in the map
% Comment the unwanted map type
% Map 1
obstacles = [0, 0, 10, 0.1;     %
             0, 0, 0.1, 10;     % these are the external walls
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
             6, 4, 0.1, 3;];    % [x, y, width, higth]

% Inflation factor applied to the obstacles
inflate = rrt_inflated;
inflated_obstacles = zeros(size(obstacles));
for i = 1:height(obstacles)
    inflated_obstacles(i, 1) = obstacles(i, 1) - inflate; % initial x left switched
    inflated_obstacles(i, 2) = obstacles(i, 2) - inflate; % initial y bottom switched
    inflated_obstacles(i, 3) = obstacles(i, 3) + 2*inflate; % inflated width
    inflated_obstacles(i, 4) = obstacles(i, 4) + 2*inflate; % inflated hight
end

% Map 2
% ...
end

