function [rrt_tree, parent_indices, rho_home, inflated_obstacles_lidar] = RRT_DRONE_TO_HOME(step_size, home_position_rel, lidar_data, inflate,home_treshold, x_map_size, y_map_size)
      
    rrt_tree = [0 , 0]; % RRT starting point
    parent_indices = 0; % RRT parent index

    inflated_obstacles_lidar = lidar_data_to_obstacles(lidar_data, inflate); % to see the inflated obstacles around the drone: taken from the lidar map
    
    % Flag to interrupt the while iteration
    flag_home = 0;
    count_iterations = 0;


    % Defining the region for directed sampling
    sampling_radius = norm(home_position_rel) * 1.2; % Initial sampling radius

    % RRT tree generation
    while flag_home == 0 
        count_iterations = count_iterations + 1;

        % If point is inside the obstacles, avoid
        flag1 = true;
        flag2 = true;
        while flag1 == true || flag2 == true
    
            
            % Define the shifted square sampling region
            random_point = (rand(1,2) - 0.5) * 2 .* sampling_radius;
            random_point = random_point + (home_position_rel); % Shift the sampling region towards the goal
            

            if isInObstacle_rel(random_point, inflated_obstacles_lidar) % if the selected point is dropped in a wall or behind
                flag1 = true;
            else
                flag1 = false;
            end
    
            % Find nearest point wrt RRT tree
            distances = sqrt(sum((rrt_tree - random_point).^2, 2));
            [ ~ , nearest_idx] = min(distances);
    
            % Identify new RRT point
            direction = random_point - rrt_tree(nearest_idx, :);
            unit_direction = direction / norm(direction);
            new_point = rrt_tree(nearest_idx, :) + step_size * unit_direction;
    
            % Check if the new point is inside the obstacles
            if isInObstacle_rel(new_point, inflated_obstacles_lidar)
                flag2 = true;
            else
                flag2 = false;
            end
        end
    
        % Add the new point to the RRT tree
        rrt_tree = [rrt_tree; new_point];       
        parent_indices = [parent_indices; nearest_idx];

        rho = norm(new_point - home_position_rel);
        
        % Check if Drone has reached home neighborhood
        if rho <= home_treshold % 0.7 meter
            flag_home = 1;
            rho_home = rho;
        end

        % Adjust sampling radius as we approach the goal
        sampling_radius = norm(home_position_rel - rrt_tree(end,:)) * 1.2;
    end
end

