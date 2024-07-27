function [rrt_tree, parent_indices,rho_home,inflated_obstacles_lidar] = RRT_DRONE_TO_HOME(step_size,home_position_rel,lidar_data, inflate, home_threshold, x_map_size,y_map_size)
      
    rrt_tree = [0 , 0]; % RRT starting point
    parent_indices = 0; % RRT parent index

    inflated_obstacles_lidar = lidar_data_to_obstacles(lidar_data, inflate); % In order to see the inflated obstacles aroud the drone: taken from the lidar map
    
    %Flag to interrupt the while iteration
    flag_home = 0;
    count_iterations = 0;

    % RRT tree generation
    while flag_home == 0 
        count_iterations = count_iterations +1;

        % If point is inside the obstacles, avoid
        flag1 = true;
        flag2 = true;
        while flag1 == true || flag2 == true
    
            % Select the random point in the free field of view of the drone
            random_point = [(2 * randi([0, 1]) - 1) * rand * x_map_size, (2 * randi([0, 1]) - 1) * rand * y_map_size];
            

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
    
            % Check compenetrarion of new point wrt the obstacles
            if isInObstacle_rel(new_point, inflated_obstacles_lidar)
                flag2 = true;
            else
                flag2 = false;
            end
    
        end
    
        % Add the new point to the RRT tree
        rrt_tree = [rrt_tree; new_point];       
        parent_indices = [parent_indices; nearest_idx];

        %distance from rrt point generation and home position in relative
        %reference frame
        rho = sqrt(sum((new_point - home_position_rel).^2, 2));
        
        %Check if Drone has reached home neighborhood
        if rho <= home_threshold 
            flag_home = 1;
            rho_home = rho;
        end
    
 
    end

end

