function [rrt_tree, parent_indices] = RRT_DRONE(step_size,n_iterations, fov_drone, lidar_data, inflate)
      
    rrt_tree = [0 , 0]; % RRT starting point
    parent_indices = 0; % RRT parent index

    inflated_obstacles_lidar = lidar_data_to_obstacles(lidar_data, inflate); % to see the inflated obstacles aroud the drone: taken from the lidar map
    
    % RRT tree generation
    for i = 2:n_iterations % from 2-> starting point at the moment is given
        % If point is inside the obstacles, avoid
    
        flag1 = true;
        flag2 = true;
        while flag1 == true || flag2 == true
    
            % Select the random point in the free field of view of the drone
            random_point = [randn * fov_drone, randn * fov_drone];
            

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
    
 
    end
end

