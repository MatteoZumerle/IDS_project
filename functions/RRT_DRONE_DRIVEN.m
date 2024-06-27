function [rrt_tree, parent_indices] = RRT_DRONE_DRIVEN(step_size,n_iterations, fov_drone, lidar_data, inflate, prev_points)
      
    rrt_tree = [0 , 0]; % RRT starting point
    parent_indices = 0; % RRT parent index

    inflated_obstacles_lidar = lidar_data_to_obstacles(lidar_data, inflate); % to see the inflated obstacles aroud the drone: taken from the lidar map
    
    % RRT tree generation
    for i = 2:n_iterations % from 2-> starting point at the moment is given

        % Calculate the com of the previous 2 point buffer to move in other
        % quadrants: find the coordinates relatively to the position

        rel_prev_pts = [];
        [num_prev_points,~] =size(prev_points(:,1)); 
        rel_prev_pts(1,:) = -prev_points(1, :);
        
        for j=2:num_prev_points
            rel_prev_pts(j,:) = rel_prev_pts(j-1,:) - prev_points(j,:);                             
        end
        
        x_com = mean(rel_prev_pts(:,1));
        y_com = mean(rel_prev_pts(:,2));
        
        
        % If point is inside the obstacles, avoid
        flag1 = true;
        flag2 = true;
        while flag1 == true || flag2 == true
    
            % Select the random point in the free field of view of the
            % drone: If the point is in the quadrant to avoid, calculate a new point
            random_point = [randn * fov_drone, randn * fov_drone];
            while sign(random_point(1, 1)) == sign(x_com) && sign(random_point(1, 2)) == sign(y_com)
                random_point = [randn * fov_drone, randn * fov_drone];
            end
            
            % If the selected point is dropped in a wall or behind
            if isInObstacle_rel(random_point, inflated_obstacles_lidar) 
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

