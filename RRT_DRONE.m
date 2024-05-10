function [rrt_tree, parent_indices] = RRT_DRONE(step_size,n_iterations, inflated_obstacles, goal_threshold,  x_map_size,  y_map_size)

    % RRT initialization
    
    start = [0, 0];
    
            
    rrt_tree = start; % RRT starting point
    parent_indices = 0; % RRT parent index
    
    % RRT tree generation
    for i = 2:n_iterations % from 2-> starting point at the moment is given
        % If point is inside the obstacles, avoid
    
        flag1 = true;
        flag2 = true;
        while flag1 == true || flag2 == true
    
            % Random point (MODIFY HERE THE EXPLORATION SPACE FOR EACH DRONE IF REQUIRED)
            sign = randi([0, 1]) * 2 - 1;
            random_point = [sign * rand() * x_map_size/2, sign * rand() * y_map_size/2];
    
            if isInObstacle(random_point, inflated_obstacles)
                flag1 = true;
            else
                flag1 = false;
            end
    
            % Find nearest point wrt RRT tree
            distances = sqrt(sum((rrt_tree - random_point).^2, 2));
            [min_distance, nearest_idx] = min(distances);
    
            % Identify new RRT point
            direction = random_point - rrt_tree(nearest_idx, :);
            unit_direction = direction / norm(direction);
            new_point = rrt_tree(nearest_idx, :) + step_size * unit_direction;
    
            % Check compenetrarion of new point wrt the obstacles
            if isInObstacle(new_point, inflated_obstacles)
                flag2 = true;
            else
                flag2 = false;
            end
    
        end
    
        % Add the new point to the RRT tree
        rrt_tree = [rrt_tree; new_point];
        parent_indices = [parent_indices; nearest_idx];
    
        % % Verify if the goal is reached
        % if norm(new_point - goal) < goal_threshold
        %     disp('Goal raggiunto!');
        %     break;
        % end
    end
end

