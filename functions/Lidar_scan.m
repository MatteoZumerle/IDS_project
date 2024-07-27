function [obstacle_abs_coords,obstacle_abs_coords_clean, relative_point, distance] = Lidar_scan(obstacles, lidar_position, lidar_angles, max_range, lidar_dist_res,measure_noise)
    
    obstacle_abs_coords = zeros(length(lidar_angles), 2); 
    relative_point = [];
    distance = [];
    
    %Scanning lines acquisition algorithm
    for angle_step = 1:length(lidar_angles)-1
        hitted = 0;
        angle = lidar_angles(angle_step);
        ray_direction = [cosd(angle), sind(angle)];

        hit_point = [0,0]; 

        for range = 0:lidar_dist_res:max_range % 0.01 is the resolution of the lidar distance
            scan_point = lidar_position + range * ray_direction;
            if isInObstacle(scan_point, obstacles) == true && hitted == 0
                hitted = 1;
                hit_point = scan_point;
                relative_point = [relative_point; range * ray_direction + measure_noise*[randn,randn]];
                distance = [distance; range];
                break;
            end
        end
           
   % Save absolute obstacle coordinates for each ray
    obstacle_abs_coords(angle_step, :) = hit_point; 
    
    %remove zeros from previous matrix
    index = any(obstacle_abs_coords ~= 0, 2);
    obstacle_abs_coords_clean= obstacle_abs_coords(index, :);
    
    end

end