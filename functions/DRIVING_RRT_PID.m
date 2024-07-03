function DRIVING_RRT_PID(Drone, drone_name, SetUp, proximity_threshold)
        
% PID control to move the drone
        % Check actual and target position to set up the controller
        Drone.(drone_name).PID.error_pos = Drone.(drone_name).RRT.final_pos - Drone.(drone_name).RRT.pos;
            
        % Proportional component
        Drone.(drone_name).PID.control_p = Drone.(drone_name).PID.Kp * Drone.(drone_name).PID.error_pos;

        % Integral component
        Drone.(drone_name).PID.error_sum = Drone.(drone_name).PID.error_sum + Drone.(drone_name).PID.error_pos * SetUp.Dt;
        Drone.(drone_name).PID.control_i = Drone.(drone_name).PID.Ki * Drone.(drone_name).PID.error_sum;

        % Derivative component
        Drone.(drone_name).PID.derivative = (Drone.(drone_name).PID.error_pos - Drone.(drone_name).PID.previous_error) / SetUp.Dt;
        Drone.(drone_name).PID.control_d = Drone.(drone_name).PID.Kd * Drone.(drone_name).PID.derivative;
        
        % Speed saturation
        Drone.(drone_name).PID.control_d = min(max(Drone.(drone_name).PID.control_d, -Drone.(drone_name).PID.max_speed), Drone.(drone_name).PID.max_speed);

        % Final limited acceleration control
        Drone.(drone_name).PID.control = Drone.(drone_name).PID.control_p + Drone.(drone_name).PID.control_i + Drone.(drone_name).PID.control_d;
        Drone.(drone_name).PID.control = min(max(Drone.(drone_name).PID.control, -Drone.(drone_name).PID.max_acc), Drone.(drone_name).PID.max_acc);
        
        % IMU noise added
        Drone.(drone_name).PID.control = Drone.(drone_name).PID.control + Drone.(drone_name).RRT.imu_noise * randn(2, 1);

        % Speed and position update
        Drone.(drone_name).RRT.vel = Drone.(drone_name).RRT.vel + Drone.(drone_name).PID.control * SetUp.Dt;
        Drone.(drone_name).RRT.pos = Drone.(drone_name).RRT.pos + Drone.(drone_name).RRT.vel * SetUp.Dt;

        % Pose memorization
        Drone.(drone_name).RRT.positions = [Drone.(drone_name).RRT.positions, Drone.(drone_name).RRT.pos];
        
        Drone.(drone_name).RRT.reached_goal(count,1) = Drone.(drone_name).map(Drone.(drone_name).count_pos,1);
        Drone.(drone_name).RRT.reached_goal(count,2) = Drone.(drone_name).map(Drone.(drone_name).count_pos,2); 
        Drone.(drone_name).RRT.all_positions(count,1) = Drone.(drone_name).RRT.reached_goal(count,1) + Drone.(drone_name).RRT.pos(1,1);
        Drone.(drone_name).RRT.all_positions(count,2) = Drone.(drone_name).RRT.reached_goal(count,2) + Drone.(drone_name).RRT.pos(2,1);
        
        % Error update
        Drone.(drone_name).PID.previous_error = Drone.(drone_name).PID.error_pos;

        %------------------------------------------------------------------------------------------------------------%

        % From now on a series of if to check and control the planning
        % steps of the drone in the RRT exploration point
        
        %  Check if a general intermediate checkpoint position of the RRT tree is reached 
        if norm(Drone.(drone_name).RRT.pos - Drone.(drone_name).RRT.final_pos) <= proximity_threshold && Drone.(drone_name).RRT.check_point_index < length(Drone.(drone_name).RRT.points)
            % aggiungere punto check point approx
            Drone.(drone_name).RRT.check_point_index = Drone.(drone_name).RRT.check_point_index +1;
            Drone.(drone_name).RRT.final_pos = Drone.(drone_name).RRT.points(:,Drone.(drone_name).RRT.check_point_index); 

        end

        % Check if the final point if the local RRT tree is reached
        if norm(Drone.(drone_name).RRT.pos - Drone.(drone_name).RRT.points(:,length(Drone.(drone_name).RRT.points))) <= proximity_threshold
            
            % Update of position index saved in the map
            Drone.(drone_name).count_pos = Drone.(drone_name).count_pos +1; 

            % Save the reached position in relative terms and use it in the
            % following RRT as the new starting point in the successive in
            % map variable
            Drone.(drone_name).map(Drone.(drone_name).count_pos,1) = Drone.(drone_name).map(Drone.(drone_name).count_pos-1,1) +  Drone.(drone_name).RRT.pos(1,1); %pos x abs
            Drone.(drone_name).map(Drone.(drone_name).count_pos,2) = Drone.(drone_name).map(Drone.(drone_name).count_pos-1,2) +  Drone.(drone_name).RRT.pos(2,1); %pos y abs
            Drone.(drone_name).map_rel(Drone.(drone_name).count_pos,1) = Drone.(drone_name).RRT.pos(1,1); %pos x rel
            Drone.(drone_name).map_rel(Drone.(drone_name).count_pos,2) = Drone.(drone_name).RRT.pos(2,1); %pos y rel
           
            % Save last speed of the drone in order to use it as the new
            % Starting velocity in the next RRT plannig step
            Drone.(drone_name).RRT.final_velocity = Drone.(drone_name).RRT.vel;

            % New scan in the reached point in order to generate the new RRT 
            [~,~, Drone.(drone_name).Lidar.scan_points_new, Drone.(drone_name).Lidar.scan_dist_new] = Lidar_scan(obstacles, Drone.(drone_name).map(Drone.(drone_name).count_pos,:), Drone.(drone_name).Lidar.lidar_angles, Drone.(drone_name).Lidar.max_range, Drone.(drone_name).Lidar.lidar_dist_res);
            
            % Fuse previous lidar points of the old map in the new just created
            map1 = Drone.(drone_name).Lidar.scan_points; % Old point clouds of drone n
            map2 = Drone.(drone_name).Lidar.scan_points_new; % New point cloud  drone n dto be fused
            pos1 = Drone.(drone_name).map(Drone.(drone_name).count_pos-1,:); % Old ground truth pose drone n
            pos2 = Drone.(drone_name).map(Drone.(drone_name).count_pos,:); % New (current) ground truth posizione drone n 
            
            % Virtual traslation (overlay) of old map (map1) on new reached point on new scanned map (map2)
            virtual_points = [map1(:,1)-Drone.(drone_name).RRT.pos(1,1),map1(:,2)-Drone.(drone_name).RRT.pos(2,1)];
            
            % Define the common points that theoretically are the same betweeen old and new scan
            soglia = 0.1;
            common_points = point_cloud_compare(virtual_points,map2,soglia);
            
            % Generate the new cumulated scanned point maps
            new_map2 = fused_map(virtual_points, common_points, map2, drone_name);

            % Initialization and saving new meshed map in the scan_points_field
            Drone.(drone_name).Lidar.scan_points = [0,0];
            Drone.(drone_name).Lidar.scan_points = new_map2;
end

