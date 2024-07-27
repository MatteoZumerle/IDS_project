function [xtrasl1,ytrasl1] = antenna_location(Drone, drone_name_1,drone_name_2, count)
    % Simulation of a communication antenna in order to virtual overlap
    % Drone 1 on Drone 2
    xtrasl1 = Drone.(drone_name_1).RRT.all_positions(count-1,1)-Drone.(drone_name_2).RRT.all_positions(count-1,1);
    ytrasl1 = Drone.(drone_name_1).RRT.all_positions(count-1,2)-Drone.(drone_name_2).RRT.all_positions(count-1,2);
end

