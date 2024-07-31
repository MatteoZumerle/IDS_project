function [xtrasl1,ytrasl1] = antenna_location(Drone, drone_name_1,drone_name_2, count)
    % Simulate the angle so the traslation to ho in onrder to locate one
    % drone wrt the other
    xtrasl1 = Drone.(drone_name_1).RRT.all_positions(count-1,1)-Drone.(drone_name_2).RRT.all_positions(count-1,1);
    ytrasl1 = Drone.(drone_name_1).RRT.all_positions(count-1,2)-Drone.(drone_name_2).RRT.all_positions(count-1,2);
end

