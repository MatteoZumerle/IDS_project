function is_inside = isInObstacle(point, obstacles)
    %Checking if selected point is inside any obstacle of the map

    %Priori condition
    is_inside = false;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);

        %Rectangle limits check for obstacles
        if point(1) >= obs(1) && point(1) <= obs(1) + obs(3) && ... %x limits
           point(2) >= obs(2) && point(2) <= obs(2) + obs(4) %y limits
            is_inside = true;
            break;
        end
    end
end
