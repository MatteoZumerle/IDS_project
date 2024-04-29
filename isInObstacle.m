function is_inside = isInObstacle(point, obstacles)
    is_inside = false;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        if point(1) >= obs(1) && point(1) <= obs(1) + obs(3) && ...
           point(2) >= obs(2) && point(2) <= obs(2) + obs(4)
            is_inside = true;
            break;
        end
    end
end
