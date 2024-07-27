function sorted_matrix = sort_matrix_by_polar_angle(matrix)
    % Calculate the polar angle in degrees of each point relative to the origin (0.0)
    angles = atan2d(matrix(:, 2), matrix(:, 1));
    
    % Adding 360 to the negative angles so that all angles become positive
    angles(angles < 0) = angles(angles < 0) + 360;
    
    % Sorting the rows of the matrix by polar angle
    [~, sortIdx] = sort(angles);
    sorted_matrix = matrix(sortIdx, :);
end
