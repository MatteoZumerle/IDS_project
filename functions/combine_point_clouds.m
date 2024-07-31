function [new_matrix1, new_matrix2] = combine_point_clouds(matrix1, matrix2)

    [~, idx_common_mat1, idx_common_mat2] = intersect(matrix1, matrix2, 'rows');
    
    common_points1 = matrix1(idx_common_mat1, :);
    common_points2 = matrix2(idx_common_mat2, :);
    
    [common_points1, sortIdx] = sortrows(common_points1);
    common_points2 = common_points2(sortIdx, :);
    
    [~, idx_unique_mat1] = setdiff(matrix1, matrix2, 'rows');
    unique_points_mat1 = matrix1(idx_unique_mat1, :);

    [~, idx_unique_mat2] = setdiff(matrix2, matrix1, 'rows');
    unique_points_mat2 = matrix2(idx_unique_mat2, :);

    unique_points_mat1_with_value = [unique_points_mat1, repmat(2, size(unique_points_mat1, 1), 1)];
    
    common_points_with_value = [common_points1, repmat(1, size(common_points1, 1), 1)];
    
    unique_points_mat2_with_value = [unique_points_mat2, repmat(3, size(unique_points_mat2, 1), 1)];
    
    % Unisco le matrici insieme
    new_matrix1 = [unique_points_mat1_with_value; common_points_with_value; unique_points_mat2_with_value];
    new_matrix2 = [unique_points_mat2_with_value; common_points_with_value; unique_points_mat1_with_value];
    
    % Calcolo degli angoli polari
    angles1 = atan2d(new_matrix1(:, 2), new_matrix1(:, 1));
    angles2 = atan2d(new_matrix2(:, 2), new_matrix2(:, 1));
    
    angles1(angles1 < 0) = angles1(angles1 < 0) + 360;
    angles2(angles2 < 0) = angles2(angles2 < 0) + 360;
    
    % Ordino i punti in base all'angolo polare
    [~, sortIdx1] = sort(angles1);
    [~, sortIdx2] = sort(angles2);
    
    new_matrix1 = new_matrix1(sortIdx1, :);
    new_matrix2 = new_matrix2(sortIdx2, :);
    
    % Trovo l'indice del primo punto con angolo 0
    idx_first_angle1 = find(new_matrix1(:, 3) == 1, 1);
    idx_first_angle2 = find(new_matrix2(:, 3) == 1, 1);
    
    % Riordino i punti finali partendo dall'angolo 0
    new_matrix1 = [new_matrix1(idx_first_angle1:end, :); new_matrix1(1:idx_first_angle1-1, :)];
    new_matrix2 = [new_matrix2(idx_first_angle2:end, :); new_matrix2(1:idx_first_angle2-1, :)];
end
