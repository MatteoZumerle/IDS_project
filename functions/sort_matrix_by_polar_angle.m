function sorted_matrix = sort_matrix_by_polar_angle(matrix)
    % Calcola l'angolo polare in gradi di ciascun punto rispetto all'origine (0,0)
    angles = atan2d(matrix(:, 2), matrix(:, 1));
    
    % Aggiungi 360 agli angoli negativi in modo che tutti gli angoli siano positivi
    angles(angles < 0) = angles(angles < 0) + 360;
    
    % Ordina le righe della matrice in base all'angolo polare
    [~, sortIdx] = sort(angles);
    sorted_matrix = matrix(sortIdx, :);
end
