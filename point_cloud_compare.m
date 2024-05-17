function common_points = point_cloud_compare(matrice1, matrice2, ground_truth1, ground_truth2, threshold)
    common_points = [];
    for i = 1:size(matrice1, 1)
        punto1 = matrice1(i, :);
        for j = 1:size(matrice2, 1)
            punto2 = matrice2(j, :);
            distanza = norm(punto1 - punto2); %distanza in linea tra i 2 punti
            if distanza <= threshold
                % Aggiungi il punto comune nei 2 sistemi di riferimento, gli indici per ogni sistema e i ground truth corrispondenti
                ground_truth_index1 = min(i, size(ground_truth1, 1));
                ground_truth_index2 = min(j, size(ground_truth2, 1));
                common_points = [common_points;i,punto1,j,punto2, ground_truth1(ground_truth_index1, :), ground_truth2(ground_truth_index2, :)];
                break;
            end
        end
    end
end
