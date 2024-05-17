function obstacles_lidar = lidar_data_to_obstacles(lidar_data, epsilon, minPts)
    %DBSCAN per clusterizzare punti
    [idx, ~] = dbscan(lidar_data, epsilon, minPts);
    obstacles_lidar = [];
    

    % salvo numero di cluster trovati 
    unique_clusters = unique(idx);
    num_clusters = length(unique_clusters);

    % Trovo i punti estremi delle pareti per ogni cluster
    for i = 1:num_clusters
        if unique_clusters(i) ~= -1 % serve per ignorare il rumore
            cluster_points = lidar_data(idx == unique_clusters(i), :);

            % Punti estremi della parete
            min_x = min(cluster_points(:, 1));
            max_x = max(cluster_points(:, 1));
            min_y = min(cluster_points(:, 2));
            max_y = max(cluster_points(:, 2));

            %Larghezza e altezza della parete
            width = max_x - min_x;
            height = max_y - min_y;

            obstacles_lidar = [obstacles_lidar; min_x, min_y, width, height];
        end
    end

end
