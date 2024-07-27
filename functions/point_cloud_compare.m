function common_points = point_cloud_compare(matrice1, matrice2, threshold)
    common_points = [];

    %After a virtual overlap of the 2 point clouds, find if different
    %points can be considered "common", calculating the Euclidean distance
    %from the common reference system and considering a compatible common measure just if the distance error is limited by a threshold 
    for i = 1:size(matrice1, 1)
        punto1 = matrice1(i, :);
        for j = 1:size(matrice2, 1)
            punto2 = matrice2(j, :);
            distanza = norm(punto1 - punto2); %Euclidean distance
            if distanza <= threshold
                % Adding common points in both reference frames, indices
                % and ground truth (just for plots)
                common_points = [common_points;i,punto1,j,punto2];
                break;
            end
        end
    end
end
