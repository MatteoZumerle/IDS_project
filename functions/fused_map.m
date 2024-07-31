function new_map2 = fused_map(virtual_points, common_points, map2, drone_name)

            for pointer=1:length(virtual_points)
                for indice=1:size(common_points,1)
                    if common_points(indice,1) == pointer
                        virtual_points(pointer,1) = (common_points(indice,2)+common_points(indice,5))/2;
                        virtual_points(pointer,2) = (common_points(indice,3)+common_points(indice,6))/2;
                    end
                end
                    
            end
            
            %punti in comune mediati, salvati in map2
            for pointer=1:length(map2)
                for indice=1:size(common_points,1)
                    if common_points(indice,4) == pointer
                        map2(pointer,1) = (common_points(indice,2)+common_points(indice,5))/2;
                        map2(pointer,2) = (common_points(indice,3)+common_points(indice,6))/2;
                    end
                end
                    
            end

            [new_virtual_points,new_map2] = combine_point_clouds(virtual_points,map2); %mantengo punti caratteristici, punti in comune e punti caratteristici altra matrice
            
            % Initialization and saving new meshed map in the scan_points_field
            Drone.(drone_name).Lidar.scan_points = [0,0];
            Drone.(drone_name).Lidar.scan_points = new_map2;
end

