function new_map2 = fused_map(virtual_points, common_points, map2, drone_name)

            %Finding indices for common points in both matrices
            for pointer=1:length(virtual_points)
                for indice=1:size(common_points,1)
                    if common_points(indice,1) == pointer
                        virtual_points(pointer,1) = (common_points(indice,2)+common_points(indice,5))/2;
                        virtual_points(pointer,2) = (common_points(indice,3)+common_points(indice,6))/2;
                    end
                end
                    
            end
            
            %Calculate the mean of common points and save them in map2
            for pointer=1:length(map2)
                for indice=1:size(common_points,1)
                    if common_points(indice,4) == pointer
                        map2(pointer,1) = (common_points(indice,2)+common_points(indice,5))/2;
                        map2(pointer,2) = (common_points(indice,3)+common_points(indice,6))/2;
                    end
                end
                    
            end

            [new_virtual_points,new_map2] = combine_point_clouds(virtual_points,map2); %Mantaining characteristic points and common points from both matrices
            
            % Initialization and saving new meshed map in the scan_points field
            Drone.(drone_name).Lidar.scan_points = [0,0];
            Drone.(drone_name).Lidar.scan_points = new_map2;
end

