
clc;
clear;
clear all;
clf;

%Map definition and obstacles plot
x_map_size = 10;
y_map_size = 10;
field_of_view = 5;

[obstacles, inflated_obstacles] = map_definition;

start = [5.5,6.5];

%LiDAR 
lidar_position1 = start;
lidar_angles = linspace(0, 360, 100);
scanning_lines = 100;% 100 scanning lines
max_range = 3;
lidar_dist_res = 0.002; 
[point_abs,point_abs_clean, point_rel1, dist_rel] = Lidar_scan(obstacles, lidar_position1, lidar_angles, max_range, lidar_dist_res);
lidar_position2 = [5,5];
[point_abs2,point_abs_clean2, point_rel2, dist_rel2] = Lidar_scan(obstacles, lidar_position2, lidar_angles, max_range, lidar_dist_res);

%Plot Lidar
figure(1);
for i = 1:size(obstacles, 1)
    hold on
    rectangle('Position', obstacles(i, :), 'FaceColor', '#0072BD');
end
% Plot inflated obstacles
for i = 1:size(inflated_obstacles, 1)
    rectangle('Position', inflated_obstacles(i, :), 'EdgeColor', 'k', 'LineStyle', '--');
end
% Plot LiDAR result map 1
for angle_step = 1:length(lidar_angles)-1
    angle = lidar_angles(angle_step);
    ray_direction = [cosd(angle), sind(angle)];
    hit_point = point_abs(angle_step, :);
    if ~all(hit_point == 0) % Plot just hit points

        plot([lidar_position1(1), hit_point(1)], [lidar_position1(2), hit_point(2)],'LineStyle', ':', 'Color', 'red', 'Marker' , "."'); % Plot ray
    end
end
grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
xlim([-0.10 x_map_size + 0.1]);
ylim([-0.10 y_map_size + 0.1]);
title('Lidar Test');

% Generazione delle mappe di punti Lidar per i due droni (sostituisci con le tue matrici)
map1 = point_rel1; %coordinate relative point cloud drone 1
map2 = point_rel2;%coordinate relative point cloud drone 2
ground_truth1 = point_abs_clean; %coordinate assolute point cloud drone 1 just for check
ground_truth2 = point_abs_clean2; %coordinate assolute point cloud drone 2 just for check
pos1 = lidar_position1; %ground truth posizione drone 1
pos2 = lidar_position2; %ground truth posizione drone 2

[distance, angle12, angle21] = antenna(pos1,pos2,scanning_lines) %funzione che mi da distanza e angoli del segnale
xtrasl = distance * cosd(angle21);
ytrasl = distance * sind(angle21);
virtual_points = [map1(:,1)+xtrasl,map1(:,2)+ytrasl];%traslo drone 1 (map1) virtualmente sorapposto a drone 2
soglia = 0.05;
common_points = point_cloud_compare(virtual_points,map2,ground_truth1,ground_truth2,soglia); %funzione che mi butta fuori anche indici di virtual e map2

%punti in comune mediati, salvati in virtual points (map1)
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
new_map1_prov = [new_virtual_points(:,1)-xtrasl,new_virtual_points(:,2)-ytrasl,new_virtual_points(:,3)]; %tolgo traslazione virtuale e riottengo map1 con punti in coune e map2
new_map1 = sort_matrix_by_polar_angle(new_map1_prov);

% Plot LiDAR result map 2 con map1 holdata
figure(1)
for angle_step = 1:length(lidar_angles)-1
    angle = lidar_angles(angle_step);
    ray_direction = [cosd(angle), sind(angle)];
    hit_point = point_abs2(angle_step, :);
    if ~all(hit_point == 0) % Plot just hit points

        plot([lidar_position2(1), hit_point(1)], [lidar_position2(2), hit_point(2)],'LineStyle', ':', 'Color', 'blue', 'Marker' , "."'); % Plot ray
    end
end
grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
xlim([-0.10 x_map_size + 0.1]);
ylim([-0.10 y_map_size + 0.1]);
title('Lidar Test');
hold off;

% Plot LiDAR result in relative envioment new_map1
figure(2)
for i=1:size(new_map1,1)
    if new_map1(i,3) == 1
        plot(new_map1(i, 1), new_map1(i, 2), 'o', 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'green', 'MarkerSize', 5); % Plot ray
    elseif new_map1(i,3) == 2
        plot(new_map1(i, 1), new_map1(i, 2), 'o', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red', 'MarkerSize', 5); % Plot ray
    elseif new_map1(i,3) == 3
        plot(new_map1(i, 1), new_map1(i, 2), 'o', 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'blue', 'MarkerSize', 5); % Plot ray
    end
hold on
grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
title('Lidar drone1 Relative point position');
end


img = imread('drone.jpg'); % Sostituisci con il percorso dell'immagine
imageWidth = 0.5; % Larghezza desiderata dell'immagine
imageHeight = 0.5; % Altezza desiderata dell'immagine
xPos = 0 - imageWidth/2; % Posizionamento sull'asse x
yPos = 0 - imageHeight/2; % Posizionamento sull'asse y

% Aggiungi l'immagine alla posizione specificata
image('CData', img, 'XData', [xPos, xPos + imageWidth], 'YData', [yPos, yPos + imageHeight]);

% Plot LiDAR result in relative envioment new_map2
figure(3)
for i=1:size(new_map2,1)
    if new_map2(i,3) == 1
        plot(new_map2(i, 1), new_map2(i, 2), 'o', 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'green', 'MarkerSize', 5); % Plot ray
    elseif new_map2(i,3) == 2
        plot(new_map2(i, 1), new_map2(i, 2), 'o', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'red', 'MarkerSize', 5); % Plot ray
    elseif new_map2(i,3) == 3
        plot(new_map2(i, 1), new_map2(i, 2), 'o', 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'blue', 'MarkerSize', 5); % Plot ray
    end
hold on
grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
title('Lidar drone2 Relative point position');
end


img = imread('drone.jpg'); % Sostituisci con il percorso dell'immagine
imageWidth = 0.5; % Larghezza desiderata dell'immagine
imageHeight = 0.5; % Altezza desiderata dell'immagine
xPos = 0 - imageWidth/2; % Posizionamento sull'asse x
yPos = 0 - imageHeight/2; % Posizionamento sull'asse y

% Aggiungi l'immagine alla posizione specificata
image('CData', img, 'XData', [xPos, xPos + imageWidth], 'YData', [yPos, yPos + imageHeight]);

%% Landmark recognition
mystruct.map1.coords = new_map1;
mystruct.map2.coords = new_map2;

for j=1:2
    map_name = ['map' num2str(j)];
    Res = 1;
    i = 1;
    while i <= size(mystruct.(map_name).coords,1)
        if i <= Res || i > size(mystruct.(map_name).coords,1) - (Res)
            mystruct.(map_name).profile(i, 1) = 1;
            mystruct.(map_name).profile(i, 2) = 0;
            mystruct.(map_name).profile(i, 3) = 0;
        else
        % Curvature
        F = [mystruct.(map_name).coords(i-Res,1),mystruct.(map_name).coords(i-Res,2), 0];
        G = [mystruct.(map_name).coords(i,1),mystruct.(map_name).coords(i,2), 0];
        H = [mystruct.(map_name).coords(i+Res,1),mystruct.(map_name).coords(i+Res,2), 0];
    
        f = norm(G - H);
        g = norm(F - H);
        h = norm(G - F);
        A = norm(cross(H - F, G - F) / 2);
        mystruct.(map_name).profile(i, 1) = 4 * A / (f * g * h); %curvature
    
        %PCA
        X = [mystruct.(map_name).coords(i-Res:i+Res,1),mystruct.(map_name).coords(i-Res:i+Res,2)];
        [Dir, score, Var] = pca(X);  
        mystruct.(map_name).profile(i, 2) = sqrt(Var(1)); %Var1
        mystruct.(map_name).profile(i, 3) = sqrt(Var(2)); %Var2
        end
        i = i+1;
    end
    
    filtering_threshold = 5;
    
    a = 1;
    while a<=size(mystruct.(map_name).profile,1)
        if mystruct.(map_name).profile(a, 1) >= filtering_threshold 
            mystruct.(map_name).peaks(a,1) = mystruct.(map_name).profile(a, 1);
        else
            mystruct.(map_name).peaks(a,1) = 0;
        end
        a = a+1;
    end
    
end

figure(2)
for i=1:size(mystruct.map1.peaks,1)
    if mystruct.map1.peaks(i,1) ~= 0
        plot(mystruct.map1.coords(i,1), mystruct.map1.coords(i,2), 'o', 'MarkerEdgeColor', 'black', 'MarkerSize', 10); 
    end
hold on
grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
title('Lidar drone1 landmarks');
end


%% generate obstacle from lidar data

%Drone 1
obstacles = lidar_data_to_obstacles(mystruct.map1.coords,0.5,1); %(coords, min distance tra punti per stesso cluster, min numero punti per cluster)
% Iinflation factor applied to the obstacles
inflate = 0.3;
inflated_obstacles = zeros(size(obstacles));
for i = 1:height(obstacles)
    inflated_obstacles(i, 1) = obstacles(i, 1) - inflate; % x iniziale spostato a sinistra
    inflated_obstacles(i, 2) = obstacles(i, 2) - inflate; % y iniziale spostato in basso
    inflated_obstacles(i, 3) = obstacles(i, 3) + 2*inflate; % larghezza aumentata
    inflated_obstacles(i, 4) = obstacles(i, 4) + 2*inflate; % altezza aumentata
end

figure (4);
for i = 1:size(obstacles, 1)
    rectangle('Position', obstacles(i, :), 'FaceColor', 'blue');
end

hold on;
for i = 1:size(inflated_obstacles, 1)
    rectangle('Position', inflated_obstacles(i, :), 'EdgeColor', 'red', 'LineStyle', '--');
end

grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
title('Room with inflated obstacles');
hold off;

%Drone 2
obstacles = lidar_data_to_obstacles(mystruct.map2.coords,1,1); %(coords, min distance tra punti per stesso cluster, min numero punti per cluster)
% Iinflation factor applied to the obstacles
inflate = 0.3;
inflated_obstacles = zeros(size(obstacles));
for i = 1:height(obstacles)
    inflated_obstacles(i, 1) = obstacles(i, 1) - inflate; % x iniziale spostato a sinistra
    inflated_obstacles(i, 2) = obstacles(i, 2) - inflate; % y iniziale spostato in basso
    inflated_obstacles(i, 3) = obstacles(i, 3) + 2*inflate; % larghezza aumentata
    inflated_obstacles(i, 4) = obstacles(i, 4) + 2*inflate; % altezza aumentata
end

figure (5);
for i = 1:size(obstacles, 1)
    rectangle('Position', obstacles(i, :), 'FaceColor', 'blue');
end

hold on;
for i = 1:size(inflated_obstacles, 1)
    rectangle('Position', inflated_obstacles(i, :), 'EdgeColor', 'red', 'LineStyle', '--');
end

grid on;
axis equal; 
xlabel('X [m]');
ylabel('Y [m]');
title('Room with inflated obstacles');
hold off;