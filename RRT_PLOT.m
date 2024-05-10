function [] = RRT_PLOT(rrt_tree, parent_indices)
    hold on;

    plot(rrt_tree(1, 1), rrt_tree(1, 2),...
                    'Color', 'black', 'Marker' , 'o');
    
    for i = 2:size(rrt_tree, 1)
    
     plot([rrt_tree(parent_indices(i), 1), rrt_tree(i, 1)], ...
     [rrt_tree(parent_indices(i), 2), rrt_tree(i, 2)], 'LineStyle', '-', 'Color', 'red', 'Marker' , '.');
             
         
    end
       
end

