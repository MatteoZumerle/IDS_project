 function color = random_color()
    % Function to select randomly a color among the drone's one
    colors = {'Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Black', 'White', 'Silver'};

    % Generate the index
    index = randi(length(colors));

    % Call back the color
    color = colors{index};
end
