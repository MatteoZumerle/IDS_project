function [selected_color] = color(i)
    if i<7
        switch i
            case 1
                selected_color = [0 0.4470 0.7410];
            case 2
                selected_color = [0.8500 0.3250 0.0980];	
            case 3
                selected_color = [0.9290 0.6940 0.1250];
            case 4
                selected_color = [0.4940 0.1840 0.5560];
            case 5
                selected_color = [0.4660 0.6740 0.1880];
            case 6
                selected_color = [0.3010 0.7450 0.9330];
            case 7
                selected_color = [0.6350 0.0780 0.1840];
        end

    else
        selected_color = rand(1,3);

    end
end

