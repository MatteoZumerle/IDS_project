function [] = PLOT_TARGET_AREA(x_center,y_center, r)

theta = linspace(0, 2*pi, 100); 

x = r * cos(theta) + x_center;
y = r * sin(theta) + y_center;

fill(x, y, 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

end

