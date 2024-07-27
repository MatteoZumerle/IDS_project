function [distance] = antenna(abs_point1,abs_point2)
% Antenna function works emulating the estimation of the distance between
% the 2 drones, permitting them to comunicate if below a given treeshold
distance_vector = abs_point2 - abs_point1;

% Eucledian distance between 2 points
distance = norm(distance_vector);

end