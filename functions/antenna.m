function [distance,angle12,angle21] = antenna(abs_point1,abs_point2,scanning_lines)
distance_vector = abs_point2 - abs_point1;

% Calcolo della distanza euclidea tra i due punti
distance = norm(distance_vector);

% Converti l'angolo in gradi
angle21 = atand(abs(abs_point1(1,2)-abs_point2(1,2))/abs(abs_point1(1,1)-abs_point2(1,1)));
angle12 = 360 - angle21;
incremento = 360/scanning_lines;
angle21 = round(angle21 / incremento) * incremento;
angle12 = round(angle12 / incremento) * incremento;


end