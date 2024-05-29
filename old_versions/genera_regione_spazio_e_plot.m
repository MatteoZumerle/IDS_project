function [regione_principale, regione_secondaria] = genera_regione_spazio_e_plot(punti, soglia)
    % Calcola il centroide dei punti
    centroide = mean(punti);

    % Calcola la distanza massima dai punti al centroide
    distanza_massima = max(sqrt(sum((punti - centroide).^2, 2)));

    % Dividi i punti in base alla distanza dal centroide
    punti_principali = punti(sqrt(sum((punti - centroide).^2, 2)) <= soglia, :);
    punti_secondari = punti(sqrt(sum((punti - centroide).^2, 2)) > soglia, :);

    % Genera la regione principale
    regione_principale = genera_forma(punti_principali, centroide, distanza_massima);

    % Genera la regione secondaria
    regione_secondaria = genera_forma(punti_secondari, centroide, distanza_massima);
    
    % Plot delle regioni
    figure;
    hold on;
    % Plot dei punti
    plot3(punti_principali(:,1), punti_principali(:,2), punti_principali(:,3), 'r.', 'MarkerSize', 10);
    plot3(punti_secondari(:,1), punti_secondari(:,2), punti_secondari(:,3), 'b.', 'MarkerSize', 10);
    % Plot delle regioni
    plot3(regione_principale{1}, regione_principale{2}, regione_principale{3}, 'r');
    plot3(regione_secondaria{1}, regione_secondaria{2}, regione_secondaria{3}, 'b');
    % Dettagli del plot
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Regioni dello Spazio');
    legend('Punti nella regione principale', 'Punti nella regione secondaria', 'Regione principale', 'Regione secondaria');
    hold off;
end

function forma = genera_forma(punti, centroide, raggio)
    % Aggiungi il centroide alla lista dei punti
    punti = [punti; centroide];

    % Calcola l'ampiezza dei punti
    ampiezza_x = max(punti(:, 1)) - min(punti(:, 1));
    ampiezza_y = max(punti(:, 2)) - min(punti(:, 2));
    ampiezza_z = max(punti(:, 3)) - min(punti(:, 3));

    % Scegli l'ampiezza massima come diametro della forma
    diametro = max([ampiezza_x, ampiezza_y, ampiezza_z]);

    % Calcola la forma come una sfera con raggio = diametro / 2
    [x, y, z] = sphere;
    x = x * (diametro / 2) + centroide(1);
    y = y * (diametro / 2) + centroide(2);
    z = z * (diametro / 2) + centroide(3);

    forma = {x, y, z};
end
