% Inserisci qui il nome della tua figura salvata
nome_file_fig = 'PID_REAL_360SIeNO_AW.fig';
nome_file_pdf = 'PID_REAL_360SIeNO_AW.pdf';

% 1. Carica la figura salvata
fig = openfig(nome_file_fig);

% 2. Trova l'asse principale
ax = gca;

% 3. Configura assi, togli il bordo nero (box) e riduci i font
box off;
ax.FontSize = 8;
ax.XLabel.FontSize = 9;
ax.YLabel.FontSize = 9;

% Imposta l'inizio dell'asse Y a -5
ax.YLim(1) = -5; 

% 4. Modifica legenda: Testo, posizione e dimensione font
lgd = findobj(fig, 'Type', 'Legend');
if ~isempty(lgd)
    lgd.String = {'Without Anti-windup', 'With Anti-windup', 'Reference'};
    lgd.Location = 'southeast';
    lgd.FontSize = 8;
else
    legend('Without Anti-windup', 'With Anti-windup', 'Reference', ...
           'Location', 'southeast', 'FontSize', 8);
end

% 5. Modifica COLORI, STILI e spessore di tutte le curve plottate (Type: Line)
plot_lines = findobj(ax, 'Type', 'Line');
if length(plot_lines) >= 3
    % NOTA: findobj legge le linee dall'ultima creata alla prima.
    
    % Reference (Ultima plottata -> indice 1)
    plot_lines(1).Color = [0 0.4470 0.7410]; 
    plot_lines(1).LineStyle = ':'; % ---> NOVITÀ: Linea puntinata <---
    
    % With Anti-windup (Penultima plottata -> indice 2)
    plot_lines(2).Color = [0.8500 0.3250 0.0980]; 
    plot_lines(2).LineStyle = '-';
    
    % Without Anti-windup (Prima plottata -> indice 3)
    plot_lines(3).Color = [0.9290 0.6940 0.1250]; 
    plot_lines(3).LineStyle = '-';
    
    % Imposta lo spessore pulito a 0.5 per tutte
    set(plot_lines, 'LineWidth', 0.5);
end

% 6. Modifica lo spessore e il font delle linee orizzontali (Type: ConstantLine)
ylines = findobj(ax, 'Type', 'ConstantLine');
if ~isempty(ylines)
    set(ylines, 'LineWidth', 0.4);
    for i = 1:length(ylines)
        % Se la linea ha un'etichetta di testo, sistemala per non sovrapporsi
        if ~isempty(ylines(i).Label)
            ylines(i).FontSize = 5.5;
            ylines(i).LabelVerticalAlignment = 'top';
        end
    end
end

% 7. Imposta le dimensioni della figura
fig.Units = 'centimeters';
fig.Position = [5, 5, 18, 6.5];
fig.Color = 'w';

% 8. Esporta in PDF vettoriale senza bordi bianchi extra
exportgraphics(fig, nome_file_pdf, 'ContentType', 'vector', 'BackgroundColor', 'none');

disp(['Modifica completata! Figura salvata come: ', nome_file_pdf]);