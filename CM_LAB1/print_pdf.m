% Inserisci qui il nome della tua figura salvata
nome_file_fig = 'nominal40.fig'; 
nome_file_pdf = 'Nominal_Tracking_40deg.pdf';

% 1. Carica la figura salvata
fig = openfig(nome_file_fig);

% 2. Trova l'asse principale
ax = gca;

% 3. Configura assi, togli il bordo nero (box) e riduci i font
box off;
ax.FontSize = 8;
ax.XLabel.FontSize = 9;
ax.YLabel.FontSize = 9;

% 4. Rimpicciolisci la legenda (se presente)
lgd = findobj(fig, 'Type', 'Legend');
if ~isempty(lgd)
    lgd.FontSize = 8;
end

% 5. Modifica lo spessore di tutte le curve plottate (Type: Line)
% findobj restituisce le linee in ordine inverso (l'ultima creata è la numero 1)
plot_lines = findobj(ax, 'Type', 'Line');
if ~isempty(plot_lines)
    set(plot_lines, 'LineWidth', 1);
    % Rendi la linea sperimentale (LAB) leggermente più spessa se è l'ultima che hai plottato
    plot_lines(1).LineWidth = 1.2; 
end

% 6. Modifica lo spessore e il font delle linee orizzontali (Type: ConstantLine)
ylines = findobj(ax, 'Type', 'ConstantLine');
if ~isempty(ylines)
    set(ylines, 'LineWidth', 0.5);
    for i = 1:length(ylines)
        % Se la linea ha un'etichetta di testo, sistemala per non sovrapporsi
        if ~isempty(ylines(i).Label)
            ylines(i).FontSize = 5.5;
            ylines(i).LabelVerticalAlignment = 'top';
        end
    end
end

% 7. Imposta le dimensioni della figura (12 cm x 6.5 cm) per combaciare con la tabella LaTeX
fig.Units = 'centimeters';
fig.Position = [5, 5, 18, 6.5];
fig.Color = 'w';

% 8. Esporta in PDF vettoriale senza bordi bianchi extra
exportgraphics(fig, nome_file_pdf, 'ContentType', 'vector', 'BackgroundColor', 'none');

disp(['Modifica completata! Figura salvata come: ', nome_file_pdf]);