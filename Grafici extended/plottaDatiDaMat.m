function plottaDatiDaMat(nomeFile, variabiliDaPlottare, nomiLegenda, titoloGrafico, labelY)
    % PLOTTADATIDAMAT Carica i dati dal file .mat e genera un grafico interattivo.

    % --- 1. GESTIONE DEGLI ARGOMENTI OPZIONALI ---
    if nargin < 2 || isempty(variabiliDaPlottare)
        variabiliDaPlottare = {};
    end
    if nargin < 3 || isempty(nomiLegenda)
        nomiLegenda = {};
    end
    if nargin < 4 || isempty(titoloGrafico)
        titoloGrafico = sprintf('Data Comparison: %s', nomeFile);
    end
    if nargin < 5 || isempty(labelY)
        labelY = 'Signal Value';
    end

    % --- 2. CONTROLLO FILE E CARICAMENTO ---
    if ~endsWith(nomeFile, '.mat')
        nomeFile = [nomeFile, '.mat'];
    end

    if ~isfile(nomeFile)
        error('❌ File %s does not exist.', nomeFile);
    end

    caricato = load(nomeFile);
    campiPresenti = fieldnames(caricato);

    % --- FIX: MANTENIAMO L'ORDINE ESATTO DELL'UTENTE ---
    if isempty(variabiliDaPlottare)
        campiScelti = campiPresenti;
    else
        if ischar(variabiliDaPlottare) || isstring(variabiliDaPlottare)
            variabiliDaPlottare = cellstr(variabiliDaPlottare);
        end
        
        campiScelti = {};
        for k = 1:length(variabiliDaPlottare)
            if ismember(variabiliDaPlottare{k}, campiPresenti)
                campiScelti{end+1} = variabiliDaPlottare{k};
            else
                warning('Variable "%s" not found in the file and will be ignored.', variabiliDaPlottare{k});
            end
        end
        
        if isempty(campiScelti)
            error('None of the requested variables are present in the file.');
        end
    end

    % Assicuriamoci che i nomi della legenda siano un cell array
    if ischar(nomiLegenda) || isstring(nomiLegenda)
        nomiLegenda = cellstr(nomiLegenda);
    end

    % --- 3. CREAZIONE GRAFICO ---
    figure('Name', 'Simulink Results', 'NumberTitle', 'off', 'Color', 'w');
    hold on;

    % Ciclo di plottaggio
    for i = 1:length(campiScelti)
        nomeVar = campiScelti{i};
        datiStruttura = caricato.(nomeVar);

        if isfield(datiStruttura, 'tempo') && isfield(datiStruttura, 'dati')
            
            % Assegnazione nome legenda: se fornito dall'utente lo usa, altrimenti usa il nome variabile
            if i <= length(nomiLegenda) && ~isempty(nomiLegenda{i})
                nomeDisplay = nomiLegenda{i};
            else
                nomeDisplay = nomeVar;
            end
            
            plot(datiStruttura.tempo, datiStruttura.dati, 'LineWidth', 1.5, 'DisplayName', nomeDisplay);
        else
            warning('Variable "%s" ignored because it misses "dati" or "tempo" fields.', nomeVar);
        end
    end

    % --- 4. GRAFICA E LAYOUT IN INGLESE ---
    hold off;
    grid on;
    set(gca, 'GridAlpha', 0.4);
    
    xlabel('Time (s)', 'FontWeight', 'bold');
    ylabel(labelY, 'FontWeight', 'bold');
    title(titoloGrafico, 'Interpreter', 'none');
    
    lgd = legend('show', 'Interpreter', 'none', 'Location', 'best');
    title(lgd, 'Click on names to hide/show'); 
    lgd.ItemHitFcn = @nascondiSegnale; 
end

% =========================================================================
% SOTTO-FUNZIONE DI CALLBACK
% =========================================================================
function nascondiSegnale(~, event)
    if strcmp(event.Peer.Visible, 'on')
        event.Peer.Visible = 'off';
    else
        event.Peer.Visible = 'on';
    end
end