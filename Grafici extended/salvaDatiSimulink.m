function salvaDatiSimulink(serieTemporale, nomeFile, nomeVariabile)
    % SALVADATISIMULINK Salva i dati aggiungendoli a un file .mat esistente
    % senza sovrascrivere i dati precedenti.

    if ~endsWith(nomeFile, '.mat')
        nomeFile = [nomeFile, '.mat'];
    end

    try
        % 1. Inseriamo dati e tempo dentro un campo dinamico di una struttura 'S'
        % Se nomeVariabile è 'sim_1', MATLAB creerà S.sim_1.dati e S.sim_1.tempo
        S.(nomeVariabile).dati = serieTemporale.Data;
        S.(nomeVariabile).tempo = serieTemporale.Time;
        
        % 2. Salviamo o aggiorniamo il file
        if isfile(nomeFile)
            % Se il file esiste già, AGGIUNGIAMO la nuova variabile (-append)
            % L'opzione '-struct' dice a MATLAB di non salvare "S", ma il suo contenuto
            save(nomeFile, '-struct', 'S', '-append');
            fprintf('✅ Aggiunta variabile "%s" al file: %s\n', nomeVariabile, nomeFile);
        else
            % Se è la prima iterazione e il file non esiste, lo creiamo
            save(nomeFile, '-struct', 'S');
            fprintf('✅ Creato file %s e salvata variabile "%s"\n', nomeFile, nomeVariabile);
        end
        
    catch ME
        error('Errore durante il salvataggio: %s', ME.message);
    end
end