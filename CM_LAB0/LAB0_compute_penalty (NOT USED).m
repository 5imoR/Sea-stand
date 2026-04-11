function penalita_totale = LAB0_compute_penalty(K, P_s)
    s = tf('s');
    Kp = K(1);
    Ki = K(2);
    Kd = K(3);
    % Evita guadagni negativi (che destabilizzano il sistema)
    if Kp < 0 || Ki < 0 || Kd < 0
        penalita_totale = 1e6; % Penalità infinita
        return;
    end
    % Crea il PID in prova
    C = Kp + Ki/s + Kd*s;
    % Calcola il sistema in anello chiuso
    W = feedback(C * P_s, 1);
    % Estrai le prestazioni nel tempo
    info = stepinfo(W);
    overshoot = info.Overshoot;
    ts = info.SettlingTime;
    % DEFINISCI I LIMITI DELLE SPECIFICHE
    max_overshoot = 10; % %
    max_ts = 0.15;      % secondi
    % Calcola gli "Sgarri" (quanto stiamo sforando le specifiche)
    sgarro_overshoot = max(0, overshoot - max_overshoot);
    sgarro_ts = max(0, ts - max_ts);
    % Pesi per guidare l'algoritmo (è più grave sforare il tempo o l'overshoot?)
    peso_overshoot = 50;
    peso_ts = 1000;
    % Calcola l'errore a regime per penalizzare se il sistema non è preciso
    dc_gain = dcgain(W);
    errore_regime = abs(1 - dc_gain);
    % Somma tutto: se questa somma è 0, le specifiche sono perfettamente rispettate!
    penalita_totale = (sgarro_overshoot * peso_overshoot) + (sgarro_ts * peso_ts) + (errore_regime * 10000);
end