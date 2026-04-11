function [Beq_est, tausf_est]=LAB0_step_mean(w, tau, t)
% Parametri temporali dei gradini
T_step = 5; % Durata di ogni gradino [s]
t_offset = 0.5; % Inizio della finestra di media (dopo il transitorio)


% Trova quanti gradini interi hai fatto nel test reale
num_steps = 9; 

% Inizializza i vettori per le medie
w_ss = zeros(num_steps, 1);
tau_ss = zeros(num_steps, 1);

% Calcola le medie gradino per gradino
for k = 1:num_steps
    % Definisci la finestra temporale per il gradino k-esimo
    t1 = (k-1)*T_step + t_offset;
    t2 = (k-1)*T_step + 5-t_offset;
    
    % Estrai gli indici di tutti i dati che cadono in questa finestra (la "shaded area")
    idx = find(t >= t1 & t <= t2);
    
    % Calcola la media e salvala nei vettori finali
    w_ss(k) = mean(w(idx));
    tau_ss(k) = mean(tau(idx));
end

% MLS, vector Y contains the measured torques
Y = tau_ss;
% matrix Phi contains velocity and sign of velocity
Phi = [w_ss, sign(w_ss)];
% Pseudo-inversa, theta will contains [Beq; tausf]
theta = Phi \ Y; 
Beq_est = theta(1);
tausf_est = theta(2);