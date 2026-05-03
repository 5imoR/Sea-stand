%% LABORATORIO 2 CHALLENGE - DIRECT DIGITAL DESIGN
clear all
clc

step_amplitude = 60;
T=0.19;
tracking_choice=2;

load ('./../../../param.mat'); % motor parameters
load ('./../../../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)

% Plant
load('./../../../ssPlant_param.mat', 'A','B','C','D')

% specs
Mp = 0.1;
ts = 0.2;
Kw=25;

% poles (lambda)
delta = (log(1/Mp)) / sqrt(pi^2 + (log(1/Mp))^2);
w_n = 3 / (delta * ts); 
sigma = -15; 
omega_d = 4.0931; 
sigma3 = -2.11;

lambda = [sigma + 1i*omega_d; sigma - 1i*omega_d]; % vector of poles

% Discrete State-Space
plant_ct = ss(A,B,C,D);
plant_dt = c2d(plant_ct,T,'zoh'); % discretized system
[phi, gamma, H, J] = ssdata(plant_dt);

lambda_dt = exp(lambda*T); % controller poles in z-plan
K = place(phi,gamma,lambda_dt); % DT controller gain 

N_xu = [phi-eye(2), gamma; H, 0]\[0; 0; 1];
Nx = N_xu(1:2);
Nu = N_xu(3);

% Discrete Observer
lambda_o = 5*sigma; 
lambda_o_dt = exp(lambda_o*T); % Observer pole in z-plane

phi11 = phi(1,1); phi22 = phi(2,2); phi12 = phi(1,2); phi21 = phi(2,1);
L = acker(phi22', phi12', lambda_o_dt)'; % Observer gain
% or L = (phi22-lambda_o_dt)/phi12

phi_o = lambda_o_dt;
gamma_o = [gamma(2)-L*gamma(1), (phi22-L*phi12)*L+phi21-L*phi11];
Ho = [0;1];
Jo = [0 1; 0 L];

lambda_e=[lambda; sigma3];
phi_e = [1 H; zeros(2,1) phi];
gamma_e = [0; gamma];
lambda_e_dt = exp(lambda_e*T);
Ke = place(phi_e, gamma_e, lambda_e_dt);
   
Ki = Ke(1);
K = Ke(2:3);
Nr = Nu + K * Nx; % feedforward gain
 


%% --- ESTRAZIONE DATI E PLOT ---
% Assicurati che i blocchi "To Workspace" siano impostati su "Structure with Time"
t = out.thl.time;
y = out.thl.signals.values; % Previene problemi di dimensioni
r = out.ref.signals.values;

% Valore di regime del gradino (partendo da t=0)
r_final = r(end); 

% Calcolo dei limiti di assestamento (±5%)
upper_bound = r_final * 1.05;
lower_bound = r_final * 0.95;

% Creazione Grafico
figure('Name', 'Tracking Performance', 'NumberTitle', 'off');
plot(t, r, 'b', 'LineWidth', 1.2); hold on;
plot(t, y, 'r', 'LineWidth', 1.2);

% Plot dei limiti al 5% con linea TRATTEGGIzATA ('r--')
plot(t, upper_bound * ones(size(t)), 'g--', 'LineWidth', 1);
plot(t, lower_bound * ones(size(t)), 'm--', 'LineWidth', 1);

% Formattazione grafico
xlabel('Time [s]');
ylabel('\theta_L (Amplitude)');
title('Step Response');
legend('Reference', 'Output (\theta_L)', '+5% Bound', '-5% Bound', 'Location', 'Southeast');
grid on;
xlim([0, t(end)]);

%% --- VALUTAZIONE PERFORMANCE ---

% 1. Calcolo del TEMPO DI ASSESTAMENTO e SOVRAELONGAZIONE sui dati simulati
info = stepinfo(y, t, r_final, 'SettlingTimeThreshold', 0.05);

% 2. Calcolo dell'ERRORE A REGIME TEORICO (Indipendente dalla durata della simulazione)
% Costruiamo il sistema a ciclo chiuso teorico: x(k+1) = (Phi - Gamma*K)x(k) + Gamma*Nr*r(k)
Phi_cl = phi - gamma * K;
Gamma_cl = gamma * Nr;
C_cl = H; % Assumiamo J (o D) = 0, comune per questi sistemi fisici
D_cl = 0;

sys_cl_dt = ss(Phi_cl, Gamma_cl, C_cl, D_cl, T);
DC_gain = dcgain(sys_cl_dt); % Guadagno in continua del sistema discreto

% Valore di output a regime teorico e rispettivo errore
y_ss_theoretical = DC_gain * r_final;
ss_error_theoretical = r_final - y_ss_theoretical;

% Stampa dei risultati nella Command Window
fprintf('\n=== PERFORMANCE METRICS ===\n');
fprintf('Tempo di assestamento (5%%): %.4f s\n', info.SettlingTime);
fprintf('Sovraelongazione (Mp):      %.2f %%\n', info.Overshoot);
fprintf('Errore a regime (teorico):  %.4e\n', ss_error_theoretical);
fprintf('===========================\n\n');