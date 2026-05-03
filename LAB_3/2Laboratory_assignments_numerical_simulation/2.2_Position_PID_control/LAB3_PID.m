%% LAB 3: POSITION CONTROL OF A DC SERVOMOTOR WITH RESONANT LOAD

clear, clc ,close all
load ('C:\Users\super\Desktop\CEL\prima\Progetti_Mio\LAB_3\params_resonant_case.mat'); % motor parameters resonant case

load ('C:\Users\super\Desktop\CEL\prima\Progetti_Mio\LAB_3\est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)

%Jeq = J_eq_est; Beq = B_eq_est; Tausf = tau_sf_est; 

%new parameters computed for LAB3:
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + mld.Jh / (gbox.N^2); %PERCHE' ORA NON ABBIAMO PIù IL DISCO QUINDI Jeq!=J_eq_est
Ts = 0.001; % ADC sampling time [s]
Beq = B_eq_est;
%%k = 0.83; % Nm/rad


%% PID:

%% 1. Definition PLANT in MATLAB

s = tf('s');

% 1. Definisci il polinomio Denominator_tau_prime come da Equazione (26)
D_tau_prime = Jeq * mld.Jb * s^3 + ...
              (Jeq * mld.Bb + mld.Jb * Beq) * s^2 + ...
              (Beq * mld.Bb + k * (Jeq + mld.Jb / (gbox.N^2))) * s + ...
              k * (Beq + mld.Bb / (gbox.N^2));

% 2. Definisci il numeratore del blocco meccanico elastico
num_mech = mld.Jb * s^2 + mld.Bb * s + k;

% 3. Costruisci il Plant esatto P_{u -> th_h}(s) come da Equazione (34)
Plant = (1 / (gbox.N * s)) * (drv.dcgain * mot.Kt * num_mech) / ...
        (Req * D_tau_prime + mot.Kt * mot.Ke * num_mech);

% Mostriamo il diagramma di Bode della pianta per controllo
figure(1);
bode(Plant);
grid on;
title('Diagramma di Bode dell''Impianto da controllare'); 


%% Il tempo e l'overshoot si traducono in pulsazione e margine di fase (con formule standard)
Mp = 0.3;     %overshoot   

delta = log(1/Mp)/sqrt(pi^2+log(1/Mp)^2);  %dumpingfactor

tsteady = 0.85;     % t steady state 5% <= 0.85s

margin_phase_rad = atan(2*delta/(sqrt(sqrt(1+4*delta^4)-(2*delta^2))));  %Fi m -->marginphase
margin_phase_deg = margin_phase_rad * (180/pi);

w_gc = 3/(delta*tsteady);          % Pulsazione di taglio 
alpha = 4;          % Rapporto Ti/Td

%%Per sapere quanto deve "spingere" il PID, dobbiamo prima sapere come si comporta il motore da solo esattamente alla frequenza desiderata
[mag_P, phase_P] = bode(Plant, w_gc);

%Appiattisco le matrici 3D in numeri singoli (non penso serva più di tanto)
mag_P=squeeze(mag_P);
phase_P=squeeze(phase_P);

%%calcolo della derivata di fase
phi_pid_deg = -180 + margin_phase_deg - phase_P;
phi_pid_rad = phi_pid_deg * (pi/180);


Td=(tan(phi_pid_rad)+sqrt((tan(phi_pid_rad))^2)+4/alpha)/(2*w_gc);
Ti = alpha * Td;

Tw = tsteady/5;

Tl = 1/(10*w_gc);
Kw = 1/Tw;

%%Il punto chiave del Metodo di Bode è che alla frequenza w_{gc} il modulo totale 
mag_pid = 1 / mag_P;  %è il delta K
Kp = mag_pid * cos(phi_pid_rad);
Ki = Kp / Ti;
Kd = Kp * Td;

%% Stampa a video dei risultati
fprintf('--- Risultati Progetto PID ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);