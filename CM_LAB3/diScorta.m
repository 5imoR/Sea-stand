%% ========================================================================
% BLOCCO 1: SETUP E PROGETTO DEL CONTROLLORE
% (Clicca qui dentro e premi "Run Section" o "Esegui Sezione")
% ========================================================================
clear all; clc;

% --- 1.1 CARICAMENTO PARAMETRI ---
load('./../resonant_params.mat');                      
load('./../est_param.mat', 'B_eq_est', 'tau_sf_est');  
load('./../est_param_resonant.mat');                   

Beq = B_eq_est;
tau_sf = tau_sf_est;
Req = mot.R + sens.curr.Rs;         
Jeq = mot.J + mld.Jh / (gbox.N^2);  
Ts = 0.001;                         
t1 = 0.7;
t0 = 0.2;

% Specifiche di progetto comuni
Mp = 0.3;   % 30%
ts = 0.85;  % 0.85 s
delta = abs(log(Mp)) / sqrt(pi^2 + log(Mp)^2);
w_n = 3 / (delta * ts);

% --- 1.2 MENU DEI TEST ---
% Cambia questo numero da 1 a 7 prima di eseguire la sezione!
test_number = 1; 

switch test_number
    case 1
        disp('--> TEST 1: PID a 50 deg CON Anti-Windup');
        controller_choice = 1;
        step_amplitude = 50;
        antiwindup = 2;
        nome_campo = 'PID_50_AW';
        
    case 2
        disp('--> TEST 2: PID a 50 deg SENZA Anti-Windup');
        controller_choice = 1;
        step_amplitude = 50;
        antiwindup = 1;
        nome_campo = 'PID_50_NO_AW';
        
    case 3
        disp('--> TEST 3: PID a 120 deg CON Anti-Windup');
        controller_choice = 1;
        step_amplitude = 120;
        antiwindup = 2;
        nome_campo = 'PID_120_AW';
        
    case 4
        disp('--> TEST 4: PID a 120 deg SENZA Anti-Windup');
        controller_choice = 1;
        step_amplitude = 120;
        antiwindup = 1;
        nome_campo = 'PID_120_NO_AW';
        
    case 5
        disp('--> TEST 5: State-Space (Eigenvalues) a 50 deg');
        controller_choice = 2;
        step_amplitude = 50;
        nome_campo = 'SS_Eigen_50';
        
    case 6
        disp('--> TEST 6: LQR (SRL) a 50 deg');
        controller_choice = 3;
        step_amplitude = 50;
        nome_campo = 'LQR_SRL_50';
        
    case 7
        disp('--> TEST 7: LQR (Bryson) a 50 deg');
        controller_choice = 4;
        step_amplitude = 50;
        nome_campo = 'LQR_Bryson_50';
        
    otherwise
        error('ERROR: choose a number from 1 to 7');
end

% --- 1.3 CALCOLO DELLE MATRICI (Per State-Space) ---
if controller_choice ~= 1 
    A = [0 0 1 0; 0 0 0 1; 0 (k_est/(gbox.N^2*Jeq)) -(1/Jeq)*(Beq+((mot.Ke*mot.Kt)/Req)) 0; 0 -k_est/mld.Jb-k_est/(Jeq*gbox.N^2) -(Bb_est/mld.Jb)+(1/Jeq)*(Beq+mot.Ke*mot.Kt/Req) -Bb_est/mld.Jb];
    B = [0; 0; (mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req); -(mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req)];
    C = [1 0 0 0];
    S = [A B; C 0];    
    N = S \ [0; 0; 0; 0; 1];
    Nx = N(1:4);        
    Nu = N(5);      
end    
        
% --- 1.4 PROGETTO DEL CONTROLLORE ---
switch controller_choice
    case 1  % PID
        alpha = 4;   
        numP= [(drv.dcgain*mot.Kt*mld.Jb) (drv.dcgain*mot.Kt*Bb_est) (drv.dcgain*mot.Kt*k_est)];
        denP = [(gbox.N*Req*Jeq*mld.Jb) gbox.N*(Req*Jeq*Bb_est+Req*mld.Jb*Beq+mot.Kt*mot.Ke*mld.Jb) gbox.N*(mot.Kt*mot.Ke*Bb_est+Req*Beq*Bb_est+Req*k_est*(Jeq+mld.Jb/(gbox.N^2))) gbox.N*(mot.Kt*mot.Ke*k_est+Req*k_est*Beq+Req*k_est*Bb_est/(gbox.N^2)) 0];
        P_s = tf(numP,denP);
        [Kp, Ki, Kd, ~, ~] = LAB3_design_controller(P_s, Mp, ts, alpha);
        Tl = 1 / (10 * w_n);
        
        if antiwindup == 2
            Kw = 1 / (ts / 5);  % AW Acceso
        else 
            Kw = 0;             % AW Spento
        end 
        
    case 2 % EIGENVALUES
        phi = atan(sqrt(1-delta^2)/(delta));
        p1 = w_n*exp(1i*(-pi+(phi))); p2 = conj(p1);
        p3 = w_n*exp(1i*(-pi+(phi/2))); p4 = conj(p3);
        poles = [p1, p2, p3, p4];         
        K = place(A, B, poles);  
        
    case 3 % LQR SRL
        sysG = ss(A,B,C,0);
        r = 1/4.59e3;
        K = lqr(sysG, C'*C, r);
        
    case 4 % LQR BRYSON
        sysG = ss(A,B,C,0);
        Q = [1/(0.3*step_amplitude*pi/180)^2 0 0 0; 0 1/(pi/36)^2 0 0; 0 0 0 0; 0 0 0 0;];
        R = 1/100;
        K = lqr(sysG,Q,R);
end 

disp('----------------------------------------------------');
disp('VAI SU SIMULINK! Parametri generati correttamente.');
disp(['1. Controlla di avere il modello corretto aperto.']);
disp(['2. Assicurati che il blocco Step sia impostato a: ', num2str(step_amplitude)]);
disp(['3. Clicca CONNECT e poi START.']);
disp(['4. Quando il motore si ferma, esegui il BLOCCO 2 qui sotto.']);
disp('----------------------------------------------------');



%% ========================================================================
% BLOCCO 2: SALVATAGGIO DEI DATI
% (Esegui questa sezione SOLO DOPO che la simulazione in Simulink è finita!)
% ========================================================================

% 1. Controllo se esiste già il file dei risultati per caricare i test precedenti
if isfile('Risultati_Totali_LAB3.mat')
    load('Risultati_Totali_LAB3.mat', 'Dati_Lab3');
else
    disp('Creo un nuovo file di salvataggio...');
end

% 2. Salvo le variabili base (che ci sono sempre)
Dati_Lab3.(nome_campo).thh_meas = thh_meas;
Dati_Lab3.(nome_campo).thh_ref  = thh_ref;
Dati_Lab3.(nome_campo).u        = u;

% 3. Salvo la deformazione del giunto SOLO SE il blocco l'ha generata
if exist('thd_meas', 'var')
    Dati_Lab3.(nome_campo).thd_meas = thd_meas;
end

% 4. Salvo su file
save('Risultati_Totali_LAB3.mat', 'Dati_Lab3');

disp(['--> OTTIMO! Test aggiunto alla struttura. Salvato nel campo: Dati_Lab3.', nome_campo]);
disp('Se devi fare un altro test, cambia "test_number" nel Blocco 1 e ricomincia!');
disp('----------------------------------------------------');