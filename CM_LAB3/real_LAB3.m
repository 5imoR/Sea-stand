%% LABORATORY 3 - RESONANT LOAD POSITION CONTROLLER
clear all
clc

%% CODE AND SIMULATION SETUP: ONLY MODIFIABLE PART

controller_choice = 1; % 1 = PID, 2 = EIGENV_ALL, 3 = LQR_SRL, 4 = LQR_BRYSON 
test_number = 1; 


%% LOAD PARAMETERS
load ('./../resonant_params.mat');                      % Nominal motor and load parameters
load ('./../est_param.mat','B_eq_est','tau_sf_est');    % Estimated friction parameters 
load ('./../est_param_resonant.mat');                   % Estimated elastic joint parameters

Beq = B_eq_est;
tau_sf = tau_sf_est;
Req = mot.R + sens.curr.Rs;         % Total equivalent resistance
Jeq = mot.J + mld.Jh / (gbox.N^2);  % Equivalent inertia seen at motor side

Ts = 0.001;                         % Sampling time [s]
t1 = 0.7;
t0 = 0.2;

% -------- RIFARE ROOT LOCUS A PARTE -----------------------


%% SIMULATION MENU

switch test_number
    case 1
        disp('--> TEST 1: PID at 50 deg WITHOUT Anti-Windup');
        controller_choice = 1;
        step_amplitude = 50;
        antiwindup = 1;
        field_name = 'dati_PID_50_NO_AW.mat';
        
    case 2
        disp('-->  TEST 2: PID at 50 deg WITH Anti-Windup');
        controller_choice = 1;
        step_amplitude = 50;
        antiwindup = 2;
        field_name = 'dati_PID_50_AW.mat';
        
    case 3
        disp('--> TEST 3: PID at 120 deg WITHOUT Anti-Windup');
        controller_choice = 1;
        step_amplitude = 120;
        antiwindup = 1;
        field_name = 'dati_PID_120_NO_AW.mat';
        
    case 4
        disp('--> TEST 1: PID at 120 deg WITH Anti-Windup');
        controller_choice = 1;
        step_amplitude = 120;
        antiwindup = 2;
        field_name = 'dati_PID_120_AW.mat';
        
    case 5
        disp('--> TEST 5: State-Space (Eigenvalues) at 50 deg');
        controller_choice = 2;
        step_amplitude = 50;
        field_name = 'dati_SS_Eigen_50.mat';
        
    case 6
        disp('--> Eseguo TEST 6: LQR (SRL) at 50 deg');
        controller_choice = 3;
        step_amplitude = 50;
        field_name = 'dati_LQR_SRL_50.mat';
        
    case 7
        disp('--> Eseguo TEST 7: LQR (Bryson) at 50 deg');
        controller_choice = 4;
        step_amplitude = 50;
        field_name = 'dati_LQR_Bryson_50.mat';
        
    otherwise
        error('ERROR: choose a number from 1 to 7');
end

%%

if controller_choice ~= 1 

    % High pass filter for real derivaties (representative of the observer) 
    wc = 2*pi*50;         % Cut.off frequency [rad/s]
    deltac = 1/sqrt(2);   % Damping factor
    
    % State space model of the plant (State vector x' = [th_h, th_d, w_h, w_d]')
    A = [0     0                                 1                                              0; 
         0     0                                 0                                              1; 
         0     (k_est/(gbox.N^2*Jeq))            -(1/Jeq)*(Beq+((mot.Ke*mot.Kt)/Req))             0; 
         0     -k_est/mld.Jb-k_est/(Jeq*gbox.N^2)  -(Bb_est/mld.Jb)+(1/Jeq)*(Beq+mot.Ke*mot.Kt/Req) -Bb_est/mld.Jb];
       
    B = [0; 0; (mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req); -(mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req)];
    Bd=[0; 0; 1/((gbox.N^2)*Jeq); -1/((gbox.N^2)*Jeq)];
    C=[1 0 0 0];
    
    % Feedforward matrices
    S = [A B; C 0];    
    N = S \ [0; 0; 0; 0; 1];
    Nx = N(1:4);    % state feedback matrix        
    Nu = N(5);      % input feedback matrix
end    
        
switch controller_choice
    
    case 1  

        % PID CONTROLLER & ANTI-WINDUP

        % Specifications for PID design
        alpha= 4;   % Ratio between Ti and Td
        Mp = 0.3;   % Maximum overshoot (30%)
        ts = 0.85;  % Settling time [s]
        
        % Plant transfer function U --> thh
        numP= [(drv.dcgain*mot.Kt*mld.Jb)  (drv.dcgain*mot.Kt*Bb_est)  (drv.dcgain*mot.Kt*k_est)];
        denP = [(gbox.N*Req*Jeq*mld.Jb)  gbox.N*(Req*Jeq*Bb_est+Req*mld.Jb*Beq+mot.Kt*mot.Ke*mld.Jb) gbox.N*(mot.Kt*mot.Ke*Bb_est+Req*Beq*Bb_est+Req*k_est*(Jeq+mld.Jb/(gbox.N^2))) gbox.N*(mot.Kt*mot.Ke*k_est+Req*k_est*Beq+Req*k_est*Bb_est/(gbox.N^2)) 0];
        
        P_s = tf(numP,denP);
        
        % Compute PID gains, required damping factor, and natural frequency
        [Kp, Ki, Kd, delta, w_n] = LAB3_design_controller(P_s, Mp, ts, alpha);
        
        % Real derivative filter design 
        Tl = 1 / (10 * w_n);
        
        % AntiWindup parameters
        if antiwindup == 2
            Tw = ts / 5;
            Kw = 1 / Tw;    % Anti-windup gain
        else 
            Kw = 0;
        end 

        model_name = 'real_LAB3_PID';

    case 2

        % STATE-SPACE CONTROLLER WITH EIGENVAlUES ALLOCATIONS METHOD
        
        % Target poles for the closed-loop system
        Mp = 0.3;   % Maximum overshoot (30%)
        ts = 0.85;  % Settling time [s]
        
        delta = (log(1/Mp))/sqrt(pi^2+(log(1/Mp))^2);
        w_n = 3/(delta*ts); 

        phi = atan(sqrt(1-delta^2)/(delta));
        p1 = w_n*exp(1i*(-pi+(phi)));     % First pair of dominant poles
        p2 = conj(p1);
        p3 = w_n*exp(1i*(-pi+(phi/2)));   % Second pair of faster poles
        p4 = conj(p3);
        poles = [p1, p2, p3, p4];         % Poles vector
        
        % State feedback matrix
        K = place(A, B, poles);  
        
        model_name = 'real_LAB3_SS';


    case 3
        
        % STATE SPACE CONTROLLER WITH LQR METHOD via Symmetric Root Locus
        sysG=ss(A,B,C,0);
        sysGp=ss(-A,-B,C,0);
        
         
        % % Plot SRL to analyze how closed-loop eigenvalues move as input cost (r) decreases
        % figure(1)
        % rlocus(sysG*sysGp)
        % hold on
        % % Plot performance boundaries based on specs (Settling time and Overshoot)
        % plot([-3/ts -3/ts], [-1 1]*(-3/ts*tan(phi)), '--', 'Color', [0 0.4470 0.7410])
        % plot([-100 -3/ts], [-100 -3/ts]*tan(phi), '--', 'Color', [0 0.4470 0.7410])
        % plot([-100 -3/ts], [-100 -3/ts]*-tan(phi), '--', 'Color', [0 0.4470 0.7410])
        % grid on
        % axis([-60 60 -60 60]);
        % title('Symmetric Root Locus (LQR Design)')
        % hold off
        
        % Gain 1/r chosen graphically to keep poles inside the desired performance region
        r = 1/4.59e3;
        
        % Compute LQR gain matrix (Standard cost function with Q = C'*C)
        K = lqr(sysG, C'*C, r);

        model_name = 'real_LAB3_SS';

    case 4
        
        % STATE SPACE CONTROLLER WITH LQR METHOD via Bryson's Rule
        sysG=ss(A,B,C,0);
        Q = [1/(0.3*50*pi/180)^2 0           0  0;
             0                   1/(pi/36)^2 0  0;
             0                   0           0  0;
             0                   0           0  0;];
        R = 1/100;
        
        % Compute LQR gain matrix using Bryson's Rule weights
        K = lqr(sysG,Q,R);

        model_name = 'real_LAB3_SS';

    otherwise
        
        error('ERROR - CHOICE IS NOT VALID! Choose 1, 2 or 3.');
end 

%% 3. ESECUZIONE SIMULINK DESKTOP REAL-TIME
disp(['Model opening: ', model_name]);
open_system(model_name);

Tsim = 5;
set_param(model_name, 'StopTime', num2str(Tsim));
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'FixedStepDiscrete'); 
set_param(model_name, 'FixedStep', num2str(Ts)); 
set_param([model_name, '/Step'], 'After', num2str(step_amplitude));
set_param([model_name, '/Step'], 'Time', '1');

% Connection and Start
set_param(model_name, 'SimulationMode', 'external');

disp('Connecting to target ... ');
set_param(model_name, 'SimulationCommand', 'connect');
pause(1); 

disp('Motor start!');
set_param(model_name, 'SimulationCommand', 'start');

% Pause for the duration of the silmulation
pause(Tsim + 0.5);



%% CREAZIONE DELLA STRUTTURA UNICA E SALVATAGGIO DEI DATI

if isfile('results_LAB3.mat')
    load('results_LAB3.mat', 'Data_Lab3');
end

Data_Lab3.(field_name).thh_meas = thh_meas;
Data_Lab3.(field_name).thh_ref  = thh_ref;
Data_Lab3.(field_name).u        = u;
if exist('thd_meas', 'var')
    Data_Lab3.(field_name).thd_meas = thd_meas;
end

save('results_LAB3.mat', 'Data_Lab3');

