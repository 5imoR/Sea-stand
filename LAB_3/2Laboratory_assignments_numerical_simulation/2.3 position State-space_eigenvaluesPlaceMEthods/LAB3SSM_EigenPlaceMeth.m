%% LAB 3: POSITION CONTROL OF A DC SERVOMOTOR WITH RESONANT LOAD

clear, clc ,close all
load ('C:\Users\super\Desktop\CEL\prima\Progetti_Mio\LAB_3\\params_resonant_case.mat'); % motor parameters resonant case

load ('C:\Users\super\Desktop\CEL\prima\Progetti_Mio\LAB_3\est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)

%Jeq = J_eq_est; Beq = B_eq_est; Tausf = tau_sf_est; 

%new parameters computed for LAB3:
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + mld.Jh / (gbox.N^2); %PERCHE' ORA NON ABBIAMO PIù IL DISCO QUINDI Jeq!=J_eq_est
Ts = 0.001; % ADC sampling time [s]
Beq = B_eq_est;
%k = 0.83; % Nm/rad  già messo dal prof mld.k


%% SSM control using eignevalues placement design methods


A=[    0,                  0,                              1,                             0
       0,                  0,                              0,                             1
   -mld.K/((gbox.N^2)*Jeq),    mld.K/((gbox.N^2)*Jeq),  (-1/Jeq)*(Beq+((mot.Kt*mot.Ke)/Req)),     0
    mld.K/Jb,                -mld.K/Jb,                            0,                          -Bb/Jb];
