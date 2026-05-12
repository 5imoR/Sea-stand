%%% LABORATORIO 4  LOGITUDINAL STATE SPACE CONTROL OF THE BALANCING ROBOT
clear all
clc

load ('./../balrob_params.mat'); % robot parameters

% Lagrangian Dynamic Equation: matrix formulation
r=sqrt((2.20*2)/1.06); %non trovo questo parametro nella lista del prof, l'ho ricavato io dall'Handout
% M(q)
M11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy+(batt.m+2*wheel.m+2*mot.m)*r^2;
M21_1 = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy;
M21_2 = (batt.m*46.05+2*mot.rot.m*-7)*r;
M22 = batt.Iyy+2*(1-gbox.N)^2*mot.rot.Iyy+batt.m*46.05^2+2*mot.rot.m*49;
% C(q,dot_q)
C11 = 0;
C12 = -(batt.m*46.05+2*mot.rot.m*-7)*r;
% Fv
F11 = 2*26.5*10^-3;
F12 = -2*25*10^-3;
F22 = 2*25*10^-3;
F=[F11 F12; F12 F22];
% g(q)
G1 = 0;
G2 = -(batt.m*46.05+2*mot.rot.m*-7)*g;