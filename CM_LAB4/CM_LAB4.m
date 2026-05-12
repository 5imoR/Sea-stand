%%% LABORATORIO 4  LOGITUDINAL STATE SPACE CONTROL OF THE BALANCING ROBOT
clear all
clc

load ('./../balrob_params.mat'); % robot parameters
load ('./../param.mat','pulse2deg'); % il prof non l'ha messo nella lista sopra

%% Lagrangian Dynamic Equation: matrix formulation
% M(q)
M11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy+(body.m+2*wheel.m+2*mot.rot.m)*wheel.r^2;
M21_1 = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy;
M21_2 = (body.m*body.zb+2*mot.rot.m*mot.rot.zb)*wheel.r;
M22 = body.Iyy+2*(1-gbox.N)^2*mot.rot.Iyy+body.m*(body.zb^2)+2*mot.rot.m*(mot.rot.zb^2);
% C(q,dot_q)
C11 = 0;
C12 = -(body.m*body.zb+2*mot.rot.m*mot.rot.zb)*wheel.r;
% Fv
F11 = 2*(gbox.B+wheel.B);
F12 = -2*gbox.B;
F22 = 2*gbox.B;
F=[F11 F12; F12 F22];
% g(q)
G1 = 0;
G2 = -(body.m*body.zb+2*mot.rot.m*mot.rot.zb)*g;