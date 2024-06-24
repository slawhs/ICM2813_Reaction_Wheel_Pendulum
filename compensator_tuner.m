%% Parameters
clc;
clear all;

Mw = 0.55;
Mp = 0.14;
Jw = 4.36*10^(-3);
Jp = 0.69*10^(-3);
L = 0.22;
Lp = 0.11;
g = 9.81;
r = 0.11/2;

% Aux Variables
a = (Mp*Lp + Mw*L)*g;
b = Mp*Lp^2 + Mw*L^2 + Jp;

%% Control System Designer
% G_beta = tf([0 1/Jw], [1 0 0]);
% controlSystemDesigner(G_beta)

G_theta = tf([1], [-b 0 -a]);
% controlSystemDesigner(G_theta)
pidTuner(G_theta)
