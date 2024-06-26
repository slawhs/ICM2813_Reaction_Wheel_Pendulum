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
%pidTuner(G_theta)

%% Step Response
% PID
kp = -71.25; %-120.74; % -7.1649  % -59.025  % -59.025;
ki = 735.369; %750.369; % 123.0725 % -8.38155 % -2.159;
kd = 26.7; %23;

pid = tf([kd kp ki], [1 0]);
control_system = feedback(pid*G_theta, 1);

t = 0:0.01:5;
y = lsim(control_system, 0*t, t);
plot(t, wrapToPi(y));

% [y, t] = step(control_system);
% y = wrapToPi(y);
% stepinfo(control_system)
title ('Respuesta a una referencia 0')
xlabel('Tiempo (s)')
ylabel('Amplitud (rad)')
grid on
