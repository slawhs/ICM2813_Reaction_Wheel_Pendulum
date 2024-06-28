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

theta_Q = pi/6;
% Aux Variables
a = (Mp*Lp + Mw*L)*g*cos(theta_Q);
b = Mp*Lp^2 + Mw*L^2 + Jp;

%% Control System Designer
G_theta = tf([1], [-b 0 -a]);
% controlSystemDesigner(G_theta)
pidTuner(G_theta,'PID')

%% Step Response
% PID
% kp = -13.7; %-120.74; % -7.1649  % -59.025  % -59.025;
% ki = -36.3; %750.369; % 123.0725 % -8.38155 % -2.159;
% kd = -1.29; %23;
% 
% pid = tf([kd kp ki], [1 0]);
% control_system = feedback(pid*G_theta, 1);
% 
% [y, t] = step(control_system);
% y = wrapToPi(y);
% plot(t, wrapToPi(y));
% stepinfo(control_system)
% title ('Respuesta a una referencia 0')
% xlabel('Tiempo (s)')
% ylabel('Amplitud (rad)')
% grid on
