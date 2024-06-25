%% main

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

%% Initial conditions
theta_0 = pi/2;
d_theta_0 = 0;
v_theta_0 = [theta_0, d_theta_0];

beta_0 = 0;
d_beta_0 = 0;
v_beta_0 = [beta_0, d_beta_0];

torque_in = 0;

theta_Q = 0;
beta_Q = 0;

%% Initialize Graphic
f1 = figure;
hold on;
grid on;

axis equal
xlabel('X (m)');
ylabel('Y (m)');
title('Reaction Wheel Inverted Pendulum')

% Initial positions
xw = -L*sin(theta_0); %wheel x center
yw = L*cos(theta_0); %wheel y center

xw_end = xw - r*sin(beta_0); %wheel x end
yw_end = yw + r*cos(beta_0); %wheel y end

pos_wheel = [xw-r, yw-r, r*2, r*2]; %[x y w h]

base = plot([-1, 1],[0, 0],'k','LineWidth',2); % base line

radious = plot([xw, xw_end],[yw, yw_end],'r','LineWidth',1.5); % Pendulum rod
pendulum = plot([0, xw],[0, yw],'b','LineWidth',1.5); % Pendulum rod
wheel = rectangle('Position',pos_wheel,'Curvature',[1 1], 'LineWidth',1.5);

axis equal
axis(gca,'equal');
xlim([-0.6 0.6]);
ylim([-0.1 0.6]);

text_handle = text(0.05, 0.95, 'Time: 0', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');

time_skip = 0.001;
time = 0;

%% Controllers discretization
% Compensator Parameters
k = 5.5;
z1 = 0.1;
p1 = 0.5;
z2 = 0.03;
p2 = 0.0093;

% PID Parameters
kp = -120.74 % -7.1649  % -59.025  % -59.025;
ki = 750.369 % 123.0725 % -8.38155 % -2.159;
kd = 20;

% Controller initial conditions
torque_prev = 0;
torque_prev_prev = 0;
error_prev = 0;
error_prev_prev = 0;

%% Main Loop
counter = 0;
while true
    % Time
    counter = counter + 1;
    time = time + time_skip;
    text = ['Time:', num2str(time)];

    % Update Variables
    [~, f_theta] = ode45(@(t,y) theta_model(t, y, a, b, torque_in), [0,time_skip], v_theta_0);
    [~, f_beta] = ode45(@(t,y) beta_model(t, y, Jw, torque_in),[0 time_skip], v_beta_0);
    
    v_theta = f_theta(end, :);
    v_beta = f_beta(end, :);
    theta = wrapToPi(v_theta(1));
    beta = wrapToPi(v_beta(1));
    
    % Update Controller Variables
    error = 0 - theta;  % Update actual error
    
    % PID Control Signal
    torque_in = kp*(error - error_prev) + ...
        ki*error*time_skip + ...
        kd*((error - 2*error_prev + error_prev_prev)/time_skip);
    
    
    % Add input perturbation
    if 2000 <= counter && counter < 2500
        torque_in = torque_in + input_perturbation(time);
    elseif counter >= 2500
        counter = 0;
    end

    % Lead Lag Compensator Control Signal
    % torque_in =  (k*(error - 2*error_prev + error_prev_prev) + ...
    %     k*time_skip*(z1+z2)*(error-error_prev) + ...
    %     error*k*z1*z2*(time_skip^2) + ...
    %     2*torque_prev - torque_prev_prev + ...
    %     time_skip*torque_prev_prev*(p1+p2)) / ...
    %     (p1*p2*(time_skip^2) + (p1+p2)*time_skip + 1);
    
    

    % limit max torque
    if torque_in < -1
        torque_in = -1;
    elseif torque_in > 1
        torque_in = 1;
    end
    % torque_in = max(min(torque_in, 1), -1);

    % Update Positions
    xw = -L*sin(theta); %wheel x center
    yw = L*cos(theta); %wheel y center
    
    xw_end = xw -  r*sin(beta); %wheel x end
    yw_end = yw + r*cos(beta); %wheel y end

    pos_wheel = [xw-r, yw-r, r*2, r*2]; %[x y w h]

    % Update initial conditions for next iteration
    v_theta_0 = v_theta;
    v_beta_0 = v_beta;

    % Draw graphics
    set(text_handle, 'String', text);
    set(pendulum,'XData',[0, xw],'YData',[0, yw]);
    set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
    set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
    drawnow;

    % update previous errors and torque
    error_prev_prev = error_prev;
    error_prev = error;  % Update previous error
    torque_prev_prev = torque_prev;
    torque_prev = torque_in;  % Update previous torque

    pause(time_skip);
end

function v_theta = theta_model(~, y, a, b, torque_in)
    v_theta = zeros(2,1);  %initialize a 2x1 null-vector ([0,0])
    v_theta(1) = y(2);  %theta
    v_theta(2) = (-a*y(1) - torque_in)/b;  %ddtheta
end

function v_beta = beta_model(~, y, Jw, torque_in)
    v_beta = zeros(2,1);  %initialize a 2x1 null-vector ([0,0])
    v_beta(1) = y(2);  %theta
    v_beta(2) = torque_in/Jw;  %ddtheta
end

function perturbation = input_perturbation(time)
    % Example perturbation: sinusoidal disturbance
    amplitude = 100;
    frequency = 0.5; % 1 Hz
    perturbation = amplitude * sin(2 * pi * frequency * time);
    %perturbation = amplitude;
end