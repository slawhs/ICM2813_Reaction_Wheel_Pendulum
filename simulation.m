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
torque_limit = 5;

theta_Q = 0;
beta_Q = 0;

%% Initialize Graphics
f1 = figure;
subplot(2,1,1);  % Create subplot for drawing
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

text_handle = text(0.05, 0.95, 'Time: 0', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold');
theta_text_handle = text(0.05, 0.85, 'Theta: 0', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold');
beta_text_handle = text(0.05, 0.75, 'Beta: 0', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold');
stabilization_text_handle = text(0.35, 0.95, 'Stabilization time: N/A', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold');
post_disturbance_stabilization_text_handle = text(0.35, 0.85, 'Post-Perturbation Stabilization time: N/A', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold');
state_frame_text_handle = text(0.35, 0.75, 'State:', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold');
state_text_handle = text(0.45, 0.75, 'Stabilizing', 'Units', 'normalized', 'FontSize', 6, 'FontWeight', 'bold', 'Color', 'r');

% Initialize variables for plotting theta over time
theta_values = [];
time_values = [];
ref_values = [];

subplot(2,1,2);  % Create subplot for theta vs time
theta_plot = plot(time_values, theta_values, 'b');
xlabel('Time (s)');
ylabel('Theta (rad)');
title('Theta vs Time');
grid on;

time_skip = 0.001;
time = 0;

%% Controller discretization
% PID Parameters
kp = -71.25; %-120.74; % -7.1649  % -59.025  % -59.025;
ki = 735.369; %750.369; % 123.0725 % -8.38155 % -2.159;
kd = 26.7; %23;

% Controller initial conditions
torque_prev = 0;
torque_prev_prev = 0;
error_prev = 0;
error_prev_prev = 0;

% Distrubance parameters
disturbance_duration = 7 * 1000;
disturbance_counter = 0;

% Stabilization text setting
stabilization_counter = 0;
stable_state = false;
post_disturbance_stable_state = false;
stable_text = 'Stabilization time: N/A';
post_disturbance_stable_text = 'Post-Perturbation Stabilization time: N/A';
state_frame_text = 'State';
state_text = 'Stabilizing';
threshold = 0.1;  % Threshold for considering the pendulum stabilized
stable_time = 0;
post_disturbance_stable_time = 0;
stabilization_period = 1;  % Time in seconds to consider the system stabilized
stabilization_count = round(stabilization_period / time_skip);

disturbance_applied = false;

%% Main Loop

while time <= 10
    % Time
    disturbance_counter = disturbance_counter + 1;
    time = time + time_skip;
    text = ['Time:', num2str(time)];

    % Update Variables
    [~, f_theta] = ode45(@(t,y) theta_model(t, y, a, b, torque_in), [0,time_skip], v_theta_0);
    [~, f_beta] = ode45(@(t,y) beta_model(t, y, Jw, torque_in),[0 time_skip], v_beta_0);
    
    v_theta = f_theta(end, :);
    v_beta = f_beta(end, :);
    theta = wrapToPi(v_theta(1));
    beta = wrapToPi(v_beta(1));
    
    % Draw variables
    theta_text = ['Theta:', num2str(theta*180/pi)];
    beta_text = ['Beta:', num2str(beta*180/pi)];

    % Update Controller Variables
    error = 0 - theta;  % Update actual error
    
    % PID Control Signal
    torque_in = kp*(error - error_prev) + ...
        ki*error*time_skip + ...
        kd*((error - 2*error_prev + error_prev_prev)/time_skip);
    
    % Get time of stabilization
    if ~stable_state
        if abs(theta*180/pi) < threshold
            stabilization_counter = stabilization_counter + 1;
            if stabilization_counter >= stabilization_count
                stable_state = true;
                stable_time = time;
                stable_text = ['Stabilization time:', num2str(stable_time)];
                state_text = 'Stable';
                set(state_text_handle, 'String', state_text, 'Color', 'g');
            end
        else
            stabilization_counter = 0;
        end
    end

    % Add input disturbance
    if disturbance_duration <= disturbance_counter && disturbance_counter < disturbance_duration + 500
        torque_in = torque_in + input_disturbance(time);
        disturbance_applied = true;
        post_disturbance_stable_state = false;
        stabilization_counter = 0;
        state_text = 'Applying Disturbance';
        set(state_text_handle, 'String', state_text, 'Color', 'b');
    elseif disturbance_counter >= 7500
        disturbance_counter = 0;
        state_text = 'Stabilizing';
        set(state_text_handle, 'String', state_text, 'Color', 'r');
    end
    
    % Check post-disturbance stabilization
    if disturbance_applied && ~post_disturbance_stable_state
        if abs(theta*180/pi) < threshold
            stabilization_counter = stabilization_counter + 1;
            if stabilization_counter >= stabilization_count
                post_disturbance_stable_state = true;
                post_disturbance_stable_time = time - disturbance_duration * time_skip - 0.5;
                post_disturbance_stable_text = ['Post-Perturbation Stabilization time:', num2str(post_disturbance_stable_time)];
                state_text = 'Stable';
                set(state_text_handle, 'String', state_text, 'Color', 'g');
            end
        else
            stabilization_counter = 0;
        end
    end

    % limit max torque
    if torque_in < -torque_limit
        torque_in = -torque_limit;
    elseif torque_in > torque_limit
        torque_in = torque_limit;
    end

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
    set(stabilization_text_handle, 'String', stable_text);
    set(post_disturbance_stabilization_text_handle, 'String', post_disturbance_stable_text);
    set(theta_text_handle, 'String', theta_text, 'Color', 'b');
    set(beta_text_handle, 'String', beta_text, 'Color', 'r');
    set(text_handle, 'String', text);
    set(pendulum,'XData',[0, xw],'YData',[0, yw]);
    set(radious,'XData',[xw, xw_end], 'YData', [yw, yw_end]);
    set(wheel, 'Position',pos_wheel,'Curvature',[1 1]);
    drawnow;

    % Update theta values for the plot
    theta_values = [theta_values, theta*180/pi];
    time_values = [time_values, time];
    ref_values = [ref_values, 0];
    plot(time_values, theta_values, 'b', time_values, ref_values, 'r--');
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

function disturbance = input_disturbance(time)
    % Sinusoidal disturbance
    amplitude = 100;
    frequency = 0.5; % 1 Hz
    disturbance = amplitude * sin(2 * pi * frequency * time);
    % Constant disturbance
    %disturbance = amplitude;
end
