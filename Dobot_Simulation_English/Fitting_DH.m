%% Fitting_DH.m - DH Parameter Calibration and Verification Tool
% Purpose: Fit and optimize DH parameters to match real robot measurements
% 
% This script helps calibrate the DH model parameters (alpha, d, a offsets)
% based on collected real robot data (joint angles vs measured end effector positions)
%
% Usage:
%   1. Collect N position-angle pairs from real robot using Dobot Studio
%   2. Store in: q_measured (N×3) and p_measured (N×3)
%   3. Run optimization to find best-fit DH parameters
%   4. Compare simulated vs real robot trajectories
%
% Note: This is a template. Implementation depends on available measurement data.

clc; clear; close all;

%% 1. Build nominal robot with given DH parameters

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

L(2).offset = -pi/2;
L(3).offset =  pi/2;
L(2).qlim = [  0,  85] * pi/180;
L(3).qlim = [-10, 90] * pi/180;

robot_nominal = SerialLink(L, 'name', 'Dobot_3R_Nominal');
robot_nominal.base = transl(0, 0, -8);

%% 2. Simulated measurement data (in real application, load from experiments)

% Example: Generate synthetic data from perturbed model
% In real use, q_measured comes from Dobot Studio readings
% p_measured comes from precision measurement (e.g., laser tracker)

n_points = 10;
q_measured = rand(n_points, 3) .* [pi, 85*pi/180, 90*pi/180];
q_measured(:,2) = q_measured(:,2) * 0.8 + 10*pi/180;  % Keep in valid range
q_measured(:,3) = q_measured(:,3) * 0.8 - 10*pi/180;

% Simulate measurements with small perturbation
T_meas = robot_nominal.fkine(q_measured);
p_measured = transl(T_meas) + randn(n_points, 3) * 0.5;  % Add 0.5mm noise

fprintf('Generated %d measurement points\n', n_points);

%% 3. DH Parameter Optimization

% DH parameters to optimize: vectorized form
% Could optimize: alpha, d, a, offset, etc.

theta0 = [alpha, d, a, -pi/2, pi/2];  % Initial guess

% Optimization function
function err = fit_error(theta, q_data, p_data, robot_template)
    % Rebuild robot with perturbed DH parameters
    alpha_new = theta(1:3);
    d_new = theta(4:6);
    a_new = theta(7:9);
    offset2_new = theta(10);
    offset3_new = theta(11);
    
    L_opt(1) = Link([0, d_new(1), a_new(1), alpha_new(1)], 'standard');
    L_opt(2) = Link([0, d_new(2), a_new(2), alpha_new(2)], 'standard');
    L_opt(3) = Link([0, d_new(3), a_new(3), alpha_new(3)], 'standard');
    
    L_opt(2).offset = offset2_new;
    L_opt(3).offset = offset3_new;
    L_opt(2).qlim = robot_template.links(2).qlim;
    L_opt(3).qlim = robot_template.links(3).qlim;
    
    robot_opt = SerialLink(L_opt, 'name', 'Dobot_Optimized');
    robot_opt.base = robot_template.base;
    
    % Calculate FK error
    T_opt = robot_opt.fkine(q_data);
    p_opt = transl(T_opt);
    
    err = sum(sqrt(sum((p_opt - p_data).^2, 2)));  % Total Euclidean distance
end

% Perform optimization
options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 100);
theta_opt = fmincon(@(th) fit_error(th, q_measured, p_measured, robot_nominal), ...
                     theta0, [], [], [], [], [], [], [], options);

%% 4. Build optimized robot and compare

alpha_opt = theta_opt(1:3);
d_opt = theta_opt(4:6);
a_opt = theta_opt(7:9);
offset2_opt = theta_opt(10);
offset3_opt = theta_opt(11);

L_opt(1) = Link([0, d_opt(1), a_opt(1), alpha_opt(1)], 'standard');
L_opt(2) = Link([0, d_opt(2), a_opt(2), alpha_opt(2)], 'standard');
L_opt(3) = Link([0, d_opt(3), a_opt(3), alpha_opt(3)], 'standard');

L_opt(2).offset = offset2_opt;
L_opt(3).offset = offset3_opt;
L_opt(2).qlim = robot_nominal.links(2).qlim;
L_opt(3).qlim = robot_nominal.links(3).qlim;

robot_opt = SerialLink(L_opt, 'name', 'Dobot_3R_Optimized');
robot_opt.base = robot_nominal.base;

%% 5. Error comparison

T_nominal = robot_nominal.fkine(q_measured);
p_nominal = transl(T_nominal);

T_opt = robot_opt.fkine(q_measured);
p_opt = transl(T_opt);

err_nominal = p_nominal - p_measured;
err_opt = p_opt - p_measured;
err_nominal_norm = sqrt(sum(err_nominal.^2, 2));
err_opt_norm = sqrt(sum(err_opt.^2, 2));

fprintf('\n===== Calibration Results =====\n');
fprintf('Nominal model - Mean error: %.3f mm, Max error: %.3f mm\n', ...
    mean(err_nominal_norm), max(err_nominal_norm));
fprintf('Optimized model - Mean error: %.3f mm, Max error: %.3f mm\n', ...
    mean(err_opt_norm), max(err_opt_norm));
fprintf('Improvement: %.1f%%\n', (1 - mean(err_opt_norm)/mean(err_nominal_norm))*100);

%% 6. Visualization

figure('Name', 'DH Calibration Results');
subplot(1,2,1);
scatter3(p_measured(:,1), p_measured(:,2), p_measured(:,3), 'r*', 'LineWidth', 2); hold on;
scatter3(p_nominal(:,1), p_nominal(:,2), p_nominal(:,3), 'b.', 'LineWidth', 2);
scatter3(p_opt(:,1), p_opt(:,2), p_opt(:,3), 'g.', 'LineWidth', 2);
grid on; axis equal; xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP Positions: Measured vs Simulated');
legend('Measured', 'Nominal Model', 'Optimized Model');

subplot(1,2,2);
plot(err_nominal_norm, 'b-', 'LineWidth', 1.5); hold on;
plot(err_opt_norm, 'g-', 'LineWidth', 1.5);
grid on; xlabel('Point Index'); ylabel('Error Norm [mm]');
title('Position Error Comparison');
legend('Nominal', 'Optimized');
