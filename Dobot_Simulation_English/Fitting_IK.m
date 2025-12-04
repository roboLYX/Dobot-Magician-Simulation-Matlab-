%% Fitting_IK.m - IK Function Parameter Optimization and Testing
% Purpose: Verify and test analytical IK function accuracy and performance
%
% Compares analytical (closed-form) IK with numerical (ikcon) IK:
% - Position accuracy after FK verification
% - Computation time efficiency
% - Reliability across different workspace regions

clc; clear; close all;

%% 1. Build nominal robot

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

robot = SerialLink(L, 'name', 'Dobot_3R');
robot.base = transl(0, 0, -8);

%% 2. Generate test points in workspace

fprintf('===== Analytical IK Fitting and Validation =====\n\n');

% Generate random test joint configurations
n_tests = 20;
q_test = rand(n_tests, 3) .* [pi, 70*pi/180, 80*pi/180];
q_test(:,2) = q_test(:,2) + 10*pi/180;   % Shift J2 into valid range [0, 85°]
q_test(:,3) = q_test(:,3) - 10*pi/180;   % Shift J3 into valid range [-10, 90°]

% Compute corresponding TCP positions via FK
T_test = robot.fkine(q_test);
p_test = transl(T_test);

fprintf('Generated %d random test points from workspace\n', n_tests);
fprintf('Test point range:\n');
fprintf('  X: [%.1f, %.1f] mm\n', min(p_test(:,1)), max(p_test(:,1)));
fprintf('  Y: [%.1f, %.1f] mm\n', min(p_test(:,2)), max(p_test(:,2)));
fprintf('  Z: [%.1f, %.1f] mm\n\n', min(p_test(:,3)), max(p_test(:,3)));

%% 3. Test analytical IK accuracy

fprintf('3. Testing Analytical IK Solution:\n');
err_an = [];
valid_an = 0;

for i = 1:n_tests
    % Solve with analytical IK
    q_solved_an = dobot3R_ik_serial(p_test(i,:), robot, 'elbow', 'up');
    
    % Verify via forward kinematics
    T_check = robot.fkine(q_solved_an);
    p_check = transl(T_check);
    
    err = norm(p_check - p_test(i,:));
    err_an = [err_an; err];
    
    if err < 1e-3  % Less than 1 micrometer error
        valid_an = valid_an + 1;
    end
end

fprintf('   Mean position error:  %.3e mm\n', mean(err_an));
fprintf('   Max position error:   %.3e mm\n', max(err_an));
fprintf('   Std dev:              %.3e mm\n', std(err_an));
fprintf('   Valid solutions:      %d/%d (%.1f%%)\n\n', valid_an, n_tests, 100*valid_an/n_tests);

%% 4. Compare with numerical ikcon

fprintf('4. Comparing with Numerical IK (ikcon):\n');
err_ikon = [];
time_ikon = [];
valid_ikon = 0;

for i = 1:n_tests
    % Solve with numerical IK
    tic;
    q_solved_ikon = robot.ikcon(T_test(i,:), q_test(i,:));
    t_ikon = toc;
    
    % Verify via forward kinematics
    T_check = robot.fkine(q_solved_ikon);
    p_check = transl(T_check);
    
    err = norm(p_check - p_test(i,:));
    err_ikon = [err_ikon; err];
    time_ikon = [time_ikon; t_ikon];
    
    if err < 1e-3
        valid_ikon = valid_ikon + 1;
    end
end

fprintf('   Mean position error:  %.3e mm\n', mean(err_ikon));
fprintf('   Max position error:   %.3e mm\n', max(err_ikon));
fprintf('   Std dev:              %.3e mm\n', std(err_ikon));
fprintf('   Valid solutions:      %d/%d (%.1f%%)\n\n', valid_ikon, n_tests, 100*valid_ikon/n_tests);

%% 5. Performance comparison

fprintf('5. Computation Time Analysis:\n\n');

time_an = [];
for i = 1:n_tests
    tic;
    q_solved_an = dobot3R_ik_serial(p_test(i,:), robot, 'elbow', 'up');
    t_an = toc;
    time_an = [time_an; t_an];
end

fprintf('   Analytical IK:\n');
fprintf('      Mean: %.3e seconds (%.1f μs)\n', mean(time_an), mean(time_an)*1e6);
fprintf('      Min:  %.3e seconds (%.2f μs)\n', min(time_an), min(time_an)*1e6);
fprintf('      Max:  %.3e seconds (%.2f μs)\n\n', max(time_an), max(time_an)*1e6);

fprintf('   Numerical ikcon:\n');
fprintf('      Mean: %.3e seconds (%.1f ms)\n', mean(time_ikon), mean(time_ikon)*1e3);
fprintf('      Min:  %.3e seconds (%.2f ms)\n', min(time_ikon), min(time_ikon)*1e3);
fprintf('      Max:  %.3e seconds (%.2f ms)\n\n', max(time_ikon), max(time_ikon)*1e3);

speedup = mean(time_ikon) / mean(time_an);
fprintf('   Speed improvement: %.0f× faster\n', speedup);
fprintf('   Time reduction: %.1f%%\n\n', (1 - mean(time_an)/mean(time_ikon))*100);

%% 6. Accuracy comparison

fprintf('6. Accuracy Ranking:\n\n');

if mean(err_an) < mean(err_ikon)
    fprintf('   ✓ Analytical IK achieves better accuracy\n');
    fprintf('     Error reduction: %.2f%% lower\n\n', ...
        (mean(err_ikon) - mean(err_an))/mean(err_ikon)*100);
else
    fprintf('   ⚠ Numerical IK is more accurate\n');
    fprintf('     Error ratio: %.2f%% higher\n\n', ...
        (mean(err_an) - mean(err_ikon))/mean(err_ikon)*100);
end

%% 7. Recommendation

fprintf('7. Recommendation:\n\n');

if speedup > 100 && mean(err_an) < mean(err_ikon)
    fprintf('   ⭐ Use ANALYTICAL IK for optimal performance\n');
    fprintf('      - %d× faster than ikcon\n', round(speedup));
    fprintf('      - Comparable or better accuracy\n');
    fprintf('      - Deterministic computation time\n\n');
elseif speedup > 100
    fprintf('   ✓ Use ANALYTICAL IK with branch selection\n');
    fprintf('      - %d× faster than ikcon\n', round(speedup));
    fprintf('      - May need accuracy post-processing\n\n');
else
    fprintf('   ! Use NUMERICAL IK for critical applications\n');
    fprintf('      - More robust to numerical issues\n');
    fprintf('      - Better accuracy guarantee\n\n');
end

%% 8. Visualization

figure('Name', 'IK Comparison Analysis', 'Position', [100, 100, 1200, 900]);

% Accuracy comparison
subplot(2,3,1);
scatter(1:n_tests, err_an*1000, 60, 'b', 'filled', 'DisplayName', 'Analytical IK'); 
hold on;
scatter(1:n_tests, err_ikon*1000, 60, 'r^', 'filled', 'DisplayName', 'ikcon');
grid on; 
xlabel('Test Point Index', 'FontSize', 11);
ylabel('Position Error [μm]', 'FontSize', 11);
title('Position Error for Each Test Point', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Time comparison
subplot(2,3,2);
plot(time_an*1e6, 'bo', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Analytical IK'); 
hold on;
plot(time_ikon*1e3, 'r^', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'ikcon');
grid on; 
xlabel('Test Point Index', 'FontSize', 11);
ylabel('Time', 'FontSize', 11);
title('Computation Time Comparison', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'YScale', 'log');
legend('Location', 'best');

% Error histogram
subplot(2,3,3);
histogram(err_an*1000, 8, 'FaceAlpha', 0.6, 'DisplayName', 'Analytical IK'); 
hold on;
histogram(err_ikon*1000, 8, 'FaceAlpha', 0.6, 'DisplayName', 'ikcon');
grid on; 
xlabel('Position Error [μm]', 'FontSize', 11);
ylabel('Frequency', 'FontSize', 11);
title('Error Distribution Histogram', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Time histogram
subplot(2,3,4);
histogram(time_an*1e6, 8, 'FaceAlpha', 0.6, 'DisplayName', 'Analytical IK'); 
hold on;
histogram(time_ikon*1e3, 8, 'FaceAlpha', 0.6, 'DisplayName', 'ikcon');
grid on; 
xlabel('Time', 'FontSize', 11);
ylabel('Frequency', 'FontSize', 11);
title('Time Distribution Histogram', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'XScale', 'log');
legend('Location', 'best');

% Summary statistics table
subplot(2,3,5);
axis off;
data = [mean(err_an)*1000, mean(err_ikon)*1000;
        std(err_an)*1000,  std(err_ikon)*1000;
        max(err_an)*1000,  max(err_ikon)*1000;
        mean(time_an)*1e6, mean(time_ikon)*1e3;
        speedup, 1];

tbl = table(data(:,1), data(:,2), ...
    'VariableNames', {'Analytical IK', 'ikcon'}, ...
    'RowNames', {'Mean Error (μm)', 'Std Dev (μm)', 'Max Error (μm)', ...
                 'Mean Time (μs/ms)', 'Speedup Factor'});

t = uitable('Parent', gca, 'Data', tbl{:,:}, ...
    'ColumnName', tbl.Properties.VariableNames, ...
    'RowName', tbl.Properties.RowNames, ...
    'Units', 'Normalized', 'Position', [0, 0, 1, 1]);
t.ColumnWidth = {100, 100};

% Scatter plot in 3D workspace
subplot(2,3,6);
scatter3(p_test(:,1), p_test(:,2), p_test(:,3), 100, err_an*1000, ...
    'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
colormap('jet');
c = colorbar;
xlabel('X [mm]', 'FontSize', 11);
ylabel('Y [mm]', 'FontSize', 11);
zlabel('Z [mm]', 'FontSize', 11);
title('Error Distribution in Workspace', 'FontSize', 12, 'FontWeight', 'bold');
c.Label.String = 'Error [μm]';
grid on;
view(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOCAL FUNCTION: Analytical IK for Dobot 3R (Position-Only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = dobot3R_ik_serial(p, robot, varargin)
% dobot3R_ik_serial  Closed-form analytical IK for Dobot 3R arm
%
% Syntax:
%   q = dobot3R_ik_serial(p, robot)
%   q = dobot3R_ik_serial(p, robot, 'elbow', 'up')
%
% Inputs:
%   p       - Target TCP position [x, y, z] in mm
%   robot   - Robot object (SerialLink)
%   elbow   - Elbow configuration: 'up' or 'down' (default: 'up')
%
% Output:
%   q       - Joint angles [q1, q2, q3] in radians
%
% Details:
%   Uses geometric solution based on law of cosines for the planar 2R
%   arm (joints 2 and 3), with joint 1 controlling base rotation.

    % Parse input arguments
    p_parser = inputParser;
    addParameter(p_parser, 'elbow', 'up', @(x) ismember(x, {'up', 'down'}));
    parse(p_parser, varargin{:});
    elbow_type = p_parser.Results.elbow;

    % Get link lengths from robot
    L2 = robot.links(2).a;   % 135 mm
    L3 = robot.links(3).a;   % 147 mm
    
    x = p(1); 
    y = p(2); 
    z = p(3);

    % ===== Step 1: Solve q1 (Base rotation) =====
    q1 = atan2(y, x);

    % ===== Step 2: Solve q2, q3 (Planar arm in vertical plane) =====
    r = hypot(x, y);  % Horizontal distance from origin
    
    % Using law of cosines: r^2 + z^2 = L2^2 + L3^2 + 2*L2*L3*cos(q3)
    cos_q3 = (r^2 + z^2 - L2^2 - L3^2) / (2*L2*L3);
    
    % Clamp to valid range [-1, 1] for numerical robustness
    cos_q3 = max(min(cos_q3, 1), -1);

    % Select elbow branch
    if strcmpi(elbow_type, 'up')
        sin_q3 =  sqrt(1 - cos_q3^2);  % Positive q3
    else
        sin_q3 = -sqrt(1 - cos_q3^2);  % Negative q3
    end

    q3 = atan2(sin_q3, cos_q3);

    % Solve q2 using atan2 difference
    A = L2 + L3*cos_q3;
    B = L3*sin_q3;
    q2 = atan2(z, r) - atan2(B, A);

    % ===== Step 3: Apply joint limits =====
    if ~isempty(robot.links(2).qlim)
        q2 = max(min(q2, robot.links(2).qlim(2)), robot.links(2).qlim(1));
    end
    if ~isempty(robot.links(3).qlim)
        q3 = max(min(q3, robot.links(3).qlim(2)), robot.links(3).qlim(1));
    end

    q = [q1, q2, q3];
end
