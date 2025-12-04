%% Dobot 3R Simplified Model: Analytical IK -> Cartesian Trajectory -> FK to Generate End Effector Trajectory
% Recommended Scheme 4: Combines advantages of high precision, fast computation, and precise trajectory
%
% Difference from other schemes:
%   - IK uses analytical solution (based on geometric 3R)
%   - Trajectory interpolated in Cartesian space using ctraj
%   - IK manually applies joint limits (q2, q3)

clc; clear; close all;

%% 1. DH Modeling (Standard DH)

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% Home position correction & joint ranges
L(2).offset = -pi/2;                 % Upper arm home position correction
L(3).offset =  pi/2;                 % Forearm home position correction
L(2).qlim   = [  0,  85] * pi/180;   % J2
L(3).qlim   = [-10,  90] * pi/180;   % J3

robot = SerialLink(L, 'name', 'Dobot_3R');

% Base coordinate: move down 8 mm to make shoulder joint (J2) the world origin (0,0,0)
robot.base = transl(0, 0, -8);

% Record two link lengths (convenient for analytical geometry)
L2 = robot.links(2).a;   % 135 mm
L3 = robot.links(3).a;   % 147 mm

%% 2. Define 4 end effector target points (world coordinate, unit mm)
% Note: This coordinate system is after robot.base transformation,
%       shoulder joint at (0,0,0), z-axis pointing up.

P1 = [ 150,   50,  -50];
P2 = [ 150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

% Corresponding poses (position only, orientation as identity matrix)
T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));
end

%% 3. Use analytical IK to solve joint angles q_wp for 4 discrete points (with limits applied)

q_wp = zeros(4,3);
for i = 1:4
    q_wp(i,:) = dobot3R_ik_serial(P_list(i,:), robot, 'elbow', 'up');
end

disp('=== Joint angles for 4 points (radians) ===');
disp(q_wp);
disp('=== Joint angles for 4 points (degrees) ===');
disp(rad2deg(q_wp));

%% 3.5 Use robot.fkine to check FK error for 4 points (completely consistent with real DH)

T_wp = robot.fkine(q_wp);
p_wp = transl(T_wp);   % 4×3

disp('=== TCP positions calculated by robot.fkine for 4 points (mm) ===');
disp(p_wp);

compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

% Error statistics
err_abs_4 = p_wp - P_list;
err_abs_norm_4 = sqrt(sum(err_abs_4.^2, 2));
P_norm_4 = sqrt(sum(P_list.^2, 2));
rel_err_4 = err_abs_norm_4 ./ max(P_norm_4, 1e-6);

fprintf('\n===== Error for 4 Key Points (Analytical IK + robot.fkine) =====\n');
for i = 1:4
    fprintf('Point %d: |Δp| = %.3e mm, Relative error = %.3e\n', ...
        i, err_abs_norm_4(i), rel_err_4(i));
end
fprintf('Maximum position error = %.3e mm\n\n', max(err_abs_norm_4));

%% 4. Use ctraj to interpolate in Cartesian space (ideal TCP trajectory)

N = 30;   % Number of interpolation points per segment, adjustable

T_traj_c = [];
for i = 1:3
    T_seg = ctraj(T{i}, T{i+1}, N);   % 4x4xN
    if i == 1
        T_traj_c = T_seg;
    else
        % Concatenate while removing duplicate at segment start
        T_traj_c = cat(3, T_traj_c, T_seg(:,:,2:end)); % 4x4x(M+N-1)
    end
end

% Extract ideal Cartesian trajectory TCP positions
M_c = size(T_traj_c, 3);
p_traj_c_ideal = zeros(M_c, 3);
for k = 1:M_c
    p_traj_c_ideal(k,:) = transl(T_traj_c(:,:,k));
end

fprintf('Ideal Cartesian trajectory has %d points\n', M_c);

%% 5. Solve joint angles for each Cartesian point using analytical IK

q_traj_c = zeros(M_c, 3);
q_prev = q_wp(1,:);   % Use first point as reference

for k = 1:M_c
    p_k = p_traj_c_ideal(k,:);   % Position of k-th point
    % Solve analytical IK for this point
    q_traj_c(k,:) = dobot3R_ik_serial(p_k, robot, 'elbow', 'up');
    q_prev = q_traj_c(k,:);
end

%% 6. Perform FK on all joint solutions to get actual TCP trajectory

T_fk_c = robot.fkine(q_traj_c);
p_fk_c = transl(T_fk_c);   % M_c × 3, actual TCP trajectory

%% 7. Error analysis: ideal Cartesian trajectory vs actual FK trajectory

err_c = p_fk_c - p_traj_c_ideal;          % (Δx,Δy,Δz) for each point
err_c_norm = sqrt(sum(err_c.^2, 2));      % Euclidean distance
err_c_max = max(err_c_norm);
err_c_mean = mean(err_c_norm);

fprintf('===== Error Analysis: ctraj ideal vs actual FK =====\n');
fprintf('Maximum position error: %.3f mm\n', err_c_max);
fprintf('Mean position error: %.3f mm\n', err_c_mean);
fprintf('RMS position error: %.3f mm\n\n', sqrt(mean(err_c_norm.^2)));

%% 8. Visualization

% 8.1 End effector trajectory comparison
figure('Name', 'End Effector Trajectory');
plot3(p_fk_c(:,1), p_fk_c(:,2), p_fk_c(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(p_traj_c_ideal(:,1), p_traj_c_ideal(:,2), p_traj_c_ideal(:,3), 'r--', 'LineWidth', 1.5);
plot3(P_list(:,1), P_list(:,2), P_list(:,3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('End Effector Trajectory: Analytical IK + ctraj');
legend('Actual FK', 'Ideal ctraj', '4 Target Points', 'Location', 'Best');

% 8.2 Position error along trajectory
figure('Name', 'Position Error');
plot(err_c_norm, 'LineWidth', 1.5); grid on;
xlabel('Sample Index');
ylabel('Error Norm ||Δp|| [mm]');
title('Position Error: Ideal Cartesian Trajectory vs Actual FK');

% 8.3 Joint angle trajectories
figure('Name', 'Joint Trajectories');
for j = 1:3
    subplot(3,1,j);
    plot(q_traj_c(:,j), 'LineWidth', 1.5); grid on;
    ylabel(sprintf('q_%d [rad]', j));
    if j == 1
        title('Joint Angle Trajectories (Analytical IK)');
    end
end
xlabel('Sample Index');

%% 9. Robot animation

figure('Name', 'Robot Animation');
robot.plot(q_traj_c, ...
    'trail', {'r', 'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R: Analytical IK + ctraj Trajectory (Scheme 4 - Recommended)');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local function: Analytical IK for Dobot 3R 3R (position-only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = dobot3R_ik_serial(p, robot, varargin)
% dobot3R_ik_serial  Closed-form analytical IK for Dobot 3R (position-only constraint)
%
% Input:
%   p     : End effector position [x y z] in mm
%   robot : SerialLink robot object
%   varargin: Optional name-value pairs, e.g., 'elbow', 'up' or 'down'
%
% Output:
%   q     : Joint angles [q1 q2 q3] in radians

    % Parse input
    p_Def = inputParser;
    addParameter(p_Def, 'elbow', 'up', @(x) ismember(x, {'up', 'down'}));
    parse(p_Def, varargin{:});
    elbow_type = p_Def.Results.elbow;

    % Get link lengths
    L2 = robot.links(2).a;   % 135 mm
    L3 = robot.links(3).a;   % 147 mm
    
    x = p(1); y = p(2); z = p(3);

    % === Solve q1 (base orientation) ===
    q1 = atan2(y, x);

    % === Solve q2, q3 in (r, z) plane ===
    r = hypot(x, y);
    d = hypot(r, z);

    % Cosine law
    cos_q3 = (r^2 + z^2 - L2^2 - L3^2) / (2*L2*L3);
    cos_q3 = max(min(cos_q3, 1), -1);  % Clamp [-1, 1]

    % Two branches
    if strcmpi(elbow_type, 'up')
        sin_q3 =  sqrt(1 - cos_q3^2);   % Elbow up
    else
        sin_q3 = -sqrt(1 - cos_q3^2);   % Elbow down
    end

    q3 = atan2(sin_q3, cos_q3);

    % Solve q2
    A = L2 + L3*cos_q3;
    B = L3*sin_q3;
    q2 = atan2(z, r) - atan2(B, A);

    % Apply joint limits if available
    if ~isempty(robot.links(2).qlim)
        q2 = max(min(q2, robot.links(2).qlim(2)), robot.links(2).qlim(1));
    end
    if ~isempty(robot.links(3).qlim)
        q3 = max(min(q3, robot.links(3).qlim(2)), robot.links(3).qlim(1));
    end

    q = [q1, q2, q3];
end
