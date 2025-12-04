%% Dobot 3R Simplified Model: Analytical IK -> Joint Trajectory -> FK to Generate End Effector Trajectory (Position-Only Constraint)
% Workflow:
% 1) Known 4 end effector poses P1~P4 (position only, no orientation)
% 2) Use "analytical IK" to solve joint angles q_wp(i,:) for each point
% 3) Perform FK on these q_wp to verify if key points reach targets with "near-zero error"
% 4) Use jtraj interpolation on q_wp in joint space to get q_traj
% 5) Perform FK on q_traj to get continuous end effector trajectory and plot position & joint angular velocity
%
% Key points:
% - IK only uses position (x,y,z), ignores orientation R
% - IK is closed-form geometric solution for current DH model, theoretically FK error within numerical precision (~1e-13)

clc; clear; close all;

%% 1. Build 3R robotic arm with given DH parameters (Standard DH, consistent with original setup)
% DH table (relative to shoulder joint coordinate system):
% L   theta_i   alpha_i   d_i   a_i
% 1    θ1       -pi/2      8     0
% 2    θ2        0         0    135
% 3    θ3        0         0    147

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% Home position correction (keep consistent with original model)
L(2).offset = -pi/2;   % J2 real robot home position correction
L(3).offset =  pi/2;   % J3 real robot home position correction

% Joint ranges (optional, doesn't affect analytical IK correctness, only affects branch selection)
L(2).qlim = [  0,  85] * pi/180;   % J2: Upper arm
L(3).qlim = [-10, 90] * pi/180;   % J3: Forearm

robot = SerialLink(L, 'name', 'Dobot_3R');

% === Base coordinate: move entire robot down 8 mm to make shoulder joint (J2) the world origin (0,0,0) ===
robot.base = transl(0, 0, -8);

% Initial posture (optional)
q0 = [0 0 0];
% figure; robot.plot(q0); grid on; axis equal;
% title('Dobot 3R Simplified Model (Shoulder joint as world origin)');

%% 2. Define 4 end effector target points (position only, no orientation specification)
% Note: These are desired TCP positions, orientation is flexible (let IK find one)

P1 = [ 150,   50,  -50];    % Pick up eraser
P2 = [ 150,   50,   50];    % Raise vertically
P3 = [-150,  150,   50];    % Move horizontally
P4 = [-150,  150,  -50];    % Put down eraser

P_list = [P1; P2; P3; P4];

% Keep structure consistent here, but IK won't use this T, only uses position P
T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % Only specify end effector position
end

%% 3. Analytical IK: Solve joint angles q_wp(i,:) for 4 points
% Use closed-form geometric solution for current DH model, constraining only (x,y,z)

q_wp = zeros(4,3);   % Store joint solutions for 4 points

% Initial reference posture (approximate elbow-up, used for branch selection/continuity)
q_guess = [0, -60, 60] * pi/180;

for i = 1:4
    if i == 1
        q_ref = q_guess;        % First point references initial value
    else
        q_ref = q_wp(i-1,:);    % Later points reference previous solution for branch continuity
    end

    % Solve analytical IK using only position P_list(i,:)
    q_wp(i,:) = dobot3R_IK_analytic_pos(P_list(i,:), q_ref, robot);
end

disp('=== Joint angles for 4 points (radians) ===');
disp(q_wp);
disp('=== Joint angles for 4 points (degrees) ===');
disp(rad2deg(q_wp));

%% 3.5 Use FK to verify if 4 discrete points reach target positions (theoretically near-zero error within numerical precision)

T_wp = robot.fkine(q_wp);     % Perform FK on 4 postures (includes base)
p_wp = transl(T_wp);          % 4×3, each row is TCP position (x,y,z)

disp('=== TCP positions calculated by FK for 4 points (mm) ===');
disp(p_wp);

% Comparison table (desired positions vs FK results)
compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

%% 3.6 Error analysis: simulated TCP coordinates vs desired TCP coordinates

% Absolute error (mm)
err_abs = p_wp - P_list;              % (Δx, Δy, Δz) for each point
err_abs_norm = sqrt(sum(err_abs.^2, 2));  % Spatial distance error ||Δp|| for each point

% Norm of desired coordinates (mm) for calculating relative error
P_norm = sqrt(sum(P_list.^2, 2));

% Relative error (by Euclidean distance): ||Δp|| / ||p_des||
rel_err = err_abs_norm ./ max(P_norm, 1e-6);   % Prevent division by zero

% Component-wise relative error
rel_err_xyz = abs(err_abs) ./ max(abs(P_list), 1e-6);

% Error table
err_tbl = table( ...
    P_list(:,1), P_list(:,2), P_list(:,3), ...          % Desired coordinates
    p_wp(:,1),   p_wp(:,2),   p_wp(:,3),   ...          % Simulated FK coordinates
    err_abs(:,1), err_abs(:,2), err_abs(:,3), ...       % Absolute error
    rel_err_xyz(:,1), rel_err_xyz(:,2), rel_err_xyz(:,3), ...  % Component-wise relative error
    err_abs_norm, rel_err, ...                          % Euclidean distance & relative error
    'VariableNames', { ...
        'Px_des','Py_des','Pz_des', ...
        'Px_fk','Py_fk','Pz_fk', ...
        'Ex','Ey','Ez', ...
        'RelEx','RelEy','RelEz', ...
        'ErrNorm','RelErrNorm' ...
    });

disp('=== Absolute and relative error of each point end effector position ===');
disp(err_tbl);

% Maximum error
[maxErrNorm, idxMax] = max(err_abs_norm);
fprintf('Maximum position Euclidean error = %.3e mm, at point %d.\n', maxErrNorm, idxMax);
fprintf('Corresponding relative error = %.4e\n', rel_err(idxMax));

%% 4. Joint space interpolation: P1->P2, P2->P3, P3->P4 three segments jtraj

N = 10;   % Number of interpolation points per segment

q_traj = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp(i,:), q_wp(i+1,:), N);
    if i == 1
        q_traj = q_seg;
    else
        q_traj = [q_traj; q_seg(2:end,:)];  % Remove duplicate point
    end
end

%% 5. Calculate entire end effector trajectory using forward kinematics (FK on q_traj)

T_fk = robot.fkine(q_traj);   % Perform FK on each frame (includes base)
p_fk = transl(T_fk);          % M×3, TCP true trajectory

figure;
plot3(p_fk(:,1), p_fk(:,2), p_fk(:,3), 'b-', 'LineWidth', 2); hold on;

% Plot 4 target points
plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P2(1), P2(2), P2(3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P3(1), P3(2), P3(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P4(1), P4(2), P4(3), 'mo', 'MarkerSize', 8, 'LineWidth', 2);

grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP True Path (Analytical IK -> jtraj -> FK)');
legend('TCP trajectory','P1','P2','P3','P4');

%% 6. Calculate and plot joint angular velocity

dt = 1;   % Set time interval between interpolation points to 1 s (shape correct, values can be adjusted)

qd_traj = diff(q_traj) / dt;            % Size: (N_total-1) × 3
t_qd    = (1:size(qd_traj,1)) * dt;     % Corresponding time axis

figure;
subplot(3,1,1);
plot(t_qd, qd_traj(:,1), 'LineWidth', 1.5);
grid on;
ylabel('\omega_1 [rad/s]');
title('Joint 1 Angular Velocity');

subplot(3,1,2);
plot(t_qd, qd_traj(:,2), 'LineWidth', 1.5);
grid on;
ylabel('\omega_2 [rad/s]');
title('Joint 2 Angular Velocity');

subplot(3,1,3);
plot(t_qd, qd_traj(:,3), 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('\omega_3 [rad/s]');
title('Joint 3 Angular Velocity');

sgtitle('Three-Joint Angular Velocity Curves');

%% 7. Animation demonstration of entire motion process

figure;
robot.plot(q_traj, ...
    'trail', {'r', 'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R Simplified Model: 4-Point Joint Space Trajectory (Analytical IK)');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local function: Analytical IK for Dobot 3R (position-only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q_best = dobot3R_IK_analytic_pos(p, q_ref, robot)
% dobot3R_IK_analytic_pos  Analytical inverse solution for current DH model (position-only)
%
% Input:
%   p     : 1x3 position [x y z], unit mm
%   q_ref : 1x3 reference joint angles (previous time or initial value), used for branch selection
%   robot : SerialLink robot (used to get a, qlim, etc.)
%
% Output:
%   q_best: 1x3 joint angle solution [q1 q2 q3] (radians)
%
% Notes:
%   - Analytical solution is derived for the following DH (including base & offset):
%       alpha = [-pi/2, 0, 0];
%       d     = [8, 0, 0];
%       a     = [0, 135, 147];
%       L(2).offset = -pi/2; L(3).offset = pi/2;
%       robot.base = transl(0,0,-8);
%   - Corresponding FK position is:
%       x = (135*sin(q2) + 147*cos(q2+q3)) * cos(q1)
%       y = (135*sin(q2) + 147*cos(q2+q3)) * sin(q1)
%       z = -147*sin(q2+q3) + 135*cos(q2)
%     (here q2,q3 are "code q", already including offset)

    % Get link lengths
    L2 = robot.links(2).a;   % 135
    L3 = robot.links(3).a;   % 147

    x = p(1); y = p(2); z = p(3);

    % Planar radius r and distance d
    r = hypot(x, y);
    d = hypot(r, z);

    % Reachability check (simple triangle inequality version)
    if d > (L2 + L3) + 1e-6 || d < abs(L2 - L3) - 1e-6
        error('Target point (%.1f, %.1f, %.1f) out of workspace, analytical IK has no solution.', x, y, z);
    end

    % === 1) Analytically solve q1 ===
    q1 = atan2(y, x);   % Rotate base to align plane with target projection

    % === 2) Solve 2R in (r,z) plane with geometric analytical solution ===
    % Derived key equation:
    %   z*sin(s) - r*cos(s) = K
    % where s = q2 + q3
    K   = (L2^2 - d^2 - L3^2) / (2*L3);
    R0  = d;                      % Actually sqrt(r^2+z^2)
    arg = K / R0;
    arg = max(min(arg, 1), -1);   % Clamp to prevent numerical issues

    % delta such that:
    %   z*sin(s) - r*cos(s) = R0 * sin(s + delta)
    delta = atan2(-r, z);         % sin(delta) = -r/d, cos(delta)=z/d

    % Two branches: s1, s2
    s1 = -delta + asin(arg);          % Branch 1
    s2 = -delta + (pi - asin(arg));   % Branch 2

    s_list = [s1, s2];
    q_candidates = zeros(2,3);

    for k = 1:2
        s = s_list(k);

        % From:
        %   L2*cos(q2) = z + L3*sin(s)
        %   L2*sin(q2) = r - L3*cos(s)
        cos_q2L2 = z + L3*sin(s);
        sin_q2L2 = r - L3*cos(s);

        q2 = atan2(sin_q2L2, cos_q2L2);
        q3 = s - q2;

        q_candidates(k,:) = [q1, q2, q3];
    end

    % === 3) Branch selection: prioritize satisfying joint limits, then closest to q_ref ===
    valid = true(2,1);
    for k = 1:2
        qk = q_candidates(k,:);

        % J2 limit
        if ~isempty(robot.links(2).qlim)
            if qk(2) < robot.links(2).qlim(1)-1e-6 || qk(2) > robot.links(2).qlim(2)+1e-6
                valid(k) = false;
            end
        end
        % J3 limit
        if ~isempty(robot.links(3).qlim)
            if qk(3) < robot.links(3).qlim(1)-1e-6 || qk(3) > robot.links(3).qlim(2)+1e-6
                valid(k) = false;
            end
        end
    end

    idx_valid = find(valid);
    if isempty(idx_valid)
        % Both branches exceed limits: give warning, still select the closest to q_ref
        warning('Analytical IK: both branches exceed joint limits, still selecting solution closest to reference posture.');
        idx_valid = 1:2;
    end

    % Without reference value, directly take first valid
    if any(isnan(q_ref)) || isempty(q_ref)
        q_best = q_candidates(idx_valid(1),:);
        return;
    end

    % With reference value, select from valid branches with minimum angle distance
    bestIdx  = idx_valid(1);
    bestDist = norm( angdiff_vec(q_ref, q_candidates(idx_valid(1),:)) );

    for ii = 2:numel(idx_valid)
        idx = idx_valid(ii);
        dtmp = norm( angdiff_vec(q_ref, q_candidates(idx,:)) );
        if dtmp < bestDist
            bestDist = dtmp;
            bestIdx  = idx;
        end
    end

    q_best = q_candidates(bestIdx,:);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local function: Element-wise angle difference (result wrapped to [-pi,pi])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function d = angdiff_vec(q1, q2)
% angdiff_vec  Element-wise angle difference, result in [-pi,pi]
    d = atan2(sin(q2 - q1), cos(q2 - q1));
end
