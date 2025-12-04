%% Dobot 3R: jtraj joint space interpolation vs ctraj Cartesian space interpolation comparison
% 1) Use ikcon to obtain joint angles for 4 discrete points
% 2) Method A: Use jtraj interpolation in joint space -> TCP trajectory A
% 3) Method B: Use ctraj interpolation in Cartesian space -> Perform ikcon on each frame -> TCP trajectory B
% 4) Compare end effector trajectories and joint trajectories of both methods (graphs + animation)

clc; clear; close all;

%% 1. Build 3R robotic arm with given DH parameters (use original setup)

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% Home position correction & joint ranges
L(2).offset = -pi/2;          % Upper arm home position correction
L(3).offset = pi/2;           % Forearm home position correction
L(2).qlim = [  0,  85] * pi/180;   % J2
L(3).qlim = [-10, 90] * pi/180;   % J3

robot = SerialLink(L, 'name', 'Dobot_3R');

% Base coordinate: move down 8 mm to make shoulder joint the world origin
robot.base = transl(0, 0, -8);

%% 2. Define 4 end effector target points (position only, orientation defaults to R=I)

P1 = [ 150,   50,  -50];
P2 = [ 150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % Only specify position, orientation is identity matrix
end

%% 3. Use ikcon to obtain discrete joint solutions q_wp for 4 points

q_wp = zeros(4,3);
q_guess = [0, -60, 60] * pi/180;   % Initial guess: elbow-up posture

% First point uses preset initial value
q_wp(1,:) = robot.ikcon(T{1}, q_guess);

% Later points use previous solution as initial value to ensure solution continuity
for i = 2:4
    q_wp(i,:) = robot.ikcon(T{i}, q_wp(i-1,:));
end

disp('=== Joint angles for 4 points (radians) ===');
disp(q_wp);
disp('=== Joint angles for 4 points (degrees) ===');
disp(rad2deg(q_wp));

%% 3.5 Use FK to check FK error for 4 discrete points (error caused by ikcon)

T_wp = robot.fkine(q_wp);   % FK for 4 postures (includes base)
p_wp = transl(T_wp);        % 4×3, TCP positions

disp('=== TCP positions calculated by FK for 4 points (mm) ===');
disp(p_wp);

compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

% Error analysis
err_abs = p_wp - P_list;
err_abs_norm = sqrt(sum(err_abs.^2, 2));
P_norm = sqrt(sum(P_list.^2, 2));
rel_err = err_abs_norm ./ max(P_norm, 1e-6);
rel_err_xyz = abs(err_abs) ./ max(abs(P_list), 1e-6);

err_tbl = table( ...
    P_list(:,1), P_list(:,2), P_list(:,3), ...
    p_wp(:,1),   p_wp(:,2),   p_wp(:,3),   ...
    err_abs(:,1), err_abs(:,2), err_abs(:,3), ...
    rel_err_xyz(:,1), rel_err_xyz(:,2), rel_err_xyz(:,3), ...
    err_abs_norm, rel_err, ...
    'VariableNames', { ...
        'Px_des','Py_des','Pz_des', ...
        'Px_fk','Py_fk','Pz_fk', ...
        'Ex','Ey','Ez', ...
        'RelEx','RelEy','RelEz', ...
        'ErrNorm','RelErrNorm' ...
    });

disp('=== Absolute and relative error for end effector position at each point (caused by ikcon) ===');
disp(err_tbl);

[maxErrNorm, idxMax] = max(err_abs_norm);
fprintf('Maximum position Euclidean error = %.3f mm, at point %d.\n', maxErrNorm, idxMax);
fprintf('Corresponding relative error = %.4f\n', rel_err(idxMax));

%% 4. Method A: Joint space interpolation jtraj (use original method)

N = 20;   % Number of interpolation points per segment

q_traj_j = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp(i,:), q_wp(i+1,:), N);
    if i == 1
        q_traj_j = q_seg;
    else
        q_traj_j = [q_traj_j; q_seg(2:end,:)];  % Remove duplicate
    end
end

% FK to get TCP trajectory (Method A)
T_fk_j = robot.fkine(q_traj_j);
p_fk_j = transl(T_fk_j);   % M_A × 3

%% 5. Method B: Cartesian space interpolation ctraj + ikcon

% 5.1 First generate T trajectory in Cartesian space (ideal TCP trajectory, accurately passing through four points)
T_traj_c = [];   % Will store as 4x4xM 3D array

for i = 1:3
    % Interpolate N steps from T{i} to T{i+1}
    T_seg = ctraj(T{i}, T{i+1}, N);   % Size: 4x4xN

    if i == 1
        % First segment: direct assignment
        T_traj_c = T_seg;             % 4x4xN
    else
        % Later segments: concatenate along 3rd dimension, remove duplicate at segment start
        % T_seg(:,:,2:end) is "frames 2 to N"
        T_traj_c = cat(3, T_traj_c, T_seg(:,:,2:end));  % 4x4x(M+N-1)
    end
end

% 5.2 Solve joint angles from each frame of T_traj_c using ikcon → q_traj_c
M_c = size(T_traj_c, 3);     % How many frames in total
q_traj_c = zeros(M_c, 3);

% Initial guess: use first discrete point q_wp(1,:)
q_prev = q_wp(1,:);

for k = 1:M_c
    % Homogeneous matrix of k-th frame is T_traj_c(:,:,k)
    Tk = T_traj_c(:,:,k);

    % Use previous frame's solution as initial value to ensure branch continuity
    q_traj_c(k,:) = robot.ikcon(Tk, q_prev);
    q_prev = q_traj_c(k,:);
end

% 5.3 Use FK to check actual TCP trajectory of "ctraj + ikcon"
T_fk_c = robot.fkine(q_traj_c);
p_fk_c = transl(T_fk_c);   % M_c × 3

% Also extract "ideal Cartesian interpolation" TCP trajectory (pure transl, no IK)
p_traj_c_ideal = zeros(M_c,3);
for k = 1:M_c
    p_traj_c_ideal(k,:) = transl(T_traj_c(:,:,k));
end

%% 6. Trajectory comparison: jtraj vs ctraj (position)

figure;
plot3(p_fk_j(:,1), p_fk_j(:,2), p_fk_j(:,3), '-', 'LineWidth', 1.5); hold on;
plot3(p_fk_c(:,1), p_fk_c(:,2), p_fk_c(:,3), '--', 'LineWidth', 1.5);
plot3(P_list(:,1), P_list(:,2), P_list(:,3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP Trajectory Comparison: jtraj (joint space) vs ctraj+ikcon (Cartesian space)');
legend('jtraj -> FK','ctraj+ikcon -> FK','4 target points','Location','Best');

%% 7. Compare joint angle trajectories: jtraj vs ctraj+ikcon

% For convenient time axis alignment, simply use sample index as "time"
t_j = 1:size(q_traj_j,1);
t_c = 1:size(q_traj_c,1);

figure;
for j = 1:3
    subplot(3,1,j);
    plot(t_j, q_traj_j(:,j), '-', 'LineWidth', 1.5); hold on;
    plot(t_c, q_traj_c(:,j), '--', 'LineWidth', 1.5);
    grid on;
    ylabel(sprintf('q_%d [rad]', j));
    if j == 1
        title('Joint Angle Trajectory Comparison: jtraj vs ctraj+ikcon');
    end
end
xlabel('Sample index');
legend('jtraj','ctraj+ikcon','Location','Best');

%% 8. Additional: Compare error between "ideal Cartesian interpolation" vs "ctraj+ikcon actual TCP"

err_c = p_fk_c - p_traj_c_ideal;          % (Δx,Δy,Δz) for each point
err_c_norm = sqrt(sum(err_c.^2, 2));      % Euclidean distance

figure;
plot(err_c_norm, 'LineWidth', 1.5);
grid on;
xlabel('Sample index');
ylabel('Error norm ||Δp|| [mm]');
title('Position Error: ctraj Ideal TCP vs Actual TCP via ikcon');

fprintf('ctraj+ikcon maximum position error norm = %.3f mm\n', max(err_c_norm));

%% 9. Animation demonstration: robot motion with both interpolation methods

% 9.1 jtraj animation (joint space interpolation)
figure;
robot.plot(q_traj_j, ...
    'trail', {'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R: Joint Space Interpolation jtraj Animation');
grid on;

% Wait for user to close the figure window before starting next animation
disp('Close the jtraj animation window to start ctraj+ikcon animation...');
pause;   % Wait for user to press key or close window

% 9.2 ctraj+ikcon animation (Cartesian path interpolation, then tracked with ikcon)
figure;
robot.plot(q_traj_c, ...
    'trail', {'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R: Cartesian Interpolation ctraj + ikcon Animation');
grid on;
