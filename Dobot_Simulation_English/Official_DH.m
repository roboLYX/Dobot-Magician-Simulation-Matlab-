%% Dobot 3R Simplified Model: IK -> Joint Trajectory -> FK to Generate End Effector Trajectory
% Workflow:
% 1) Known 4 end effector poses P1~P4
% 2) Use inverse kinematics ikcon to solve joint angles q_wp(i,:) for each point
% 3) Perform FK on these joint angles to verify if key points reach targets
% 4) Use jtraj interpolation on q_wp in joint space to get q_traj
% 5) Perform FK on entire q_traj to get continuous end effector trajectory and plot position & joint angular velocity

clc; clear; close all;

%% 1. Build 3R robotic arm with given DH parameters (Standard DH)
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
% Verify standard DH through alpha angle and a in diagram
L(2).offset = -pi/2;  % Correct simulation home position to real robot home position: in simulation zero position is when upper arm is parallel to x-y plane, but in real robot it's when upper arm is vertical
% Q: Why does coordinate system change but home position is different?
% A: In simulation the home position is code-defined (by default aligned with positive x-axis), but in real robot it may differ, so home position needs to be explicitly defined
% Joint ranges (reference manual: upper arm 0~85°, forearm -10~90°)
L(3).offset = pi/2;
L(2).qlim = [  0,  85] * pi/180;   % J2: Upper arm
L(3).qlim = [-10, 90] * pi/180;   % J3: Forearm
% Limit joint ranges to constrain IK solutions to be more consistent with real robot
robot = SerialLink(L, 'name', 'Dobot_3R');
% === Base coordinate: move entire robot down 8 mm to make shoulder joint (J2) the world origin (0,0,0) ===
robot.base = transl(0, 0, -8);
% Define world coordinate - by default world coordinate is frame {0}, but in Dobot they are different.
% According to official manual, world coordinate system is at the second joint, so we need to explicitly declare this.
% teach(robot);   % Uncomment when interactive adjustment is needed
% Verification: position of shoulder joint in DH-0 coordinate system and world coordinate system
% q0 = [0 0 0];
% T1_DH    = robot.A(1, q0);                 % Only DH chain A1
% p1_DH    = transl(T1_DH);                  % Position of shoulder joint in DH-0 coordinates
% T1_world = robot.base * T1_DH;             % Multiply base first to get world coordinates
% p1_world = transl(T1_world);               % Position of shoulder joint in world coordinates
% disp('p1_DH    = Position of shoulder joint in DH-0 coordinate system:');
% disp(p1_DH);
% disp('p1_world = Position of shoulder joint in world coordinate system:');
% disp(p1_world);
% Plot initial posture
% figure;
% robot.plot(q0);
% title('Dobot 3R Simplified Model (Shoulder joint as world origin, with TCP offset)');
% grid on;
%% 2. Define 4 end effector target points (TCP in world coordinate Σ0, unit mm)
% Note: These are desired TCP positions, orientation defaults to R = I (identity matrix)

P1 = [150,   50,  -50];    % Pick up eraser
P2 = [150,   50,   50];    % Raise vertically
P3 = [-150, 150,   50];    % Move horizontally
P4 = [-150, 150,   -50];   % Put down eraser

P_list = [P1; P2; P3; P4];

T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % Only specify end effector position
end

%% 3. Inverse Kinematics: Solve joint angles q_wp(i,:) for 4 points

q_wp = zeros(4,3);   % Store joint solutions for 4 points

% Initial guess: an elbow-up posture (not real robot data)
q_guess = [0, -60, 60] * pi/180;

% First point: use q_guess as initial value
q_wp(1,:) = robot.ikcon(T{1}, q_guess);

% Remaining points: use previous solution as initial value to ensure solution continuity & stay in elbow-up branch
for i = 2:4
    q_wp(i,:) = robot.ikcon(T{i}, q_wp(i-1,:));
end

disp('=== Joint angles for 4 points (radians) ===');
disp(q_wp);
disp('=== Joint angles for 4 points (degrees) ===');
disp(rad2deg(q_wp));

%% 3.5 Use FK to verify if 4 discrete points reach target positions

T_wp = robot.fkine(q_wp);     % Perform FK on 4 postures (includes base and tool)
p_wp = transl(T_wp);          % 4×3, each row is TCP position (x,y,z)

disp('=== TCP positions calculated by FK for 4 points (mm) ===');
disp(p_wp);

% Comparison table (desired positions vs FK results)
compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

% 3.6 Error analysis: simulated TCP coordinates vs desired/real robot TCP coordinates
% Absolute error (mm)
err_abs = p_wp - P_list;              % (Δx, Δy, Δz) for each point
err_abs_norm = sqrt(sum(err_abs.^2, 2));  % Spatial distance error ||Δp|| for each point

% Norm of desired coordinates (mm) for calculating relative error
P_norm = sqrt(sum(P_list.^2, 2));

% Relative error (by Euclidean distance): ||Δp|| / ||p_des||
rel_err = err_abs_norm ./ max(P_norm, 1e-6);   % Prevent division by zero

% Component-wise relative error if desired
rel_err_xyz = abs(err_abs) ./ max(abs(P_list), 1e-6);

% Error table for easy viewing in command window
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
fprintf('Maximum position Euclidean error = %.3f mm, at point %d.\n', maxErrNorm, idxMax);
fprintf('Corresponding relative error = %.4f\n', rel_err(idxMax));
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

T_fk = robot.fkine(q_traj);   % Perform FK on each frame (includes base and tool)
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
title('TCP True Path (IK -> jtraj -> FK)');
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
title('Dobot 3R Simplified Model: 4-Point Joint Space Trajectory');
grid on;
