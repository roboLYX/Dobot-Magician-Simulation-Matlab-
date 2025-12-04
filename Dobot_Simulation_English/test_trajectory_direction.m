%% Test_Trajectory_Direction.m - Verify Trajectory Direction Correctness
% Purpose: Check if generated trajectory moves in the correct direction
% 
% Common issue: If offset parameters are incorrectly set, the robot may
% move in the opposite direction to what was commanded
%
% This script provides diagnostic tools to verify trajectory direction

clc; clear; close all;

%% 1. Build robot

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

%% 2. Single-Joint Direction Test

fprintf('===== Single-Joint Direction Test =====\n\n');

% Test J1 (base rotation)
fprintf('Testing J1 (Base Rotation):\n');
q_test1_j1 = [0,     -60*pi/180, 60*pi/180];
q_test2_j1 = [pi/4,  -60*pi/180, 60*pi/180];

T1_j1 = robot.fkine(q_test1_j1);
T2_j1 = robot.fkine(q_test2_j1);
p1_j1 = transl(T1_j1);
p2_j1 = transl(T2_j1);

dir_j1 = p2_j1 - p1_j1;
fprintf('  q1: 0° → 45° (Δq1 = +45°)\n');
fprintf('  TCP displacement: [%.1f, %.1f, %.1f] mm\n', dir_j1);
fprintf('  Expected: Movement in xy-plane (z should be ~0)\n');
if abs(dir_j1(3)) < 1
    fprintf('  ✓ Direction OK\n\n');
else
    fprintf('  ⚠️ Unexpected z component, check offset!\n\n');
end

% Test J2 (upper arm)
fprintf('Testing J2 (Upper Arm):\n');
q_test1_j2 = [0, -60*pi/180, 60*pi/180];
q_test2_j2 = [0, -45*pi/180, 60*pi/180];

T1_j2 = robot.fkine(q_test1_j2);
T2_j2 = robot.fkine(q_test2_j2);
p1_j2 = transl(T1_j2);
p2_j2 = transl(T2_j2);

dir_j2 = p2_j2 - p1_j2;
fprintf('  q2: -60° → -45° (Δq2 = +15°)\n');
fprintf('  TCP displacement: [%.1f, %.1f, %.1f] mm\n', dir_j2);
fprintf('  ✓ Upper arm rotates, TCP should move\n\n');

% Test J3 (forearm)
fprintf('Testing J3 (Forearm):\n');
q_test1_j3 = [0, -60*pi/180, 60*pi/180];
q_test2_j3 = [0, -60*pi/180, 75*pi/180];

T1_j3 = robot.fkine(q_test1_j3);
T2_j3 = robot.fkine(q_test2_j3);
p1_j3 = transl(T1_j3);
p2_j3 = transl(T2_j3);

dir_j3 = p2_j3 - p1_j3;
fprintf('  q3: 60° → 75° (Δq3 = +15°)\n');
fprintf('  TCP displacement: [%.1f, %.1f, %.1f] mm\n', dir_j3);
fprintf('  ✓ Forearm rotates, TCP should move\n\n');

%% 3. Trajectory Direction Verification

fprintf('===== Trajectory Direction Verification =====\n\n');

P1 = [150,   50,  -50];
P2 = [150,   50,   50];
P3 = [-150, 150,   50];
P4 = [-150, 150,  -50];

P_list = [P1; P2; P3; P4];

T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));
end

% Generate trajectory using ikcon + jtraj
q_guess = [0, -60, 60] * pi/180;
q_wp = zeros(4,3);
q_wp(1,:) = robot.ikcon(T{1}, q_guess);
for i = 2:4
    q_wp(i,:) = robot.ikcon(T{i}, q_wp(i-1,:));
end

N = 20;
q_traj = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp(i,:), q_wp(i+1,:), N);
    if i == 1
        q_traj = q_seg;
    else
        q_traj = [q_traj; q_seg(2:end,:)];
    end
end

T_fk = robot.fkine(q_traj);
p_fk = transl(T_fk);

% Calculate direction vectors
dir_ideal_1to2 = P2 - P1;   % Expected: [0, 0, 100]
dir_fk_1to2 = p_fk(end,:) - p_fk(1,:);

dir_ideal_2to3 = P3 - P2;   % Expected: [-300, 100, 0]
dir_fk_2to3 = p_fk(end,:) - p_fk(length(p_fk)/3*2,:);

fprintf('Segment P1→P2:\n');
fprintf('  Ideal direction:  [%.1f, %.1f, %.1f]\n', dir_ideal_1to2);
fprintf('  Actual direction: [%.1f, %.1f, %.1f]\n', dir_fk_1to2);
dot1 = dot(dir_ideal_1to2, dir_fk_1to2);
if dot1 > 0
    fprintf('  ✓ Correct direction (dot product > 0)\n\n');
else
    fprintf('  ⚠️ REVERSED direction (dot product < 0)\n');
    fprintf('     → Check offset parameters!\n\n');
end

%% 4. Visualization

figure('Name', 'Trajectory Direction Test');

% Plot 3D trajectory
plot3(p_fk(:,1), p_fk(:,2), p_fk(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(P_list(:,1), P_list(:,2), P_list(:,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% Add direction arrows
quiver3(P1(1), P1(2), P1(3), dir_ideal_1to2(1), dir_ideal_1to2(2), dir_ideal_1to2(3), ...
        'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); 

grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('Trajectory Direction Verification');
legend('Actual trajectory', 'Target points', 'Expected direction');

%% 5. Troubleshooting Guide

fprintf('===== Troubleshooting Guide =====\n\n');
fprintf('If trajectory moves in opposite direction:\n');
fprintf('1. Check L(2).offset and L(3).offset values\n');
fprintf('2. Verify signs are correct:\n');
fprintf('   - L(2).offset = -pi/2  (try +pi/2 if reversed)\n');
fprintf('   - L(3).offset =  pi/2  (try -pi/2 if reversed)\n');
fprintf('3. Verify robot.base transformation\n');
fprintf('4. Check that DH parameters match official specifications\n');
fprintf('5. Test single joint rotations to isolate the problem\n\n');
