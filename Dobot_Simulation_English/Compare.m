%% Compare all four schemes: Scheme 1, 2, 3, 4
% Dobot 3R: Build with given DH
% Compare:
%   1) jtraj vs ctraj
%   2) ikcon (iterative DH) vs Analytical IK (geometric + mapped to DH)
%
% Total of 4 combinations:
%   A1: ikcon + jtraj
%   A2: Analytical IK + jtraj
%   B1: ikcon + ctraj
%   B2: Analytical IK + ctraj
%
% Depends: Peter Corke Robotics Toolbox (SerialLink, jtraj, ctraj, transl)

clc; clear; close all;

%% 1. Build robot with given DH (Standard DH)

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% Home position correction & joint ranges (completely following original setup)
L(2).offset = -pi/2;                 % J2 offset
L(3).offset =  pi/2;                 % J3 offset
L(2).qlim   = [  0,  85] * pi/180;   % J2 [0,85] deg
L(3).qlim   = [-10,  90] * pi/180;   % J3 [-10,90] deg

robot = SerialLink(L, 'name', 'Dobot_3R');

% Base: move down 8mm to make shoulder joint the world origin
robot.base = transl(0, 0, -8);

% Extract geometric lengths for analytical IK
L2 = robot.links(2).a;   % 135 mm
L3 = robot.links(3).a;   % 147 mm

fprintf('Robot model:\n');
robot

%% 2. Define 4 end effector target points (world coordinate system, unit mm)
% Shoulder joint at (0,0,0), z pointing up

P1 = [ 150,   50,  -50];
P2 = [ 150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % Position + identity orientation
end

%% 3. Four points IK: ikcon vs Analytical IK

% ---------- 3.1 ikcon ----------
q_wp_ikon = zeros(4,3);
q_guess   = [0, -60, 60] * pi/180;   % First point initial guess

q_wp_ikon(1,:) = robot.ikcon(T{1}, q_guess);
for i = 2:4
    q_wp_ikon(i,:) = robot.ikcon(T{i}, q_wp_ikon(i-1,:));
end

% ---------- 3.2 Analytical IK ----------
q_wp_an = zeros(4,3);
for i = 1:4
    q_wp_an(i,:) = dobot3R_ik_serial(P_list(i,:), robot, 'elbow', 'up');
end

disp('=== Joint angles for 4 points (ikcon, radians) ===');
disp(q_wp_ikon);
disp('=== Joint angles for 4 points (Analytical IK, radians) ===');
disp(q_wp_an);

disp('=== Joint angles for 4 points (ikcon, degrees) ===');
disp(rad2deg(q_wp_ikon));
disp('=== Joint angles for 4 points (Analytical IK, degrees) ===');
disp(rad2deg(q_wp_an));

%% 3.5 Four points FK error comparison (ikcon vs Analytical IK)

T_wp_ikon = robot.fkine(q_wp_ikon);
p_wp_ikon = transl(T_wp_ikon);

T_wp_an   = robot.fkine(q_wp_an);
p_wp_an   = transl(T_wp_an);

err_ikon   = p_wp_ikon - P_list;
err_an     = p_wp_an   - P_list;
errN_ikon  = sqrt(sum(err_ikon.^2, 2));
errN_an    = sqrt(sum(err_an.^2, 2));

tbl_ikon = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                 p_wp_ikon(:,1), p_wp_ikon(:,2), p_wp_ikon(:,3), ...
                 err_ikon(:,1), err_ikon(:,2), err_ikon(:,3), ...
                 errN_ikon, ...
    'VariableNames', {'Px_des','Py_des','Pz_des', ...
                      'Px_fk','Py_fk','Pz_fk', ...
                      'Ex','Ey','Ez','ErrNorm'});
tbl_an = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
               p_wp_an(:,1), p_wp_an(:,2), p_wp_an(:,3), ...
               err_an(:,1), err_an(:,2), err_an(:,3), ...
               errN_an, ...
    'VariableNames', {'Px_des','Py_des','Pz_des', ...
                      'Px_fk','Py_fk','Pz_fk', ...
                      'Ex','Ey','Ez','ErrNorm'});

disp('=== End effector position error for 4 points (ikcon) ===');
disp(tbl_ikon);
disp('=== End effector position error for 4 points (Analytical IK) ===');
disp(tbl_an);

fprintf('ikcon maximum point error = %.3f mm\n', max(errN_ikon));
fprintf('Analytical IK maximum point error = %.3f mm\n\n', max(errN_an));

%% 4. Joint space interpolation: jtraj (Schemes A1, A2)

N_j = 30;   % Number of interpolation points per segment

% A1: ikcon + jtraj
q_traj_j_ikon = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp_ikon(i,:), q_wp_ikon(i+1,:), N_j);
    if i == 1
        q_traj_j_ikon = q_seg;
    else
        q_traj_j_ikon = [q_traj_j_ikon; q_seg(2:end,:)];
    end
end

% A2: Analytical IK + jtraj
q_traj_j_an = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp_an(i,:), q_wp_an(i+1,:), N_j);
    if i == 1
        q_traj_j_an = q_seg;
    else
        q_traj_j_an = [q_traj_j_an; q_seg(2:end,:)];
    end
end

% FK to get TCP trajectories
p_j_ikon = transl(robot.fkine(q_traj_j_ikon));
p_j_an   = transl(robot.fkine(q_traj_j_an));

%% 5. Cartesian space interpolation: ctraj (Schemes B1, B2)

N_c = 30;

% B1: ikcon + ctraj (requires point-wise IK)
T_traj_c = [];
for i = 1:3
    T_seg = ctraj(T{i}, T{i+1}, N_c);
    if i == 1
        T_traj_c = T_seg;
    else
        T_traj_c = cat(3, T_traj_c, T_seg(:,:,2:end));
    end
end

M_c = size(T_traj_c, 3);
q_traj_c_ikon = zeros(M_c, 3);
q_prev = q_wp_ikon(1,:);

for k = 1:M_c
    Tk = T_traj_c(:,:,k);
    q_traj_c_ikon(k,:) = robot.ikcon(Tk, q_prev);
    q_prev = q_traj_c_ikon(k,:);
end

p_c_ikon = transl(robot.fkine(q_traj_c_ikon));

% B2: Analytical IK + ctraj (also requires point-wise IK but faster)
p_traj_c_ideal = zeros(M_c, 3);
for k = 1:M_c
    p_traj_c_ideal(k,:) = transl(T_traj_c(:,:,k));
end

q_traj_c_an = zeros(M_c, 3);
for k = 1:M_c
    q_traj_c_an(k,:) = dobot3R_ik_serial(p_traj_c_ideal(k,:), robot, 'elbow', 'up');
end

p_c_an = transl(robot.fkine(q_traj_c_an));

%% 6. Comprehensive comparison plots

% 6.1 End effector trajectory comparison
figure('Name', 'Trajectory Comparison');
plot3(p_j_ikon(:,1), p_j_ikon(:,2), p_j_ikon(:,3), '-', 'LineWidth', 1.5); hold on;
plot3(p_j_an(:,1), p_j_an(:,2), p_j_an(:,3), '--', 'LineWidth', 1.5);
plot3(p_c_ikon(:,1), p_c_ikon(:,2), p_c_ikon(:,3), '-.', 'LineWidth', 1.5);
plot3(p_c_an(:,1), p_c_an(:,2), p_c_an(:,3), ':', 'LineWidth', 2);
plot3(P_list(:,1), P_list(:,2), P_list(:,3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('End Effector Trajectory: Four Schemes Comparison');
legend('ikcon+jtraj', 'Analytical+jtraj', 'ikcon+ctraj', 'Analytical+ctraj', '4 Targets');

% 6.2 Error summary table
fprintf('\n===== Comprehensive Scheme Comparison =====\n');
fprintf('Scheme 1 (ikcon+jtraj):    Max error = %.3e mm\n', ...
    max(sqrt(sum((p_j_ikon - p_j_ikon([1,1,1,1],:)).^2, 2))));
fprintf('Scheme 2 (Analytical+jtraj): Max error = %.3e mm\n', ...
    max(sqrt(sum((p_j_an - p_j_an([1,1,1,1],:)).^2, 2))));
fprintf('Scheme 3 (ikcon+ctraj):    Max error = %.3e mm\n', ...
    max(sqrt(sum((p_c_ikon - p_traj_c_ideal).^2, 2))));
fprintf('Scheme 4 (Analytical+ctraj): Max error = %.3e mm\n', ...
    max(sqrt(sum((p_c_an - p_traj_c_ideal).^2, 2))));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local function: Analytical IK for Dobot 3R
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = dobot3R_ik_serial(p, robot, varargin)
% dobot3R_ik_serial  Closed-form analytical IK for Dobot 3R (position-only)

    % Parse input
    p_Def = inputParser;
    addParameter(p_Def, 'elbow', 'up', @(x) ismember(x, {'up', 'down'}));
    parse(p_Def, varargin{:});
    elbow_type = p_Def.Results.elbow;

    % Get link lengths
    L2 = robot.links(2).a;   % 135 mm
    L3 = robot.links(3).a;   % 147 mm
    
    x = p(1); y = p(2); z = p(3);

    % Solve q1
    q1 = atan2(y, x);

    % Solve q2, q3
    r = hypot(x, y);
    
    % Cosine law
    cos_q3 = (r^2 + z^2 - L2^2 - L3^2) / (2*L2*L3);
    cos_q3 = max(min(cos_q3, 1), -1);

    % Branch selection
    if strcmpi(elbow_type, 'up')
        sin_q3 =  sqrt(1 - cos_q3^2);
    else
        sin_q3 = -sqrt(1 - cos_q3^2);
    end

    q3 = atan2(sin_q3, cos_q3);

    % Solve q2
    A = L2 + L3*cos_q3;
    B = L3*sin_q3;
    q2 = atan2(z, r) - atan2(B, A);

    % Apply joint limits
    if ~isempty(robot.links(2).qlim)
        q2 = max(min(q2, robot.links(2).qlim(2)), robot.links(2).qlim(1));
    end
    if ~isempty(robot.links(3).qlim)
        q3 = max(min(q3, robot.links(3).qlim(2)), robot.links(3).qlim(1));
    end

    q = [q1, q2, q3];
end
