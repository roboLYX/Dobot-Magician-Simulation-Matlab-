%% Compare_IK_ctraj_jtraj.m
% Dobot 3R：按给定 DH 建模
% 对比：
%   1) jtraj vs ctraj
%   2) ikcon(迭代 DH) vs 解析 IK（几何 + 映射到 DH）
%
% 组合一共 4 套：
%   A1: ikcon + jtraj
%   A2: 解析IK + jtraj
%   B1: ikcon + ctraj
%   B2: 解析IK + ctraj
%
% 依赖：Peter Corke Robotics Toolbox (SerialLink, jtraj, ctraj, transl)

clc; clear; close all;

%% 1. 按你给的 DH 建模（标准 DH）

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% 零位修正 & 关节范围（完全按照你给的设置）
L(2).offset = -pi/2;                 % J2 offset
L(3).offset =  pi/2;                 % J3 offset
L(2).qlim   = [  0,  85] * pi/180;   % J2 [0,85] deg
L(3).qlim   = [-10,  90] * pi/180;   % J3 [-10,90] deg

robot = SerialLink(L, 'name', 'Dobot_3R');

% 底座：向下平移 8mm，使肩关节为世界原点
robot.base = transl(0, 0, -8);

% 提取几何长度，给解析 IK 用
L2 = robot.links(2).a;   % 135 mm
L3 = robot.links(3).a;   % 147 mm

fprintf('机器人模型：\n');
robot

%% 2. 定义 4 个末端目标点（世界系，单位 mm）
% 肩关节在 (0,0,0)，z 向上

P1 = [ 150,   50,  -50];
P2 = [ 150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % P + 单位姿态
end

%% 3. 4 个点的 IK：ikcon vs 解析IK

% ---------- 3.1 ikcon ----------
q_wp_ikon = zeros(4,3);
q_guess   = [0, -60, 60] * pi/180;   % 第一个点初始猜测

q_wp_ikon(1,:) = robot.ikcon(T{1}, q_guess);
for i = 2:4
    q_wp_ikon(i,:) = robot.ikcon(T{i}, q_wp_ikon(i-1,:));
end

% ---------- 3.2 解析 IK ----------
q_wp_an = zeros(4,3);
for i = 1:4
    q_wp_an(i,:) = dobot3R_ik_serial(P_list(i,:), robot, 'elbow', 'up');
end

disp('=== 4 个点的关节角（ikcon, 弧度） ===');
disp(q_wp_ikon);
disp('=== 4 个点的关节角（解析IK, 弧度） ===');
disp(q_wp_an);

disp('=== 4 个点的关节角（ikcon, 角度） ===');
disp(rad2deg(q_wp_ikon));
disp('=== 4 个点的关节角（解析IK, 角度） ===');
disp(rad2deg(q_wp_an));

%% 3.5 4 个点的 FK 误差对比（ikcon vs 解析IK）

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

disp('=== 4 个点末端位置误差（ikcon） ===');
disp(tbl_ikon);
disp('=== 4 个点末端位置误差（解析IK） ===');
disp(tbl_an);

fprintf('ikcon 最大点误差 = %.3f mm\n', max(errN_ikon));
fprintf('解析IK 最大点误差 = %.3f mm\n\n', max(errN_an));

%% 4. 关节空间插值：jtraj（组合 A1, A2）

N_j = 30;   % 每段插值点数

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

% A2: 解析IK + jtraj
q_traj_j_an = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp_an(i,:), q_wp_an(i+1,:), N_j);
    if i == 1
        q_traj_j_an = q_seg;
    else
        q_traj_j_an = [q_traj_j_an; q_seg(2:end,:)];
    end
end

% FK 得到 TCP 轨迹
p_j_ikon = transl(robot.fkine(q_traj_j_ikon));
p_j_an   = transl(robot.fkine(q_traj_j_an));

%% 5. 笛卡尔插值：ctraj（组合 B1, B2）

N_c = 30;   % 每段插值点数

T_traj = [];
for i = 1:3
    T_seg = ctraj(T{i}, T{i+1}, N_c);   % 4x4xN_c
    if i == 1
        T_traj = T_seg;
    else
        T_traj = cat(3, T_traj, T_seg(:,:,2:end));
    end
end
M_c = size(T_traj,3);

% 理想 TCP 轨迹（仅位移）
p_c_ideal = zeros(M_c,3);
for k = 1:M_c
    p_c_ideal(k,:) = transl(T_traj(:,:,k));
end

% B1: ctraj + ikcon
q_traj_c_ikon = zeros(M_c,3);
q_prev = q_wp_ikon(1,:);
for k = 1:M_c
    q_traj_c_ikon(k,:) = robot.ikcon(T_traj(:,:,k), q_prev);
    q_prev = q_traj_c_ikon(k,:);
end
p_c_ikon = transl(robot.fkine(q_traj_c_ikon));

% B2: ctraj + 解析IK
q_traj_c_an = zeros(M_c,3);
for k = 1:M_c
    p_des = p_c_ideal(k,:);
    q_traj_c_an(k,:) = dobot3R_ik_serial(p_des, robot, 'elbow','up');
end
p_c_an = transl(robot.fkine(q_traj_c_an));

% 误差（相对于 ctraj 理想轨迹）
err_c_ikon  = sqrt(sum((p_c_ikon - p_c_ideal).^2, 2));
err_c_an    = sqrt(sum((p_c_an   - p_c_ideal).^2, 2));

fprintf('ctraj+ikcon 最大误差 = %.3f mm, RMSE = %.3f mm\n', ...
    max(err_c_ikon), sqrt(mean(err_c_ikon.^2)));
fprintf('ctraj+解析IK 最大误差 = %.3f mm, RMSE = %.3f mm\n\n', ...
    max(err_c_an), sqrt(mean(err_c_an.^2)));

%% 6. TCP 轨迹对比图

figure;
plot3(p_j_ikon(:,1), p_j_ikon(:,2), p_j_ikon(:,3), '-', 'LineWidth', 1.5); hold on;
plot3(p_j_an(:,1),   p_j_an(:,2),   p_j_an(:,3),   '--', 'LineWidth', 1.5);
plot3(p_c_ikon(:,1), p_c_ikon(:,2), p_c_ikon(:,3), ':', 'LineWidth', 1.5);
plot3(p_c_an(:,1),   p_c_an(:,2),   p_c_an(:,3),   '-.', 'LineWidth', 1.5);

plot3(P_list(:,1),   P_list(:,2),   P_list(:,3), ...
    'ko', 'MarkerSize', 8, 'LineWidth', 2);

grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP 轨迹对比：jtraj vs ctraj, ikcon vs 解析IK');
legend('jtraj + ikcon', 'jtraj + 解析IK', ...
       'ctraj + ikcon', 'ctraj + 解析IK', ...
       '4 个离散点', 'Location','Best');

%% 7. ctraj 情况下的误差曲线（ikcon vs 解析IK）

figure;
plot(err_c_ikon, 'LineWidth', 1.5); hold on;
plot(err_c_an,   'LineWidth', 1.5);
grid on;
xlabel('样本 index');
ylabel('||Δp|| [mm]');
title('ctraj 理想轨迹跟踪误差：ikcon vs 解析IK');
legend('ctraj + ikcon 误差','ctraj + 解析IK 误差','Location','Best');

%% 8. jtraj vs ctraj 在关节空间的差异（用解析IK举例）

t_j = 1:size(q_traj_j_an,1);
t_c = linspace(1, numel(t_j), M_c);   % 简单拉伸对齐时间轴

figure;
for j = 1:3
    subplot(3,1,j);
    plot(t_j, q_traj_j_an(:,j), '-', 'LineWidth', 1.5); hold on;
    plot(t_c, q_traj_c_an(:,j), '--', 'LineWidth', 1.5);
    grid on;
    ylabel(sprintf('q_%d [rad]', j));
    if j == 1
        title('关节角轨迹对比：jtraj vs ctraj（解析IK）');
    end
end
xlabel('时间 index');
legend('jtraj + 解析IK','ctraj + 解析IK','Location','Best');

%% 9. 两段动画：jtraj + 解析IK vs ctraj + 解析IK

figure;
robot.plot(q_traj_j_an, ...
    'trail', {'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R：jtraj + 解析IK 轨迹动画');
grid on;

disp('关闭当前动画窗口后，将播放 ctraj + 解析IK 动画...');
pause;

figure;
robot.plot(q_traj_c_an, ...
    'trail', {'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R：ctraj + 解析IK 轨迹动画');
grid on;

%% =================== 解析 IK（按你的 DH + 限位 + 映射） ===================

function q = dobot3R_ik_serial(p, robot, varargin)
% 解析 IK，几何 3R → 映射到你的 DH 关节角（含 offset、qlim）
% p      : 1x3 [x y z] 末端位置（世界坐标，肩在原点）
% robot  : 你的 SerialLink 机器人
% varargin: 'elbow','up' or 'down'

    x = p(1); y = p(2); z = p(3);

    elbow = 'up';
    if ~isempty(varargin)
        for k = 1:2:length(varargin)
            switch lower(varargin{k})
                case 'elbow'
                    elbow = lower(varargin{k+1});
            end
        end
    end

    % --- 从 robot 中取几何参数 & offset & 限位 ---
    L2 = robot.links(2).a;
    L3 = robot.links(3).a;
    off2 = robot.links(2).offset;
    off3 = robot.links(3).offset;
    qlim2 = robot.links(2).qlim;
    qlim3 = robot.links(3).qlim;

    % --- 1) q1：方位角 ---
    q1 = atan2(y, x);

    % --- 2) 平面 2R 几何 IK 求 q2_eff, q3_eff ---
    r = sqrt(x^2 + y^2);
    D = (r^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3);
    D = min(max(D, -1), 1);

    switch elbow
        case 'up'
            q3_eff = atan2( sqrt(1 - D^2), D );
        case 'down'
            q3_eff = atan2(-sqrt(1 - D^2), D );
        otherwise
            error('elbow must be ''up'' or ''down''.');
    end

    phi = atan2(z, r);
    psi = atan2(L3 * sin(q3_eff), L2 + L3 * cos(q3_eff));
    q2_eff = phi - psi;

    % --- 3) 几何角 → 你的 DH 关节变量 ---
    % 假设：theta2_total = q2_eff, theta3_total = q3_eff
    % 而 theta2_total = q2 + off2, theta3_total = q3 + off3
    % ⇒ q2 = q2_eff - off2, q3 = q3_eff - off3
    q2 = q2_eff - off2;
    q3 = q3_eff - off3;

    % --- 4) 施加关节限位（clamp） ---
    q2 = min(max(q2, qlim2(1)), qlim2(2));
    q3 = min(max(q3, qlim3(1)), qlim3(2));

    q = [q1, q2, q3];
end