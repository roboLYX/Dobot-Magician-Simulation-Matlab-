% 差异：
%   - IK 用解析解（根据几何 3R）
%   - 轨迹在笛卡尔空间用 ctraj 插值
%   - IK 内部手动施加关节限位（q2, q3）

clc; clear; close all;

%% 1. DH 建模（标准 DH）

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% 零位修正 & 关节范围
L(2).offset = -pi/2;                 % 大臂零位修正
L(3).offset =  pi/2;                 % 小臂零位修正
L(2).qlim   = [  0,  85] * pi/180;   % J2
L(3).qlim   = [-10,  90] * pi/180;   % J3

robot = SerialLink(L, 'name', 'Dobot_3R');

% 底座坐标：向下平移 8 mm，使肩关节为世界原点(0,0,0)
robot.base = transl(0, 0, -8);

% 记录两段连杆长度（方便解析几何）
L2 = robot.links(2).a;   % 135 mm
L3 = robot.links(3).a;   % 147 mm

%% 2. 定义 4 个末端目标点（世界坐标，单位 mm）
% 注意：这里的坐标系就是你现在 robot.base 之后的世界系，
%       肩关节在 (0,0,0)，z 轴向上。

P1 = [ 150,   50,  -50];
P2 = [ 150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

% 对应位姿（只给位置，姿态取单位矩阵）
T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));
end

%% 3. 用解析 IK 求 4 个离散点的关节角 q_wp（并施加限位）

q_wp = zeros(4,3);
for i = 1:4
    q_wp(i,:) = dobot3R_ik_serial(P_list(i,:), robot, 'elbow', 'up');
end

disp('=== 4 个点的关节角（弧度） ===');
disp(q_wp);
disp('=== 4 个点的关节角（角度） ===');
disp(rad2deg(q_wp));

%% 3.5 用 robot.fkine 检查 4 个点的 FK 误差（与真实 DH 完全一致）

T_wp = robot.fkine(q_wp);
p_wp = transl(T_wp);   % 4×3

disp('=== 4 个点用 robot.fkine 计算得到的 TCP 位置（mm） ===');
disp(p_wp);

compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

% 误差统计
err_abs_4 = p_wp - P_list;
err_abs_norm_4 = sqrt(sum(err_abs_4.^2, 2));
P_norm_4 = sqrt(sum(P_list.^2, 2));
rel_err_4 = err_abs_norm_4 ./ max(P_norm_4, 1e-6);

fprintf('\n===== 4 个关键点（解析IK + robot.fkine）的误差 =====\n');
for i = 1:4
    fprintf('点 %d：|Δp| = %.3e mm, 相对误差 = %.3e\n', ...
        i, err_abs_norm_4(i), rel_err_4(i));
end
fprintf('最大位置误差 = %.3e mm\n\n', max(err_abs_norm_4));

%% 4. 使用 ctraj 在笛卡尔空间插值（理想 TCP 轨迹）

N = 30;   % 每段插值点数，可调

T_traj_c = [];
for i = 1:3
    T_seg = ctraj(T{i}, T{i+1}, N);   % 4x4xN
    if i == 1
        T_traj_c = T_seg;
    else
        % 拼接时去掉段首重复
        T_traj_c = cat(3, T_traj_c, T_seg(:,:,2:end)); % 4x4x(M+N-1)
    end
end

M_c = size(T_traj_c, 3);

% ctraj 理想 TCP 轨迹（只看位移）
p_traj_c_ideal = zeros(M_c, 3);
for k = 1:M_c
    p_traj_c_ideal(k,:) = transl(T_traj_c(:,:,k));
end

%% 5. 对 ctraj 的每个位置，用解析 IK（带限位）求关节角 -> q_traj_c

q_traj_c = zeros(M_c, 3);
for k = 1:M_c
    p_des = p_traj_c_ideal(k,:);
    q_traj_c(k,:) = dobot3R_ik_serial(p_des, robot, 'elbow', 'up');
end

%% 6. 用 robot.fkine 再算一遍 TCP 轨迹，验证误差

T_fk_c = robot.fkine(q_traj_c);
p_fk_c = transl(T_fk_c);   % M_c×3

err_c = p_fk_c - p_traj_c_ideal;
err_c_norm = sqrt(sum(err_c.^2, 2));

fprintf('===== ctraj 理想 TCP vs 解析IK+robot.FK 实际 TCP 的误差 =====\n');
fprintf('最大位置误差范数 = %.3e mm\n', max(err_c_norm));
fprintf('均方根误差(RMSE) = %.3e mm\n\n', sqrt(mean(err_c_norm.^2)));

%% 7. 轨迹对比：理想 ctraj vs 实际 FK + 四个关键点

figure;
plot3(p_traj_c_ideal(:,1), p_traj_c_ideal(:,2), p_traj_c_ideal(:,3), ...
    '-', 'LineWidth', 1.5); hold on;
plot3(p_fk_c(:,1), p_fk_c(:,2), p_fk_c(:,3), ...
    '--', 'LineWidth', 1.5);
plot3(P_list(:,1), P_list(:,2), P_list(:,3), ...
    'ko', 'MarkerSize', 8, 'LineWidth', 2);

grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('解析IK + ctraj：理想 TCP 轨迹 vs 实际 TCP 轨迹（按你的 DH）');
legend('ctraj 理想 TCP','解析IK+robot.fkine 实际 TCP','4 个关键点','Location','Best');

%% 8. 位置误差曲线

figure;
plot(err_c_norm, 'LineWidth', 1.5);
grid on;
xlabel('样本 index');
ylabel('||Δp|| [mm]');
title('解析IK + ctraj：TCP 位置误差范数（按你的 DH）');

%% 9. 关节角轨迹 & 关节角速度（可以看到是否撞限位）

t = (0:M_c-1)';   % 用样本 index 模拟时间

figure;
subplot(3,1,1);
plot(t, q_traj_c(:,1), 'LineWidth', 1.5); grid on;
ylabel('q_1 [rad]');
title('关节角轨迹（解析IK + ctraj）');

subplot(3,1,2);
plot(t, q_traj_c(:,2), 'LineWidth', 1.5); grid on;
ylabel('q_2 [rad]');

subplot(3,1,3);
plot(t, q_traj_c(:,3), 'LineWidth', 1.5); grid on;
xlabel('样本 index'); ylabel('q_3 [rad]');

% 角速度
dt = 1;   % 如果有真实采样时间，可替换
qd_traj_c = diff(q_traj_c) / dt;
t_qd = t(1:end-1);

figure;
subplot(3,1,1);
plot(t_qd, qd_traj_c(:,1), 'LineWidth', 1.5); grid on;
ylabel('\omega_1 [rad/s]');
title('关节角速度（解析IK + ctraj）');

subplot(3,1,2);
plot(t_qd, qd_traj_c(:,2), 'LineWidth', 1.5); grid on;
ylabel('\omega_2 [rad/s]');

subplot(3,1,3);
plot(t_qd, qd_traj_c(:,3), 'LineWidth', 1.5); grid on;
xlabel('样本 index'); ylabel('\omega_3 [rad/s]');

%% 10. 动画演示（完全用你的 robot 模型 + qlim）

figure;
robot.plot(q_traj_c, ...
    'trail', {'r', 'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R：解析IK + ctraj 轨迹动画（按你的 DH & 限位）');
grid on;

%% ================= 局部函数：解析 IK =================
%
% 几何假设：
%   - 肩关节在世界原点 (0,0,0)，z 向上（你 base 已经这么设）
%   - q1 为绕 z 的偏航角 → atan2(y,x)
%   - 在"r-z" 平面上（r = sqrt(x^2+y^2)），是一个 2R 连杆：
%         L2, L3
%   - 解析 IK 先求几何角 q2_eff, q3_eff（物理平面角），
%     再通过 offset 映射到 DH 关节变量 q2, q3：
%         theta2_total = q2 + offset2 = q2 - pi/2 ≈ q2_eff
%         theta3_total = q3 + offset3 = q3 + pi/2 ≈ q3_eff
%     ⇒ q2 = q2_eff + pi/2
%        q3 = q3_eff - pi/2
%   - 最后将 q2,q3 clamp 到 L(2).qlim, L(3).qlim

function q = dobot3R_ik_serial(p, robot, varargin)
    % p: [x y z] 末端在世界坐标下的位置（单位 mm）
    % robot: SerialLink（按你给的 DH）
    % 可选参数：'elbow','up' or 'down'

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

    L2 = robot.links(2).a;
    L3 = robot.links(3).a;

    % ---- 1) q1：绕 z 轴的方位角 ----
    q1 = atan2(y, x);

    % ---- 2) 在平面 (r,z) 上做 2R 解析 IK ----
    r = sqrt(x^2 + y^2);  % 水平距离

    D = (r^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3);
    D = min(max(D, -1), 1);   % 数值截断，避免超出 [-1,1]

    switch elbow
        case 'up'
            q3_eff = atan2( sqrt(1 - D^2), D );   % 上肘
        case 'down'
            q3_eff = atan2(-sqrt(1 - D^2), D );   % 下肘
        otherwise
            error('elbow must be ''up'' or ''down''.');
    end

    phi = atan2(z, r);
    psi = atan2(L3 * sin(q3_eff), L2 + L3 * cos(q3_eff));
    q2_eff = phi - psi;

    % ---- 3) 将几何角(q2_eff, q3_eff)映射到你的 DH 关节变量 q2,q3 ----
    offset2 = robot.links(2).offset;   % -pi/2
    offset3 = robot.links(3).offset;   %  pi/2

    % 几何假设：theta2_total ≈ q2_eff, theta3_total ≈ q3_eff
    %   theta2_total = q2 + offset2
    %   theta3_total = q3 + offset3
    % ⇒ q2 = q2_eff - offset2
    %   q3 = q3_eff - offset3

    q2 = q2_eff - offset2;
    q3 = q3_eff - offset3;

    % ---- 4) 显式施加关节限位（clamp 到 qlim 内）----
    q2_lim = robot.links(2).qlim;
    q3_lim = robot.links(3).qlim;

    q2_before = q2;
    q3_before = q3;

    q2 = min(max(q2, q2_lim(1)), q2_lim(2));
    q3 = min(max(q3, q3_lim(1)), q3_lim(2));

    if abs(q2 - q2_before) > 1e-6 || abs(q3 - q3_before) > 1e-6
        % 如果你想看哪些点撞限位，可以在这里打印信息
        % fprintf('警告：某点解析IK超出关节限位，已夹紧到范围内。\n');
    end

    q = [q1, q2, q3];
end