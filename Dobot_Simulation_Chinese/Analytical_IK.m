%% Dobot 3R 简化模型：解析 IK -> 关节轨迹 -> FK 生成末端轨迹（仅位置约束）
% 流程：
% 1) 已知 4 个末端位姿 P1~P4（只用位置，不管姿态）
% 2) 用“解析 IK”求出每个点的关节角 q_wp(i,:)
% 3) 用这些 q_wp 做一次 FK，验证关键点是否“几乎零误差”到位
% 4) 在关节空间对 q_wp 做 jtraj 插值，得到 q_traj
% 5) 对 q_traj 做 FK，得到连续末端轨迹，并绘制位置 & 关节角速度
%
% 关键点：
% - IK 只用位置 (x,y,z)，不约束姿态 R
% - IK 是对当前 DH 模型的闭式几何解，理论上 FK 误差在数值精度内 (~1e-13)

clc; clear; close all;

%% 1. 按给定 DH 建 3R 机械臂（标准 DH，一致复用你原来的设定）
% DH 表（相对于肩关节坐标系）：
% L   theta_i   alpha_i   d_i   a_i
% 1    θ1       -pi/2      8     0
% 2    θ2        0         0    135
% 3    θ3        0         0    147，

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% 零位补偿（保持与你原来的模型一致）
L(2).offset = -pi/2;   % J2 实机零位矫正
L(3).offset =  pi/2;   % J3 实机零位矫正

% 关节范围（可选，不影响解析 IK 的正确性，只影响分支选择）
L(2).qlim = [  0,  85] * pi/180;   % J2：大臂
L(3).qlim = [-10, 90] * pi/180;   % J3：小臂

robot = SerialLink(L, 'name', 'Dobot_3R');

% === 底座坐标：将整机向下平移 8 mm，使肩关节(J2)成为世界原点(0,0,0) ===
robot.base = transl(0, 0, -8);

% 初始姿态（可选）
q0 = [0 0 0];
% figure; robot.plot(q0); grid on; axis equal;
% title('Dobot 3R 简化模型（肩关节为世界原点）');

%% 2. 定义 4 个末端目标点（只用位置，不指定姿态）
% 注意：这些是 TCP 的期望位置，姿态不管（R 随便，让 IK 自己找一个）

P1 = [ 150,   50,  -50];    % 拾取橡皮
P2 = [ 150,   50,   50];    % 垂直抬高
P3 = [-150,  150,   50];    % 水平移动
P4 = [-150,  150,  -50];    % 放下橡皮

P_list = [P1; P2; P3; P4];

% 这里只是留着结构一致，IK 不用这个 T，只用位置 P
T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % 只指定末端位置
end

%% 3. 解析 IK：对 4 个点分别求关节角 q_wp(i,:)
% 使用的是对当前 DH（含 offset & base）的闭式几何解，仅约束 (x,y,z)

q_wp = zeros(4,3);   % 存 4 个点的关节解

% 初始参考姿态（近似上肘姿态，用来选分支/保持连续）
q_guess = [0, -60, 60] * pi/180;

for i = 1:4
    if i == 1
        q_ref = q_guess;        % 第一个点参考初值
    else
        q_ref = q_wp(i-1,:);    % 之后的点参考上一个解，保持分支连续
    end

    % 只用位置 P_list(i,:) 做解析 IK
    q_wp(i,:) = dobot3R_IK_analytic_pos(P_list(i,:), q_ref, robot);
end

disp('=== 4 个点的关节角（弧度） ===');
disp(q_wp);
disp('=== 4 个点的关节角（角度） ===');
disp(rad2deg(q_wp));

%% 3.5 用 FK 检查这 4 个离散点是否基本到达目标位置（理论上数值精度内零误差）

T_wp = robot.fkine(q_wp);     % 对 4 个姿态做 FK（包含 base）
p_wp = transl(T_wp);          % 4×3，每一行为 TCP 的 (x,y,z)

disp('=== 4 个点用 FK 计算得到的 TCP 位置（mm） ===');
disp(p_wp);

% 对比表 (目标位置 vs FK 结果)
compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

%% 3.6 误差分析：仿真 TCP 坐标 vs 期望 TCP 坐标

% 绝对误差（mm）
err_abs = p_wp - P_list;              % 每个点的 (Δx, Δy, Δz)
err_abs_norm = sqrt(sum(err_abs.^2, 2));  % 每个点的空间距离误差 ||Δp||

% 期望坐标的模长（mm），用于算相对误差
P_norm = sqrt(sum(P_list.^2, 2));

% 相对误差（按欧氏距离）： ||Δp|| / ||p_des||
rel_err = err_abs_norm ./ max(P_norm, 1e-6);   % 防止除 0

% 各个坐标分量的相对误差
rel_err_xyz = abs(err_abs) ./ max(abs(P_list), 1e-6);

% 做一个误差表
err_tbl = table( ...
    P_list(:,1), P_list(:,2), P_list(:,3), ...          % 期望坐标
    p_wp(:,1),   p_wp(:,2),   p_wp(:,3),   ...          % 仿真 FK 坐标
    err_abs(:,1), err_abs(:,2), err_abs(:,3), ...       % 绝对误差
    rel_err_xyz(:,1), rel_err_xyz(:,2), rel_err_xyz(:,3), ...  % 各轴相对误差
    err_abs_norm, rel_err, ...                          % 欧氏距离 & 相对误差
    'VariableNames', { ...
        'Px_des','Py_des','Pz_des', ...
        'Px_fk','Py_fk','Pz_fk', ...
        'Ex','Ey','Ez', ...
        'RelEx','RelEy','RelEz', ...
        'ErrNorm','RelErrNorm' ...
    });

disp('=== 各点末端位置的绝对误差 / 相对误差 ===');
disp(err_tbl);

% 最大误差
[maxErrNorm, idxMax] = max(err_abs_norm);
fprintf('最大位置欧氏误差 = %.3e mm，对应第 %d 个点。\n', maxErrNorm, idxMax);
fprintf('对应的相对误差 = %.4e\n', rel_err(idxMax));

%% 4. 关节空间插值：P1->P2, P2->P3, P3->P4 三段 jtraj

N = 10;   % 每段插值点数

q_traj = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp(i,:), q_wp(i+1,:), N);
    if i == 1
        q_traj = q_seg;
    else
        q_traj = [q_traj; q_seg(2:end,:)];  % 去掉重复点
    end
end

%% 5. 用正运动学算整条末端轨迹（FK on q_traj）

T_fk = robot.fkine(q_traj);   % 对每一帧做 FK（已包含 base）
p_fk = transl(T_fk);          % M×3，TCP 真实轨迹

figure;
plot3(p_fk(:,1), p_fk(:,2), p_fk(:,3), 'b-', 'LineWidth', 2); hold on;

% 把 4 个目标点画出来
plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P2(1), P2(2), P2(3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P3(1), P3(2), P3(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
plot3(P4(1), P4(2), P4(3), 'mo', 'MarkerSize', 8, 'LineWidth', 2);

grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP 真实路径（解析 IK -> jtraj -> FK）');
legend('TCP 轨迹','P1','P2','P3','P4');

%% 6. 计算关节角速度并绘图

dt = 1;   % 先设每个插值点之间的时间间隔为 1 s（形状正确，数值可后调）

qd_traj = diff(q_traj) / dt;            % 尺寸：(N_total-1) × 3
t_qd    = (1:size(qd_traj,1)) * dt;     % 对应时间轴

figure;
subplot(3,1,1);
plot(t_qd, qd_traj(:,1), 'LineWidth', 1.5);
grid on;
ylabel('\omega_1 [rad/s]');
title('关节 1 角速度');

subplot(3,1,2);
plot(t_qd, qd_traj(:,2), 'LineWidth', 1.5);
grid on;
ylabel('\omega_2 [rad/s]');
title('关节 2 角速度');

subplot(3,1,3);
plot(t_qd, qd_traj(:,3), 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('\omega_3 [rad/s]');
title('关节 3 角速度');

sgtitle('三关节角速度曲线');

%% 7. 动画演示整个运动过程

figure;
robot.plot(q_traj, ...
    'trail', {'r', 'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R 简化模型：4 点关节空间轨迹（解析 IK）');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 本地函数：Dobot 3R 的解析 IK（仅位置）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q_best = dobot3R_IK_analytic_pos(p, q_ref, robot)
% dobot3R_IK_analytic_pos  对当前 DH 模型的解析逆解（只用位置）
%
% 输入：
%   p     : 1x3 位置 [x y z]，单位 mm
%   q_ref : 1x3 参考关节角（上一时刻或者初值），用于选取最接近的分支
%   robot : SerialLink 机器人（用于取 a、qlim 等）
%
% 输出：
%   q_best: 1x3 关节角解 [q1 q2 q3]（弧度）
%
% 说明：
%   - 解析解是针对如下 DH（含 base & offset）推导：
%       alpha = [-pi/2, 0, 0];
%       d     = [8, 0, 0];
%       a     = [0, 135, 147];
%       L(2).offset = -pi/2; L(3).offset = pi/2;
%       robot.base = transl(0,0,-8);
%   - 对应的 FK 位置为：
%       x = (135*sin(q2) + 147*cos(q2+q3)) * cos(q1)
%       y = (135*sin(q2) + 147*cos(q2+q3)) * sin(q1)
%       z = -147*sin(q2+q3) + 135*cos(q2)
%     （这里的 q2,q3 是“代码里的 q”，已经包含 offset）

    % 取连杆长度
    L2 = robot.links(2).a;   % 135
    L3 = robot.links(3).a;   % 147

    x = p(1); y = p(2); z = p(3);

    % 平面半径 r 和距离 d
    r = hypot(x, y);
    d = hypot(r, z);

    % 可达性检查（简单版三角形不等式）
    if d > (L2 + L3) + 1e-6 || d < abs(L2 - L3) - 1e-6
        error('目标点 (%.1f, %.1f, %.1f) 超出工作空间，解析 IK 无解。', x, y, z);
    end

    % === 1) 解析求 q1 ===
    q1 = atan2(y, x);   % 旋转底座，使平面对准目标投影

    % === 2) 在 (r,z) 平面上做 2R 几何解析解 ===
    % 推导得到的关键方程：
    %   z*sin(s) - r*cos(s) = K
    % 其中 s = q2 + q3
    K   = (L2^2 - d^2 - L3^2) / (2*L3);
    R0  = d;                      % 实际就是 sqrt(r^2+z^2)
    arg = K / R0;
    arg = max(min(arg, 1), -1);   % 限幅防数值问题

    % delta 使得：
    %   z*sin(s) - r*cos(s) = R0 * sin(s + delta)
    delta = atan2(-r, z);         % sin(delta) = -r/d, cos(delta)=z/d

    % 两个分支：s1, s2
    s1 = -delta + asin(arg);          % 分支 1
    s2 = -delta + (pi - asin(arg));   % 分支 2

    s_list = [s1, s2];
    q_candidates = zeros(2,3);

    for k = 1:2
        s = s_list(k);

        % 由：
        %   L2*cos(q2) = z + L3*sin(s)
        %   L2*sin(q2) = r - L3*cos(s)
        cos_q2L2 = z + L3*sin(s);
        sin_q2L2 = r - L3*cos(s);

        q2 = atan2(sin_q2L2, cos_q2L2);
        q3 = s - q2;

        q_candidates(k,:) = [q1, q2, q3];
    end

    % === 3) 分支选择：优先满足关节限位，其次离 q_ref 最近 ===
    valid = true(2,1);
    for k = 1:2
        qk = q_candidates(k,:);

        % J2 限位
        if ~isempty(robot.links(2).qlim)
            if qk(2) < robot.links(2).qlim(1)-1e-6 || qk(2) > robot.links(2).qlim(2)+1e-6
                valid(k) = false;
            end
        end
        % J3 限位
        if ~isempty(robot.links(3).qlim)
            if qk(3) < robot.links(3).qlim(1)-1e-6 || qk(3) > robot.links(3).qlim(2)+1e-6
                valid(k) = false;
            end
        end
    end

    idx_valid = find(valid);
    if isempty(idx_valid)
        % 两个分支都超限：给个警告，仍然选一个“最接近 q_ref”的
        warning('解析 IK：两个分支都超出关节限位，仍将选择最接近参考姿态的解。');
        idx_valid = 1:2;
    end

    % 没有参考值时，直接取第一个 valid
    if any(isnan(q_ref)) || isempty(q_ref)
        q_best = q_candidates(idx_valid(1),:);
        return;
    end

    % 有参考值时，在 valid 里选一个“角度距离最小”的
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
%% 本地函数：关节角差（逐元素 wrap 到 [-pi,pi]）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function d = angdiff_vec(q1, q2)
% angdiff_vec  逐元素角度差，结果在 [-pi,pi]
    d = atan2(sin(q2 - q1), cos(q2 - q1));
end