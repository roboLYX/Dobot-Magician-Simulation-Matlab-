%% Dobot 3R：jtraj 关节插值 vs ctraj 笛卡尔插值 对比
% 1) 使用 ikcon 得到 4 个离散点的关节角
% 2) 方式 A：在关节空间用 jtraj 插值 -> TCP 轨迹 A
% 3) 方式 B：在笛卡尔空间用 ctraj 插值 -> 对每一帧做 ikcon -> TCP 轨迹 B
% 4) 对比两种方式的末端轨迹、关节轨迹（图表 + 动画）

clc; clear; close all;

%% 1. 按给定 DH 建 3R 机械臂（沿用你原来的设置）

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% 零位修正 & 关节范围
L(2).offset = -pi/2;          % 大臂零位修正
L(3).offset = pi/2;           % 小臂零位修正
L(2).qlim = [  0,  85] * pi/180;   % J2
L(3).qlim = [-10, 90] * pi/180;   % J3

robot = SerialLink(L, 'name', 'Dobot_3R');

% 底座坐标：向下平移 8 mm，使肩关节为世界原点
robot.base = transl(0, 0, -8);

%% 2. 定义 4 个末端目标点（只指定位置，姿态默认 R=I）

P1 = [ 150,   50,  -50];
P2 = [ 150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

T = cell(4,1);
for i = 1:4
    T{i} = transl(P_list(i,:));   % 只指定位置，姿态为单位矩阵
end

%% 3. 使用 ikcon 对 4 个点求离散关节解 q_wp

q_wp = zeros(4,3);
q_guess = [0, -60, 60] * pi/180;   % 初始猜测：上肘姿态

% 第一个点用预设初值
q_wp(1,:) = robot.ikcon(T{1}, q_guess);

% 后续点用前一点的解作为初值，保证解的连续性
for i = 2:4
    q_wp(i,:) = robot.ikcon(T{i}, q_wp(i-1,:));
end

disp('=== 4 个点的关节角（弧度） ===');
disp(q_wp);
disp('=== 4 个点的关节角（角度） ===');
disp(rad2deg(q_wp));

%% 3.5 用 FK 检查 4 个离散点的 FK 误差（ikcon 带来的误差）

T_wp = robot.fkine(q_wp);   % 4 个姿态的 FK（含 base）
p_wp = transl(T_wp);        % 4×3，TCP 位置

disp('=== 4 个点用 FK 计算得到的 TCP 位置（mm） ===');
disp(p_wp);

compare_tbl = table(P_list(:,1), P_list(:,2), P_list(:,3), ...
                    p_wp(:,1),  p_wp(:,2),  p_wp(:,3), ...
    'VariableNames', {'Px_des','Py_des','Pz_des','Px_fk','Py_fk','Pz_fk'});
disp(compare_tbl);

% 误差分析
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

disp('=== 各点末端位置的绝对误差 / 相对误差（由 ikcon 导致） ===');
disp(err_tbl);

[maxErrNorm, idxMax] = max(err_abs_norm);
fprintf('最大位置欧氏误差 = %.3f mm，对应第 %d 个点。\n', maxErrNorm, idxMax);
fprintf('对应的相对误差 = %.4f\n', rel_err(idxMax));

%% 4. 方式 A：关节空间插值 jtraj（沿用你原来的方式）

N = 20;   % 每段插值点数

q_traj_j = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp(i,:), q_wp(i+1,:), N);
    if i == 1
        q_traj_j = q_seg;
    else
        q_traj_j = [q_traj_j; q_seg(2:end,:)];  % 去掉重复
    end
end

% FK 得到 TCP 轨迹（方式 A）
T_fk_j = robot.fkine(q_traj_j);
p_fk_j = transl(T_fk_j);   % M_A × 3

%% 5. 方式 B：笛卡尔空间插值 ctraj + ikcon

% 5.1 先在笛卡尔空间生成 T 轨迹（理想 TCP 轨迹，精准通过四点）
T_traj_c = [];   % 将要存成 4x4xM 的三维数组

for i = 1:3
    % 从 T{i} 到 T{i+1} 做 N 步插值
    T_seg = ctraj(T{i}, T{i+1}, N);   % 尺寸：4x4xN

    if i == 1
        % 第一段：直接赋值
        T_traj_c = T_seg;             % 4x4xN
    else
        % 之后的段：沿第 3 维拼接，并去掉段首重复的那一帧
        % T_seg(:,:,2:end) 才是 "第 2 到第 N 帧"
        T_traj_c = cat(3, T_traj_c, T_seg(:,:,2:end));  % 4x4x(M+N-1)
    end
end

% 5.2 从 T_traj_c 的每一帧再用 ikcon 求出关节角 → q_traj_c
M_c = size(T_traj_c, 3);     % 共有多少帧
q_traj_c = zeros(M_c, 3);

% 初始猜测：用第一个离散点 q_wp(1,:)
q_prev = q_wp(1,:);

for k = 1:M_c
    % 第 k 帧的齐次矩阵是 T_traj_c(:,:,k)
    Tk = T_traj_c(:,:,k);

    % 使用上一帧的解作为初值，保证分支连续
    q_traj_c(k,:) = robot.ikcon(Tk, q_prev);
    q_prev = q_traj_c(k,:);
end

% 5.3 用 FK 看一下 "ctraj + ikcon" 实际得到的 TCP 轨迹
T_fk_c = robot.fkine(q_traj_c);
p_fk_c = transl(T_fk_c);   % M_c × 3

% 同时取出 "理想笛卡尔插值"的 TCP 轨迹（纯 transl，不经过 IK）
p_traj_c_ideal = zeros(M_c,3);
for k = 1:M_c
    p_traj_c_ideal(k,:) = transl(T_traj_c(:,:,k));
end

%% 6. 轨迹对比：jtraj vs ctraj（位置）

figure;
plot3(p_fk_j(:,1), p_fk_j(:,2), p_fk_j(:,3), '-', 'LineWidth', 1.5); hold on;
plot3(p_fk_c(:,1), p_fk_c(:,2), p_fk_c(:,3), '--', 'LineWidth', 1.5);
plot3(P_list(:,1), P_list(:,2), P_list(:,3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('TCP 轨迹对比：jtraj (关节空间) vs ctraj+ikcon (笛卡尔空间)');
legend('jtraj -> FK','ctraj+ikcon -> FK','4 个目标点','Location','Best');

%% 7. 对比关节角轨迹：jtraj vs ctraj+ikcon

% 为了便于对齐时间轴，简单统一用样本 index 作"时间"
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
        title('关节角轨迹对比：jtraj vs ctraj+ikcon');
    end
end
xlabel('样本 index');
legend('jtraj','ctraj+ikcon','Location','Best');

%% 8. 额外：比较 "理想笛卡尔插值" vs "ctraj+ikcon 实际 TCP" 的误差

err_c = p_fk_c - p_traj_c_ideal;          % 每点的 (Δx,Δy,Δz)
err_c_norm = sqrt(sum(err_c.^2, 2));      % 欧氏距离

figure;
plot(err_c_norm, 'LineWidth', 1.5);
grid on;
xlabel('样本 index');
ylabel('误差范数 ||Δp|| [mm]');
title('ctraj 理想 TCP vs 通过 ikcon 实际 TCP 的位置误差');

fprintf('ctraj+ikcon 最大位置误差范数 = %.3f mm\n', max(err_c_norm));

%% 9. 动画演示：两种插值方式的机器人运动

% 9.1 jtraj 动画（关节空间插值）
figure;
robot.plot(q_traj_j, ...
    'trail', {'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R：关节空间插值 jtraj 动画');
grid on;

% 等用户关闭图窗再播放下一段动画比较清晰
disp('关闭 jtraj 动画窗口后，将开始 ctraj+ikcon 动画...');
pause;   % 等待用户按键或关闭窗口

% 9.2 ctraj+ikcon 动画（笛卡尔路径插值，再用 ikcon 跟踪）
figure;
robot.plot(q_traj_c, ...
    'trail', {'LineWidth', 1.5}, ...
    'nowrist', 'noname');
title('Dobot 3R：笛卡尔插值 ctraj + ikcon 动画');
grid on;