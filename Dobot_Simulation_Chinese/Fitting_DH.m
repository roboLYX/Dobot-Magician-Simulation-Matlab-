clc; clear; close all;

%% ========= 1. 训练集数据：实机 TCP 坐标 & 关节角（度） =========
% P_real: Dobot Studio 显示的 TCP 坐标 (mm)
P_real = [
    260,   0,    0;
    260,   0,  -66;
    260,   0,    0;
      0, 260,    0;
      0, 260,  -66;
      0, 260,    0;
    146,   0,    0;
    146,   0,  -50;
      0, 146,   50;
     20, 150,  -20;
      0, 240,    0;
    -65, 159,   36;
];

% q_meas_deg: 对应的关节角 (deg)，来自 Dobot Studio
q_meas_deg = [
      0, 66.17, 21.78;
      0, 85.47, 31.44;
      0, 66.17, 21.78;
     90, 66.17, 21.78;
     90, 85.37, 31.44;
     90, 66.17, 21.78;
      0, 27.06, 54.87;
      0, 48.27, 72.07;
     90, 10.46, 34.26;
     82.41,36.43, 61.04;
     90, 56.79, 30.20;
    112.24,23.52, 36.67;
];

assert(size(P_real,1) == size(q_meas_deg,1), '训练集数据行数不一致！');

q_meas = deg2rad(q_meas_deg);   % 统一转成弧度

%% ========= 2. 定义等效几何模型 FK 函数 =========
% p = [L1, L2, h, tx, tz,  s2, s3,  o1, o2, o3];
fk_fun = @(q, p) dobot_fk_equiv(q, p);   % 子函数在文件末尾

%% ========= 3. 构造误差目标函数（仅用训练集） =========
% 初始猜测：用官方几何 + 大概的 TCP 偏移和零位修正
p0 = [135, 147,  0,  60, -60,  1, 1,   0, -pi/2, pi/2];

% cost_fun 返回一个长向量：所有点的 (X,Y,Z) 残差摊平
cost_fun = @(p) reshape( fk_fun(q_meas, p) - P_real, [], 1 );

%% ========= 4. 用 lsqnonlin / fminsearch 做标定 =========
use_lsq = license('test','Optimization_Toolbox');  % 有优化工具箱就用 lsqnonlin

if use_lsq
    options = optimoptions('lsqnonlin',...
        'Display','iter',...
        'MaxFunEvals',1e4,...
        'MaxIter',1e3);
    p_opt = lsqnonlin(cost_fun, p0, [], [], options);
else
    % 没有优化工具箱就用 fminsearch（效果差一点但能跑）
    f_cost_scalar = @(p) norm(cost_fun(p));
    options = optimset('Display','iter',...
        'MaxFunEvals',1e5,...
        'MaxIter',1e4);
    p_opt = fminsearch(f_cost_scalar, p0, options);
end

disp('=== 标定得到的参数 p_opt ===');
fprintf('L1 = %.3f mm\n', p_opt(1));
fprintf('L2 = %.3f mm\n', p_opt(2));
fprintf('h  = %.3f mm\n', p_opt(3));
fprintf('tx = %.3f mm, tz = %.3f mm\n', p_opt(4), p_opt(5));
fprintf('s2 = %.4f, s3 = %.4f\n', p_opt(6), p_opt(7));
fprintf('o1 = %.3f deg, o2 = %.3f deg, o3 = %.3f deg\n', ...
        rad2deg(p_opt(8)), rad2deg(p_opt(9)), rad2deg(p_opt(10)));

%% ========= 5. 查看训练集拟合误差 =========
P_fit = fk_fun(q_meas, p_opt);
err   = P_fit - P_real;
err_norm = sqrt(sum(err.^2,2));

disp('=== 训练集：拟合结果与误差（mm） ===');
T_train = table(P_real(:,1), P_real(:,2), P_real(:,3), ...
                P_fit(:,1), P_fit(:,2), P_fit(:,3), ...
                err(:,1),   err(:,2),   err(:,3),   err_norm, ...
 'VariableNames',{'X_real','Y_real','Z_real', ...
                  'X_fit','Y_fit','Z_fit', ...
                  'dX','dY','dZ','ErrNorm'});
disp(T_train);

fprintf('训练集平均误差 = %.3f mm, 最大误差 = %.3f mm\n', ...
        mean(err_norm), max(err_norm));

%% ========= 6. 验证集：只用 q + p_opt 做 FK 预测 =========
% 注意：这里 FK 只依赖 q_val 和 p_opt，不用验证集坐标。
%       验证集真实坐标仅在后面用来做误差对比。

% 验证集真实 TCP 坐标（mm）—— Dobot Studio 读数
P_val_real = [
    89,   150,   40;    % V1
    76,   120,  -30;    % V2
    248,  -30,   60;    % V3
];

% 对应的关节角（deg）
q_val_deg = [
    59.32,  23.43, 34.79;   % V1
    57.65,  38.76, 66.96;   % V2
    -6.90,  51.02,  9.76;   % V3
];

q_val = deg2rad(q_val_deg);

% —— 核心：只用 q_val 和 p_opt 做正向运动学预测 ——
P_val_pred = fk_fun(q_val, p_opt);

disp('=== 验证集：预测的 TCP 坐标（mm）（只用 q_val + p_opt） ===');
disp(P_val_pred);

%% ========= 7. （可选）验证集误差统计 =========
err_val      = P_val_pred - P_val_real;
err_val_norm = sqrt(sum(err_val.^2, 2));

T_val = table( ...
    P_val_real(:,1), P_val_real(:,2), P_val_real(:,3), ...
    P_val_pred(:,1), P_val_pred(:,2), P_val_pred(:,3), ...
    err_val(:,1),    err_val(:,2),    err_val(:,3),    err_val_norm, ...
    'VariableNames', {'X_real','Y_real','Z_real', ...
                      'X_pred','Y_pred','Z_pred', ...
                      'dX','dY','dZ','ErrNorm'});

disp('=== 验证集：真实 vs 预测 & 误差（mm） ===');
disp(T_val);

fprintf('验证集平均误差 = %.3f mm, 最大误差 = %.3f mm\n', ...
        mean(err_val_norm), max(err_val_norm));

%% ========= 8. 等效 FK 子函数 =========
function P = dobot_fk_equiv(q_meas, p)
    % q_meas: N x 3 (rad) [q1,q2,q3] 来自 Dobot Studio（界面角）
    % p: [L1, L2, h, tx, tz,  s2, s3,  o1, o2, o3]
    %
    % 模型说明：
    %  - 在 XZ 平面里用一个 2R 机构表示大臂 + 小臂
    %  - L1, L2 为等效连杆长度
    %  - h 为肩关节（J2）相对世界坐标原点的高度
    %  - tx, tz 为 TCP 相对 J3 法兰的偏移（在末端方向 & 竖直方向）
    %  - s2, s3 为比例因子，允许 q2, q3 和“数学关节角”之间有缩放
    %  - o1, o2, o3 为零位偏置（rad）

    L1 = p(1);  L2 = p(2);  h  = p(3);
    tx = p(4);  tz = p(5);
    s2 = p(6);  s3 = p(7);
    o1 = p(8);  o2 = p(9);  o3 = p(10);

    q1m = q_meas(:,1);
    q2m = q_meas(:,2);
    q3m = q_meas(:,3);

    % 把 Dobot Studio 的角度映射到“几何关节角”
    theta1 = q1m + o1;
    theta2 = s2.*q2m + o2;
    theta3 = s3.*q3m + o3;

    % —— 平面 2R 位置（J3 法兰）——
    x_p = L1.*cos(theta2) + L2.*cos(theta2 + theta3);
    z_p = L1.*sin(theta2) + L2.*sin(theta2 + theta3);

    % —— 加 TCP 偏移（沿末端方向 tx，竖直 tz）——
    x_p_tcp = x_p + tx.*cos(theta2 + theta3);
    z_p_tcp = z_p + tx.*sin(theta2 + theta3) + tz;

    % —— 绕 Z 轴用 q1 旋转成 3D 世界坐标 —— 
    x = x_p_tcp .* cos(theta1);
    y = x_p_tcp .* sin(theta1);
    z = h + z_p_tcp;

    P = [x, y, z];
end