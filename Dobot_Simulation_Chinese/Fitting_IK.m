clc; clear; close all;

%% ========= 1. 拟合得到的几何参数（用你自己的 p_opt 替换） =========
% p = [L1, L2, h, tx, tz,  s2,  s3,   o1,    o2,    o3];
% L1, L2: 两个臂长 (mm)
% h     : 基座到肩关节高度 (mm)
% tx,tz : TCP 相对法兰的等效偏移 (mm) —— 只在 FK 中用；IK 里暂时忽略
% s2,s3 : 将 Dobot Studio 读出的角度缩放为真实几何角度的比例
% o1,o2,o3: 角度偏移 (rad)，把 Studio 的零位映射到几何模型的零位

% ====== 这里先放一个合理的模板，你要用你拟合出来的 p_opt 覆盖它 ======
p_opt = [ ...
    135.0, ...   % L1
    147.0, ...   % L2
      0.0, ...   % h
     60.0, ...   % tx
    -60.0, ...   % tz
      1.0, ...   % s2
      1.0, ...   % s3
      0.0, ...   % o1
   -pi/2, ...   % o2
    pi/2  ...   % o3
];

%% ========= 2. 关节角度限制（Dobot Studio 面板给的范围） =========
% 关节 1: -135° ~ 135°
% 关节 2:   0°  ~  85°    (大臂)
% 关节 3: -10°  ~  90°    (小臂)

joint_limit_deg = [ ...
   -135,  135;  % q1
      0,   85;  % q2
    -10,   90   % q3
];

%% ========= 3. 想要到达的末端位置 P_des (世界坐标, mm) =========
% 这里先用你“验证集”的 3 个点，后续可以自己改成 6 个点、更多点

P_des = [ ...
    89, 150,  40;   % 点 1
    76, 120, -30;   % 点 2
   248, -30,  60;   % 点 3
];

%% ========= 4. 逆运动学求解（用拟合后的几何模型） =========
[q_ik, flag] = dobot_ik_equiv(P_des, p_opt, joint_limit_deg);
q_ik_deg = rad2deg(q_ik);

disp('=== IK 求解得到的关节角 (deg) ===');
disp(q_ik_deg);
disp('=== 每个点是否可达 flag (1=可达, 0=不可达) ===');
disp(flag.');

%% ========= 5. 用 FK 检查 IK 解是否回到 P_des 附近 =========
P_fk = dobot_fk_equiv(q_ik, p_opt);   % 用等效几何 FK 再算一遍

err = P_fk - P_des;
err_norm = sqrt(sum(err.^2, 2));

disp('=== IK + FK 回代后的末端坐标与误差 (mm) ===');
T = table(P_des(:,1),P_des(:,2),P_des(:,3), ...
          P_fk(:,1),P_fk(:,2),P_fk(:,3), ...
          err(:,1),err(:,2),err(:,3),err_norm, ...
    'VariableNames',{'X_des','Y_des','Z_des', ...
                     'X_fk','Y_fk','Z_fk', ...
                     'dX','dY','dZ','ErrNorm'});
disp(T);

fprintf('平均误差 = %.3f mm, 最大误差 = %.3f mm\n', ...
        mean(err_norm(~isnan(err_norm))), max(err_norm(~isnan(err_norm))));