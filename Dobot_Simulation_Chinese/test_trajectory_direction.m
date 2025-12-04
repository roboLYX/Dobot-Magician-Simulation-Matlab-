%% 轨迹倒置诊断工具 - test_trajectory_direction.m
% 
% 用途：快速诊断末端轨迹是否倒置
% 使用：直接运行此文件
%
% 输出：
%   - 诊断结果：轨迹是否反向
%   - 建议：如何修正
%   - 可视化：轨迹对比图

clc; clear; close all;

fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║   Dobot 3R 轨迹倒置诊断工具                    ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

%% 第 1 部分：建立机器人模型

fprintf('第 1 步：建立机器人模型...\n');

alpha = [-pi/2, 0, 0];
d     = [8,      0, 0];
a     = [0,    135, 147];

L(1) = Link([0, d(1), a(1), alpha(1)], 'standard');
L(2) = Link([0, d(2), a(2), alpha(2)], 'standard');
L(3) = Link([0, d(3), a(3), alpha(3)], 'standard');

% ========== 关键参数（调试时可以改这里）==========
fprintf('\n当前设置：\n');
fprintf('  L(2).offset = %f (= %.1f°)\n', -pi/2, -90);
fprintf('  L(3).offset = %f (= %.1f°)\n', pi/2, 90);

L(2).offset = -pi/2;
L(3).offset =  pi/2;
L(2).qlim = [  0,  85] * pi/180;
L(3).qlim = [-10, 90] * pi/180;

robot = SerialLink(L, 'name', 'Dobot_3R');
robot.base = transl(0, 0, -8);

fprintf('✓ 机器人模型建立完成\n');

%% 第 2 部分：定义四个目标点

fprintf('\n第 2 步：定义目标点...\n');

P1 = [150,   50,  -50];
P2 = [150,   50,   50];
P3 = [-150,  150,   50];
P4 = [-150,  150,  -50];

P_list = [P1; P2; P3; P4];

fprintf('四个目标点：\n');
for i = 1:4
    fprintf('  P%d = [%6.0f, %6.0f, %6.0f] mm\n', i, P_list(i,1), P_list(i,2), P_list(i,3));
end

fprintf('✓ 目标点定义完成\n');

%% 第 3 部分：IK 求解

fprintf('\n第 3 步：IK 求解...\n');

q_wp = zeros(4, 3);
q_guess = [0, -60*pi/180, 60*pi/180];

try
    for i = 1:4
        if i == 1
            q_wp(i,:) = robot.ikcon(transl(P_list(i,:)), q_guess);
        else
            q_wp(i,:) = robot.ikcon(transl(P_list(i,:)), q_wp(i-1,:));
        end
    end
    fprintf('✓ IK 求解完成\n');
catch
    fprintf('❌ IK 求解失败，可能初值不合适\n');
    return;
end

%% 第 4 部分：关键点 FK 验证

fprintf('\n第 4 步：关键点 FK 验证...\n');

T_wp = robot.fkine(q_wp);
p_wp = transl(T_wp);

fprintf('4 个关键点的 FK 结果：\n');
for i = 1:4
    err = norm(p_wp(i,:) - P_list(i,:));
    fprintf('  P%d 误差 = %.4f mm\n', i, err);
end

%% 第 5 部分：jtraj 插值

fprintf('\n第 5 步：jtraj 插值...\n');

N = 50;
q_traj = [];
for i = 1:3
    [q_seg, ~, ~] = jtraj(q_wp(i,:), q_wp(i+1,:), N);
    if i == 1
        q_traj = q_seg;
    else
        q_traj = [q_traj; q_seg(2:end,:)];
    end
end

fprintf('插值完成：%d 个采样点\n', size(q_traj, 1));

%% 第 6 部分：FK 得到末端轨迹

fprintf('\n第 6 步：FK 计算末端轨迹...\n');

T_fk = robot.fkine(q_traj);
p_fk = transl(T_fk);

fprintf('✓ 末端轨迹计算完成\n');

%% ============= 第 7 部分：诊断 =============

fprintf('\n');
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║              诊断结果                        ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

% 7.1 方向向量检查
fprintf('【方向性诊断】\n');

dir_ideal = P_list(end,:) - P_list(1,:);    % P1 → P4
dir_actual = p_fk(end,:) - p_fk(1,:);      % FK 起终点

fprintf('期望方向（P1→P4）：[%7.2f, %7.2f, %7.2f] mm\n', dir_ideal);
fprintf('实际方向（始→末）：[%7.2f, %7.2f, %7.2f] mm\n', dir_actual);

dot_prod = dot(dir_ideal, dir_actual);
norm_ideal = norm(dir_ideal);
norm_actual = norm(dir_actual);
norm_prod = dot_prod / (norm_ideal * norm_actual);

fprintf('\n点积 = %.6f\n', dot_prod);
fprintf('归一化点积 = %.6f (余弦相似度)\n\n', norm_prod);

% 判断
if norm_prod > 0.5
    fprintf('✅ 结论：轨迹方向正确\n');
    diagnosis_result = 'CORRECT';
elseif norm_prod < -0.5
    fprintf('❌ 结论：轨迹方向反向\n');
    diagnosis_result = 'REVERSED';
else
    fprintf('⚠️  结论：轨迹方向不确定（可能是螺旋路径）\n');
    diagnosis_result = 'UNCLEAR';
end

% 7.2 轨迹长度
fprintf('\n【轨迹长度】\n');
segment_lengths = sqrt(sum(diff(p_fk).^2, 2));
total_length = sum(segment_lengths);
fprintf('总轨迹长度 = %.2f mm\n', total_length);

% 7.3 轨迹偏差
fprintf('\n【轨迹质量】\n');

% 与直线的偏差
line_direction = dir_ideal / norm(dir_ideal);  % 单位方向向量
line_start = p_fk(1,:);

deviations = [];
for i = 1:size(p_fk, 1)
    point = p_fk(i,:);
    % 点到直线的距离
    v = point - line_start;
    proj_length = dot(v, line_direction);
    proj_point = line_start + proj_length * line_direction;
    deviation = norm(point - proj_point);
    deviations = [deviations; deviation];
end

max_deviation = max(deviations);
mean_deviation = mean(deviations);

fprintf('轨迹与理想直线的最大偏差 = %.4f mm\n', max_deviation);
fprintf('轨迹与理想直线的平均偏差 = %.4f mm\n', mean_deviation);

%% 第 8 部分：建议

fprintf('\n');
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║              修正建议                        ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

if strcmp(diagnosis_result, 'REVERSED')
    fprintf('检测到轨迹反向！建议按以下顺序修改：\n\n');
    fprintf('第 1 个试：修改 L(2).offset\n');
    fprintf('  当前值：%.4f (= %.1f°)\n', L(2).offset, L(2).offset*180/pi);
    fprintf('  试试改成：%.4f (= %.1f°)\n', pi/2, 90);
    fprintf('  代码：L(2).offset = pi/2;\n\n');
    
    fprintf('第 2 个试：修改 L(3).offset\n');
    fprintf('  当前值：%.4f (= %.1f°)\n', L(3).offset, L(3).offset*180/pi);
    fprintf('  试试改成：%.4f (= %.1f°)\n', -pi/2, -90);
    fprintf('  代码：L(3).offset = -pi/2;\n\n');
    
    fprintf('第 3 个试：修改 robot.base 的 z 方向\n');
    fprintf('  当前值：z = -8\n');
    fprintf('  试试改成：z = 8\n');
    fprintf('  代码：robot.base = transl(0, 0, 8);\n\n');
    
    fprintf('第 4 个试：加上旋转\n');
    fprintf('  试试：robot.base = transl(0, 0, -8) * rotz(pi);\n');
    fprintf('  或：robot.base = transl(0, 0, -8) * rotx(pi);\n\n');
    
    fprintf('修改后请重新运行此脚本验证。\n');
    
elseif strcmp(diagnosis_result, 'CORRECT')
    fprintf('✅ 无需修改！轨迹方向正确。\n\n');
    if max_deviation < 1
        fprintf('✅ 轨迹质量优秀！偏差 < 1 mm\n');
    elseif max_deviation < 10
        fprintf('⚠️  轨迹质量一般。偏差 %.2f mm\n', max_deviation);
        fprintf('   建议：增加 jtraj 的插值点数（从 %d 改为 100）\n', N);
        fprintf('   或切换到 ctraj（笛卡尔空间插值）\n');
    else
        fprintf('❌ 轨迹质量差。偏差 %.2f mm\n', max_deviation);
        fprintf('   强烈建议：使用 ctraj 而不是 jtraj\n');
    end
else
    fprintf('轨迹方向不确定，可能是以下情况：\n');
    fprintf('1. 路径本质上是螺旋形（正常）\n');
    fprintf('2. 轨迹配置复杂\n');
    fprintf('请查看可视化结果确认。\n');
end

%% 第 9 部分：可视化

fprintf('\n');
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║              可视化                         ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

% 9.1 3D 轨迹图
figure('Position', [100, 100, 1200, 400]);

subplot(1,3,1);
plot3(p_fk(:,1), p_fk(:,2), p_fk(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(P_list(:,1), P_list(:,2), P_list(:,3), 'ro', 'MarkerSize', 10);
plot3(P_list(1,1), P_list(1,2), P_list(1,3), 'g*', 'MarkerSize', 15);  % 起点
plot3(P_list(end,1), P_list(end,2), P_list(end,3), 'r*', 'MarkerSize', 15);  % 终点
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('末端轨迹');
legend('FK轨迹', '目标点', '起点', '终点');

% 9.2 误差曲线
subplot(1,3,2);
plot(deviations, 'LineWidth', 1.5);
grid on;
xlabel('采样点');
ylabel('到理想直线的距离 (mm)');
title('轨迹偏差曲线');
fprintf('✓ 生成了轨迹可视化图\n');

% 9.3 关键点对比
subplot(1,3,3);
bar(1:4, err_abs_norm, 'FaceColor', [0.2 0.6 0.8]); hold on;
plot(1:4, ones(1,4)*0.1, 'r--', 'LineWidth', 2);  % 0.1mm 阈值
grid on;
xlabel('点号'); ylabel('误差 (mm)');
title('关键点 FK 误差');
set(gca, 'XTick', 1:4);

% 计算关键点误差（如果还没有）
if ~exist('err_abs_norm', 'var')
    err_abs = p_wp - P_list;
    err_abs_norm = sqrt(sum(err_abs.^2, 2));
end

%% 最终总结

fprintf('\n');
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║              诊断完成                        ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

fprintf('诊断结果摘要：\n');
fprintf('  轨迹方向：%s\n', diagnosis_result);
fprintf('  归一化点积：%.4f\n', norm_prod);
fprintf('  轨迹长度：%.2f mm\n', total_length);
fprintf('  最大偏差：%.4f mm\n', max_deviation);
fprintf('  平均偏差：%.4f mm\n\n', mean_deviation);

if strcmp(diagnosis_result, 'REVERSED')
    fprintf('🔧 建议立即修改配置文件！\n');
else
    fprintf('✓ 无需修改！\n');
end

fprintf('\n完成！\n');
