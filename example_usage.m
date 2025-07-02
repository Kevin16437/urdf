%% ARM2.0 机械臂工作空间分析 - 使用示例
% 演示如何使用各种分析工具

clear all; close all; clc;

fprintf('=== ARM2.0 机械臂分析示例 ===\n\n');

%% 示例1: 快速工作空间分析
fprintf('示例1: 快速工作空间分析\n');
fprintf('----------------------------------------\n');

% 设置机械臂参数
L1 = 0.1845; L2 = 0.1785; L3 = 0.39; 
L4 = 0.194; L5 = 0.2665; L6 = 0.084;

% DH参数表
DH_params = [
    0,      -pi/2,  L1,     0;
    0,      pi/2,   0,      pi/2;
    L3,     0,      0,      0;
    0,      -pi/2,  L4,     0;
    0,      pi/2,   0,      0;
    0,      0,      L6,     0
];

% 关节限制
joint_limits = [
    -pi,     pi;
    -1.57,   1.57;
    -1.57,   1.57;
    -pi,     pi;
    -1.57,   1.57;
    -pi,     pi
];

% 快速采样计算工作空间
resolution = 0.5;  % 粗糙采样
max_points = 5000;

fprintf('正在计算工作空间 (粗糙采样)...\n');
reachable_points = [];
point_count = 0;

tic;
for j1 = joint_limits(1,1):resolution:joint_limits(1,2)
    for j2 = joint_limits(2,1):resolution:joint_limits(2,2)
        for j3 = joint_limits(3,1):resolution:joint_limits(3,2)
            if point_count >= max_points, break; end
            
            % 固定后三个关节为0（简化计算）
            theta = [j1, j2, j3, 0, 0, 0];
            
            % 计算末端位置
            T = forward_kinematics(theta, DH_params);
            pos = T(1:3, 4)';
            
            reachable_points = [reachable_points; pos];
            point_count = point_count + 1;
        end
        if point_count >= max_points, break; end
    end
    if point_count >= max_points, break; end
end

fprintf('计算完成，用时: %.2f 秒\n', toc);
fprintf('计算点数: %d\n', size(reachable_points, 1));

% 简单可视化
figure('Name', '示例1: 快速工作空间分析');
scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), 'filled');
title('ARM2.0 工作空间 (前3关节)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

%% 示例2: 单点运动学分析
fprintf('\n示例2: 单点运动学分析\n');
fprintf('----------------------------------------\n');

% 测试关节角度
test_angles = [pi/4, pi/6, -pi/6, pi/3, -pi/4, pi/2];
fprintf('测试关节角度 (度): ');
fprintf('%.1f ', rad2deg(test_angles));
fprintf('\n');

% 正向运动学
T_end = forward_kinematics(test_angles, DH_params);
end_position = T_end(1:3, 4);
end_orientation = rotm2eul(T_end(1:3, 1:3), 'ZYX');

fprintf('末端执行器位置: [%.3f, %.3f, %.3f] m\n', end_position);
fprintf('末端执行器姿态 (ZYX欧拉角): [%.2f, %.2f, %.2f] 度\n', rad2deg(end_orientation));

% 雅可比矩阵分析
J = compute_jacobian(test_angles, DH_params);
fprintf('雅可比矩阵条件数: %.2f\n', cond(J));

% 可操作性
[manipulability, condition_num] = analyze_manipulability(test_angles, DH_params);
fprintf('可操作性指标: %.6f\n', manipulability);

%% 示例3: 轨迹规划和动画
fprintf('\n示例3: 轨迹规划和动画\n');
fprintf('----------------------------------------\n');

% 起始和终止关节角度
theta_start = [0, 0, 0, 0, 0, 0];
theta_end = [pi/2, pi/4, -pi/4, pi/6, pi/3, -pi/2];

fprintf('起始角度 (度): ');
fprintf('%.1f ', rad2deg(theta_start));
fprintf('\n');
fprintf('终止角度 (度): ');
fprintf('%.1f ', rad2deg(theta_end));
fprintf('\n');

% 生成轨迹
duration = 3.0;  % 3秒
dt = 0.1;        % 0.1秒步长
[trajectory, time_stamps] = plan_joint_trajectory(theta_start, theta_end, duration, dt);

fprintf('轨迹点数: %d\n', size(trajectory, 1));

% 计算轨迹上的末端位置
end_trajectory = zeros(size(trajectory, 1), 3);
for i = 1:size(trajectory, 1)
    T = forward_kinematics(trajectory(i, :), DH_params);
    end_trajectory(i, :) = T(1:3, 4)';
end

% 可视化轨迹
figure('Name', '示例3: 轨迹规划');
subplot(2,2,1);
plot(time_stamps, rad2deg(trajectory));
title('关节角度轨迹');
xlabel('时间 (s)'); ylabel('角度 (度)');
legend({'j1','j2','j3','j4','j5','j6'}, 'Location', 'best');
grid on;

subplot(2,2,2);
plot3(end_trajectory(:,1), end_trajectory(:,2), end_trajectory(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(end_trajectory(1,1), end_trajectory(1,2), end_trajectory(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(end_trajectory(end,1), end_trajectory(end,2), end_trajectory(end,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('末端执行器轨迹');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('轨迹', '起点', '终点');
grid on; axis equal;

subplot(2,2,3);
plot(time_stamps, end_trajectory);
title('末端位置vs时间');
xlabel('时间 (s)'); ylabel('位置 (m)');
legend({'X','Y','Z'}, 'Location', 'best');
grid on;

subplot(2,2,4);
distances = sqrt(sum(end_trajectory.^2, 2));
plot(time_stamps, distances, 'k-', 'LineWidth', 2);
title('到原点距离vs时间');
xlabel('时间 (s)'); ylabel('距离 (m)');
grid on;

%% 示例4: 工作空间边界分析
fprintf('\n示例4: 工作空间边界分析\n');
fprintf('----------------------------------------\n');

% 使用示例1的数据进行边界分析
if size(reachable_points, 1) > 100
    fprintf('分析工作空间边界...\n');
    
    % 计算凸包
    try
        [K, V] = convhull(reachable_points);
        fprintf('凸包体积: %.6f m³\n', V);
        fprintf('凸包面数: %d\n', size(K, 1));
        
        % 可视化凸包
        figure('Name', '示例4: 工作空间边界');
        trisurf(K, reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), ...
            'FaceAlpha', 0.3, 'EdgeColor', 'blue');
        hold on;
        scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), ...
            10, 'red', 'filled');
        title('工作空间凸包边界');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        grid on; axis equal;
        
    catch
        fprintf('凸包计算失败，点数可能不足\n');
    end
    
    % 基本统计
    x_range = [min(reachable_points(:,1)), max(reachable_points(:,1))];
    y_range = [min(reachable_points(:,2)), max(reachable_points(:,2))];
    z_range = [min(reachable_points(:,3)), max(reachable_points(:,3))];
    
    fprintf('工作空间范围:\n');
    fprintf('  X: [%.3f, %.3f] m (跨度: %.3f m)\n', x_range(1), x_range(2), diff(x_range));
    fprintf('  Y: [%.3f, %.3f] m (跨度: %.3f m)\n', y_range(1), y_range(2), diff(y_range));
    fprintf('  Z: [%.3f, %.3f] m (跨度: %.3f m)\n', z_range(1), z_range(2), diff(z_range));
    
    distances = sqrt(sum(reachable_points.^2, 2));
    fprintf('到达距离: 最大=%.3f m, 最小=%.3f m, 平均=%.3f m\n', ...
        max(distances), min(distances), mean(distances));
end

%% 示例5: 逆运动学求解
fprintf('\n示例5: 逆运动学求解\n');
fprintf('----------------------------------------\n');

% 目标位置和姿态
target_position = [0.4, 0.2, 0.3];  % 目标位置
target_orientation = [0, 0, 0];      % 目标姿态 (ZYX欧拉角)

fprintf('目标位置: [%.3f, %.3f, %.3f] m\n', target_position);
fprintf('目标姿态: [%.1f, %.1f, %.1f] 度\n', rad2deg(target_orientation));

% 构造目标位姿矩阵
target_rot = eul2rotm(target_orientation, 'ZYX');
target_pose = eye(4);
target_pose(1:3, 1:3) = target_rot;
target_pose(1:3, 4) = target_position;

% 初始猜测
theta_init = [0, 0, 0, 0, 0, 0];

% 求解逆运动学
options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-6, 'MaxIterations', 1000);
[theta_solution, success] = inverse_kinematics_numerical(target_pose, DH_params, theta_init, options);

if success
    fprintf('逆运动学求解成功!\n');
    fprintf('解关节角度 (度): ');
    fprintf('%.2f ', rad2deg(theta_solution));
    fprintf('\n');
    
    % 验证解的正确性
    T_verify = forward_kinematics(theta_solution, DH_params);
    actual_position = T_verify(1:3, 4);
    position_error = norm(actual_position - target_position);
    
    fprintf('位置误差: %.6f m\n', position_error);
    
    if position_error < 0.001
        fprintf('解验证通过!\n');
    else
        fprintf('解验证失败，误差较大\n');
    end
else
    fprintf('逆运动学求解失败\n');
end

%% 示例6: 比较不同配置的可操作性
fprintf('\n示例6: 可操作性比较\n');
fprintf('----------------------------------------\n');

% 定义几个测试配置
test_configs = [
    0,     0,     0,     0,     0,     0;      % 零位
    pi/4,  pi/6,  -pi/6, 0,     0,     0;      % 配置1
    pi/2,  pi/4,  -pi/4, pi/6,  pi/3,  0;      % 配置2
    -pi/4, -pi/6, pi/3,  -pi/4, pi/6,  pi/2;   % 配置3
    pi/6,  pi/3,  -pi/2, pi/4,  -pi/3, -pi/4   % 配置4
];

config_names = {'零位', '配置1', '配置2', '配置3', '配置4'};

fprintf('配置名称\t可操作性\t条件数\t\t末端位置\n');
fprintf('--------\t--------\t------\t\t--------\n');

manipulability_values = zeros(size(test_configs, 1), 1);
condition_numbers = zeros(size(test_configs, 1), 1);
end_positions = zeros(size(test_configs, 1), 3);

for i = 1:size(test_configs, 1)
    theta = test_configs(i, :);
    
    % 计算可操作性
    [manip, cond_num] = analyze_manipulability(theta, DH_params);
    manipulability_values(i) = manip;
    condition_numbers(i) = cond_num;
    
    % 计算末端位置
    T = forward_kinematics(theta, DH_params);
    end_positions(i, :) = T(1:3, 4)';
    
    fprintf('%s\t\t%.6f\t%.2f\t\t[%.3f, %.3f, %.3f]\n', ...
        config_names{i}, manip, cond_num, end_positions(i, :));
end

% 可视化比较
figure('Name', '示例6: 可操作性比较');
subplot(1,2,1);
bar(manipulability_values);
set(gca, 'XTickLabel', config_names);
title('可操作性比较');
ylabel('可操作性');
grid on;

subplot(1,2,2);
bar(condition_numbers);
set(gca, 'XTickLabel', config_names);
title('条件数比较');
ylabel('条件数');
grid on;

%% 总结
fprintf('\n=== 示例演示完成 ===\n');
fprintf('本示例展示了以下功能:\n');
fprintf('1. 快速工作空间计算和可视化\n');
fprintf('2. 单点运动学分析\n');
fprintf('3. 轨迹规划和可视化\n');
fprintf('4. 工作空间边界分析\n');
fprintf('5. 逆运动学求解\n');
fprintf('6. 不同配置的可操作性比较\n');
fprintf('\n要进行完整的工作空间分析，请运行: simple_workspace_analysis\n');

%% 正向运动学函数
function T = forward_kinematics(theta, DH_params)
    % 基于DH参数的正向运动学
    T = eye(4);

    for i = 1:size(DH_params, 1)
        a = DH_params(i, 1);
        alpha = DH_params(i, 2);
        d = DH_params(i, 3);
        theta_offset = DH_params(i, 4);

        theta_i = theta(i) + theta_offset;

        T_i = [
            cos(theta_i), -sin(theta_i)*cos(alpha),  sin(theta_i)*sin(alpha), a*cos(theta_i);
            sin(theta_i),  cos(theta_i)*cos(alpha), -cos(theta_i)*sin(alpha), a*sin(theta_i);
            0,             sin(alpha),               cos(alpha),              d;
            0,             0,                       0,                       1
        ];

        T = T * T_i;
    end
end

%% 雅可比矩阵计算
function J = compute_jacobian(theta, DH_params)
    n = length(theta);
    J = zeros(6, n);

    T = cell(n+1, 1);
    T{1} = eye(4);

    for i = 1:n
        a = DH_params(i, 1);
        alpha = DH_params(i, 2);
        d = DH_params(i, 3);
        theta_offset = DH_params(i, 4);

        theta_i = theta(i) + theta_offset;
        T_i = [
            cos(theta_i), -sin(theta_i)*cos(alpha),  sin(theta_i)*sin(alpha), a*cos(theta_i);
            sin(theta_i),  cos(theta_i)*cos(alpha), -cos(theta_i)*sin(alpha), a*sin(theta_i);
            0,             sin(alpha),               cos(alpha),              d;
            0,             0,                       0,                       1
        ];
        T{i+1} = T{i} * T_i;
    end

    p_end = T{end}(1:3, 4);

    for i = 1:n
        z_i = T{i}(1:3, 3);
        p_i = T{i}(1:3, 4);
        J(1:3, i) = cross(z_i, p_end - p_i);
        J(4:6, i) = z_i;
    end
end

%% 可操作性分析
function [manipulability, condition_number] = analyze_manipulability(theta, DH_params)
    J = compute_jacobian(theta, DH_params);
    J_pos = J(1:3, :);
    manipulability = sqrt(det(J_pos * J_pos'));
    condition_number = cond(J_pos);
end

%% 轨迹规划函数
function [trajectory, time_stamps] = plan_joint_trajectory(theta_start, theta_end, duration, dt)
    if nargin < 4
        dt = 0.01;
    end

    time_stamps = 0:dt:duration;
    n_points = length(time_stamps);
    n_joints = length(theta_start);

    trajectory = zeros(n_points, n_joints);

    for j = 1:n_joints
        q0 = theta_start(j);
        qf = theta_end(j);

        a0 = q0;
        a1 = 0;
        a2 = 0;
        a3 = 10 * (qf - q0) / duration^3;
        a4 = -15 * (qf - q0) / duration^4;
        a5 = 6 * (qf - q0) / duration^5;

        for i = 1:n_points
            t = time_stamps(i);
            trajectory(i, j) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
        end
    end
end

%% 逆运动学求解
function [theta_sol, success] = inverse_kinematics_numerical(target_pose, DH_params, theta_init, options)
    if nargin < 4
        options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-6);
    end

    if size(target_pose, 1) == 4 && size(target_pose, 2) == 4
        target_pos = target_pose(1:3, 4);
        target_rot = target_pose(1:3, 1:3);
        target_euler = rotm2eul(target_rot, 'ZYX');
        target_vec = [target_pos; target_euler'];
    else
        target_vec = target_pose(:);
    end

    function error = objective_function(theta)
        T_current = forward_kinematics(theta, DH_params);
        current_pos = T_current(1:3, 4);
        current_rot = T_current(1:3, 1:3);
        current_euler = rotm2eul(current_rot, 'ZYX');
        current_vec = [current_pos; current_euler'];

        pos_weight = 10;
        ori_weight = 1;

        error = [pos_weight * (target_vec(1:3) - current_vec(1:3));
                ori_weight * (target_vec(4:6) - current_vec(4:6))];
    end

    try
        [theta_sol, ~, exitflag] = fsolve(@objective_function, theta_init, options);
        success = (exitflag > 0);
    catch
        theta_sol = theta_init;
        success = false;
    end
end
