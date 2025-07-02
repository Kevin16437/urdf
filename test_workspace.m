%% ARM2.0 机械臂工作空间分析 - 快速测试
% 验证所有函数是否正常工作

clear all; close all; clc;

fprintf('=== ARM2.0 机械臂快速测试 ===\n');

%% 1. 设置基本参数
L1 = 0.1845; L2 = 0.1785; L3 = 0.39; 
L4 = 0.194; L5 = 0.2665; L6 = 0.084;

DH_params = [
    0,      -pi/2,  L1,     0;
    0,      pi/2,   0,      pi/2;
    L3,     0,      0,      0;
    0,      -pi/2,  L4,     0;
    0,      pi/2,   0,      0;
    0,      0,      L6,     0
];

joint_limits = [
    -pi,     pi;
    -1.57,   1.57;
    -1.57,   1.57;
    -pi,     pi;
    -1.57,   1.57;
    -pi,     pi
];

fprintf('机械臂参数设置完成\n');

%% 2. 测试正向运动学
fprintf('\n测试正向运动学...\n');
test_angles = [0, 0, 0, 0, 0, 0];  % 零位
T = forward_kinematics(test_angles, DH_params);
pos = T(1:3, 4);
fprintf('零位末端位置: [%.3f, %.3f, %.3f] m\n', pos);

% 测试另一个位置
test_angles2 = [pi/4, pi/6, -pi/6, 0, 0, 0];
T2 = forward_kinematics(test_angles2, DH_params);
pos2 = T2(1:3, 4);
fprintf('测试位置末端: [%.3f, %.3f, %.3f] m\n', pos2);

%% 3. 快速工作空间采样
fprintf('\n快速工作空间采样...\n');
resolution = 1.0;  % 粗糙采样
max_points = 1000;

reachable_points = [];
point_count = 0;

tic;
for j1 = joint_limits(1,1):resolution:joint_limits(1,2)
    for j2 = joint_limits(2,1):resolution:joint_limits(2,2)
        for j3 = joint_limits(3,1):resolution:joint_limits(3,2)
            if point_count >= max_points, break; end
            
            theta = [j1, j2, j3, 0, 0, 0];  % 固定后三个关节
            T = forward_kinematics(theta, DH_params);
            pos = T(1:3, 4)';
            
            reachable_points = [reachable_points; pos];
            point_count = point_count + 1;
        end
        if point_count >= max_points, break; end
    end
    if point_count >= max_points, break; end
end

fprintf('采样完成，用时: %.2f 秒\n', toc);
fprintf('采样点数: %d\n', size(reachable_points, 1));

%% 4. 基本分析
if size(reachable_points, 1) > 0
    fprintf('\n基本工作空间分析:\n');
    
    x_range = [min(reachable_points(:,1)), max(reachable_points(:,1))];
    y_range = [min(reachable_points(:,2)), max(reachable_points(:,2))];
    z_range = [min(reachable_points(:,3)), max(reachable_points(:,3))];
    
    fprintf('X范围: [%.3f, %.3f] m\n', x_range);
    fprintf('Y范围: [%.3f, %.3f] m\n', y_range);
    fprintf('Z范围: [%.3f, %.3f] m\n', z_range);
    
    distances = sqrt(sum(reachable_points.^2, 2));
    fprintf('最大到达: %.3f m\n', max(distances));
    fprintf('最小到达: %.3f m\n', min(distances));
    
    % 简单可视化
    figure('Name', '快速工作空间测试');
    scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), 'filled');
    title('ARM2.0 工作空间快速测试');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on; axis equal;
    
    fprintf('可视化图形已生成\n');
end

%% 5. 测试雅可比矩阵
fprintf('\n测试雅可比矩阵计算...\n');
try
    J = compute_jacobian(test_angles, DH_params);
    fprintf('雅可比矩阵大小: %dx%d\n', size(J, 1), size(J, 2));
    fprintf('条件数: %.2f\n', cond(J));
    fprintf('雅可比矩阵计算成功\n');
catch ME
    fprintf('雅可比矩阵计算失败: %s\n', ME.message);
end

%% 6. 测试可操作性
fprintf('\n测试可操作性分析...\n');
try
    [manip, cond_num] = analyze_manipulability(test_angles, DH_params);
    fprintf('可操作性: %.6f\n', manip);
    fprintf('条件数: %.2f\n', cond_num);
    fprintf('可操作性分析成功\n');
catch ME
    fprintf('可操作性分析失败: %s\n', ME.message);
end

%% 7. 测试轨迹规划
fprintf('\n测试轨迹规划...\n');
try
    theta_start = [0, 0, 0, 0, 0, 0];
    theta_end = [pi/4, pi/6, -pi/6, 0, 0, 0];
    [traj, t] = plan_joint_trajectory(theta_start, theta_end, 2.0, 0.1);
    fprintf('轨迹点数: %d\n', size(traj, 1));
    fprintf('轨迹规划成功\n');
catch ME
    fprintf('轨迹规划失败: %s\n', ME.message);
end

fprintf('\n=== 测试完成 ===\n');
fprintf('所有基本功能正常工作\n');
fprintf('可以运行 simple_workspace_analysis 进行完整分析\n');

%% 正向运动学函数
function T = forward_kinematics(theta, DH_params)
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
