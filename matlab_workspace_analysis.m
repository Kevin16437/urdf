%% 6-DOF机械臂运动空间分析
% 基于URDF文件中的机械臂参数进行运动学分析和工作空间可视化
% 作者: AI Assistant
% 日期: 2025-07-02

clear all; close all; clc;

%% 机械臂参数定义 (基于URDF文件)
% DH参数表 [a, alpha, d, theta_offset]
% 根据URDF文件中的joint origin和axis信息计算得出

% 连杆长度和偏移量 (单位: 米)
L1 = 0.1845;    % base_link到Link1的距离
L2 = 0.1785;    % Link1到Link2的距离  
L3 = 0.39;      % Link2到Link3的距离
L4_x = 0.00025; % Link3到Link4的x偏移
L4_y = -0.159242809429263; % Link3到Link4的y偏移
L4_z = -0.149976330526953; % Link3到Link4的z偏移
L5 = 0.2665;    % Link4到Link5的距离
L6 = 0.084;     % Link5到Link6的距离

% 关节限制 (单位: 弧度)
joint_limits = [
    -pi,     pi;      % j1: 基座旋转
    -1.57,   1.57;    % j2: 肩部俯仰
    -1.57,   1.57;    % j3: 肘部俯仰
    -pi,     pi;      % j4: 腕部旋转
    -1.57,   1.57;    % j5: 腕部俯仰
    -pi,     pi       % j6: 末端旋转
];

% DH参数表 (根据URDF转换)
% [a, alpha, d, theta_offset]
DH_params = [
    0,      -pi/2,  L1,     0;      % Link1
    0,      pi/2,   0,      pi/2;   % Link2  
    L3,     0,      0,      0;      % Link3
    0,      -pi/2,  sqrt(L4_y^2 + L4_z^2), 0; % Link4
    0,      pi/2,   0,      0;      % Link5
    0,      0,      L6,     0       % Link6
];

%% 工作空间分析参数
resolution = 0.3;  % 关节角度采样分辨率 (弧度)
max_points = 20000; % 最大计算点数

fprintf('开始机械臂工作空间分析...\n');
fprintf('关节限制:\n');
for i = 1:6
    fprintf('  j%d: [%.2f, %.2f] rad\n', i, joint_limits(i,1), joint_limits(i,2));
end

%% 生成关节角度采样点
joint_samples = cell(6,1);
for i = 1:6
    joint_samples{i} = joint_limits(i,1):resolution:joint_limits(i,2);
    fprintf('关节%d采样点数: %d\n', i, length(joint_samples{i}));
end

total_combinations = prod(cellfun(@length, joint_samples));
fprintf('总组合数: %d\n', total_combinations);
fprintf('限制计算点数为: %d\n', max_points);

%% 计算可达工作空间
reachable_points = [];
point_count = 0;

% 进度显示
progress_step = max(1, floor(max_points/20));

tic;
for i1 = 1:length(joint_samples{1})
    for i2 = 1:length(joint_samples{2})
        for i3 = 1:length(joint_samples{3})
            for i4 = 1:length(joint_samples{4})
                for i5 = 1:length(joint_samples{5})
                    for i6 = 1:length(joint_samples{6})
                        
                        if point_count >= max_points
                            break;
                        end
                        
                        % 当前关节角度
                        theta = [
                            joint_samples{1}(i1),
                            joint_samples{2}(i2),
                            joint_samples{3}(i3),
                            joint_samples{4}(i4),
                            joint_samples{5}(i5),
                            joint_samples{6}(i6)
                        ];
                        
                        % 计算正向运动学
                        T_end = forward_kinematics(theta, DH_params);
                        
                        % 提取末端执行器位置
                        end_pos = T_end(1:3, 4);
                        reachable_points = [reachable_points; end_pos'];
                        
                        point_count = point_count + 1;
                        
                        % 显示进度
                        if mod(point_count, progress_step) == 0
                            fprintf('已计算: %d/%d 点 (%.1f%%)\n', ...
                                point_count, max_points, 100*point_count/max_points);
                        end
                        
                    end
                    if point_count >= max_points, break; end
                end
                if point_count >= max_points, break; end
            end
            if point_count >= max_points, break; end
        end
        if point_count >= max_points, break; end
    end
    if point_count >= max_points, break; end
end

computation_time = toc;
fprintf('计算完成! 用时: %.2f 秒\n', computation_time);
fprintf('可达点数: %d\n', size(reachable_points, 1));

%% 工作空间分析
fprintf('\n=== 工作空间分析结果 ===\n');

% 计算工作空间边界
x_range = [min(reachable_points(:,1)), max(reachable_points(:,1))];
y_range = [min(reachable_points(:,2)), max(reachable_points(:,2))];
z_range = [min(reachable_points(:,3)), max(reachable_points(:,3))];

fprintf('X轴范围: [%.3f, %.3f] m (跨度: %.3f m)\n', x_range(1), x_range(2), diff(x_range));
fprintf('Y轴范围: [%.3f, %.3f] m (跨度: %.3f m)\n', y_range(1), y_range(2), diff(y_range));
fprintf('Z轴范围: [%.3f, %.3f] m (跨度: %.3f m)\n', z_range(1), z_range(2), diff(z_range));

% 计算最大到达距离
distances = sqrt(sum(reachable_points.^2, 2));
max_reach = max(distances);
min_reach = min(distances);
fprintf('最大到达距离: %.3f m\n', max_reach);
fprintf('最小到达距离: %.3f m\n', min_reach);

% 估算工作空间体积 (使用凸包)
try
    [K, V] = convhull(reachable_points);
    fprintf('工作空间体积 (凸包): %.6f m³\n', V);
catch
    fprintf('无法计算凸包体积\n');
end

%% 可视化工作空间
figure('Name', '机械臂工作空间分析', 'Position', [100, 100, 1200, 800]);

% 3D散点图
subplot(2,2,1);
scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), ...
    1, distances, 'filled');
colorbar;
colormap(jet);
title('3D工作空间 (颜色表示到原点距离)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

% XY平面投影
subplot(2,2,2);
scatter(reachable_points(:,1), reachable_points(:,2), 1, reachable_points(:,3), 'filled');
colorbar;
title('XY平面投影 (颜色表示Z高度)');
xlabel('X (m)'); ylabel('Y (m)');
grid on; axis equal;

% XZ平面投影  
subplot(2,2,3);
scatter(reachable_points(:,1), reachable_points(:,3), 1, reachable_points(:,2), 'filled');
colorbar;
title('XZ平面投影 (颜色表示Y位置)');
xlabel('X (m)'); ylabel('Z (m)');
grid on; axis equal;

% 到达距离分布
subplot(2,2,4);
histogram(distances, 50);
title('到达距离分布');
xlabel('距离 (m)'); ylabel('频次');
grid on;

%% 保存结果
save('workspace_analysis_results.mat', 'reachable_points', 'DH_params', 'joint_limits', ...
     'x_range', 'y_range', 'z_range', 'max_reach', 'min_reach');

fprintf('\n结果已保存到 workspace_analysis_results.mat\n');

%% 正向运动学函数
function T = forward_kinematics(theta, DH_params)
    % 计算6自由度机械臂的正向运动学
    % 输入: theta - 关节角度向量 [6x1]
    %       DH_params - DH参数矩阵 [6x4]
    % 输出: T - 末端执行器的齐次变换矩阵 [4x4]
    
    T = eye(4);  % 初始化为单位矩阵
    
    for i = 1:6
        a = DH_params(i, 1);
        alpha = DH_params(i, 2);
        d = DH_params(i, 3);
        theta_offset = DH_params(i, 4);
        
        % 当前关节角度
        theta_i = theta(i) + theta_offset;
        
        % DH变换矩阵
        T_i = [
            cos(theta_i), -sin(theta_i)*cos(alpha),  sin(theta_i)*sin(alpha), a*cos(theta_i);
            sin(theta_i),  cos(theta_i)*cos(alpha), -cos(theta_i)*sin(alpha), a*sin(theta_i);
            0,             sin(alpha),               cos(alpha),              d;
            0,             0,                       0,                       1
        ];
        
        % 累积变换
        T = T * T_i;
    end
end

fprintf('\n=== 分析完成 ===\n');
fprintf('请查看生成的图形和保存的数据文件\n');
