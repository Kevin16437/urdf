%% ARM2.0 机械臂工作空间分析 - 简化版本
% 所有函数包含在一个文件中，避免依赖问题

clear all; close all; clc;

fprintf('=== ARM2.0 机械臂工作空间分析 ===\n');

%% 1. 机械臂参数设置
% 基于URDF文件的参数
L1 = 0.1845;    % base_link到Link1
L2 = 0.1785;    % Link1到Link2
L3 = 0.39;      % Link2到Link3
L4_y = -0.159242809429263; % Link3到Link4的y偏移
L4_z = -0.149976330526953; % Link3到Link4的z偏移
L4 = sqrt(L4_y^2 + L4_z^2); % Link3到Link4的距离
L5 = 0.2665;    % Link4到Link5
L6 = 0.084;     % Link5到Link6

% 关节限制 (弧度)
joint_limits = [
    -pi,     pi;      % j1: 基座旋转
    -1.57,   1.57;    % j2: 肩部俯仰
    -1.57,   1.57;    % j3: 肘部俯仰
    -pi,     pi;      % j4: 腕部旋转
    -1.57,   1.57;    % j5: 腕部俯仰
    -pi,     pi       % j6: 末端旋转
];

% DH参数表 [a, alpha, d, theta_offset]
DH_params = [
    0,      -pi/2,  L1,     0;      % Link1
    0,      pi/2,   0,      pi/2;   % Link2
    L3,     0,      0,      0;      % Link3
    0,      -pi/2,  L4,     0;      % Link4
    0,      pi/2,   0,      0;      % Link5
    0,      0,      L6,     0       % Link6
];

fprintf('机械臂参数:\n');
fprintf('  连杆长度: %.3f %.3f %.3f %.3f %.3f %.3f (m)\n', L1, L2, L3, L4, L5, L6);
fprintf('  关节限制:\n');
for i = 1:6
    fprintf('    j%d: [%.2f, %.2f] rad ([%.1f°, %.1f°])\n', i, ...
        joint_limits(i,1), joint_limits(i,2), ...
        rad2deg(joint_limits(i,1)), rad2deg(joint_limits(i,2)));
end

%% 2. 分析参数设置
resolution = 0.4;       % 关节角度采样分辨率 (弧度)
max_points = 15000;     % 最大计算点数

fprintf('\n分析参数:\n');
fprintf('  采样分辨率: %.2f rad (%.1f°)\n', resolution, rad2deg(resolution));
fprintf('  最大点数: %d\n', max_points);

%% 3. 计算工作空间
fprintf('\n开始计算工作空间...\n');

% 生成关节角度采样点
joint_samples = cell(6,1);
for i = 1:6
    joint_samples{i} = joint_limits(i,1):resolution:joint_limits(i,2);
    fprintf('关节%d采样点数: %d\n', i, length(joint_samples{i}));
end

total_combinations = prod(cellfun(@length, joint_samples));
fprintf('理论总组合数: %d\n', total_combinations);
fprintf('实际计算点数: %d\n', min(total_combinations, max_points));

% 初始化结果存储
reachable_points = [];
joint_configs = [];
point_count = 0;
progress_step = max(1, floor(max_points/20));

fprintf('\n开始计算...\n');
tic;

% 嵌套循环采样关节空间
for i1 = 1:length(joint_samples{1})
    for i2 = 1:length(joint_samples{2})
        for i3 = 1:length(joint_samples{3})
            for i4 = 1:length(joint_samples{4})
                for i5 = 1:length(joint_samples{5})
                    for i6 = 1:length(joint_samples{6})
                        
                        if point_count >= max_points
                            break;
                        end
                        
                        % 当前关节配置
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
                        end_pos = T_end(1:3, 4)';
                        
                        % 存储结果
                        reachable_points = [reachable_points; end_pos];
                        joint_configs = [joint_configs; theta];
                        
                        point_count = point_count + 1;
                        
                        % 显示进度
                        if mod(point_count, progress_step) == 0
                            fprintf('进度: %d/%d (%.1f%%)\n', ...
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
fprintf('实际计算点数: %d\n', size(reachable_points, 1));

%% 4. 工作空间分析
fprintf('\n=== 工作空间分析结果 ===\n');

% 基本统计
x_range = [min(reachable_points(:,1)), max(reachable_points(:,1))];
y_range = [min(reachable_points(:,2)), max(reachable_points(:,2))];
z_range = [min(reachable_points(:,3)), max(reachable_points(:,3))];

fprintf('工作空间范围:\n');
fprintf('  X: [%.3f, %.3f] m (跨度: %.3f m)\n', x_range(1), x_range(2), diff(x_range));
fprintf('  Y: [%.3f, %.3f] m (跨度: %.3f m)\n', y_range(1), y_range(2), diff(y_range));
fprintf('  Z: [%.3f, %.3f] m (跨度: %.3f m)\n', z_range(1), z_range(2), diff(z_range));

% 到达距离分析
distances = sqrt(sum(reachable_points.^2, 2));
max_reach = max(distances);
min_reach = min(distances);
avg_reach = mean(distances);
std_reach = std(distances);

fprintf('到达距离:\n');
fprintf('  最大: %.3f m\n', max_reach);
fprintf('  最小: %.3f m\n', min_reach);
fprintf('  平均: %.3f ± %.3f m\n', avg_reach, std_reach);

% 工作空间体积
try
    [~, convex_volume] = convhull(reachable_points);
    fprintf('凸包体积: %.6f m³\n', convex_volume);
catch
    fprintf('凸包体积计算失败\n');
    convex_volume = NaN;
end

%% 5. 可视化
fprintf('\n生成可视化图形...\n');

figure('Name', 'ARM2.0 工作空间分析', 'Position', [50, 50, 1400, 900]);

% 3D工作空间
subplot(2,3,1);
scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), ...
    2, distances, 'filled');
colorbar; colormap(jet);
title('3D工作空间 (颜色=距离)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

% XY平面投影
subplot(2,3,2);
scatter(reachable_points(:,1), reachable_points(:,2), 2, reachable_points(:,3), 'filled');
colorbar;
title('XY平面投影 (颜色=Z高度)');
xlabel('X (m)'); ylabel('Y (m)');
grid on; axis equal;

% XZ平面投影
subplot(2,3,3);
scatter(reachable_points(:,1), reachable_points(:,3), 2, reachable_points(:,2), 'filled');
colorbar;
title('XZ平面投影 (颜色=Y位置)');
xlabel('X (m)'); ylabel('Z (m)');
grid on; axis equal;

% YZ平面投影
subplot(2,3,4);
scatter(reachable_points(:,2), reachable_points(:,3), 2, reachable_points(:,1), 'filled');
colorbar;
title('YZ平面投影 (颜色=X位置)');
xlabel('Y (m)'); ylabel('Z (m)');
grid on; axis equal;

% 距离分布
subplot(2,3,5);
histogram(distances, 50);
title('到达距离分布');
xlabel('距离 (m)'); ylabel('频次');
grid on;

% 高度分布
subplot(2,3,6);
histogram(reachable_points(:,3), 50);
title('高度分布');
xlabel('Z (m)'); ylabel('频次');
grid on;

sgtitle('ARM2.0 机械臂工作空间完整分析');

%% 6. 保存结果
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('ARM2_workspace_simple_%s.mat', timestamp);

save(filename, 'reachable_points', 'joint_configs', 'DH_params', 'joint_limits', ...
     'x_range', 'y_range', 'z_range', 'max_reach', 'min_reach', 'avg_reach', ...
     'convex_volume', 'computation_time');

fprintf('\n结果已保存到: %s\n', filename);

%% 7. 生成简单报告
report_filename = sprintf('ARM2_report_simple_%s.txt', timestamp);
fid = fopen(report_filename, 'w');

fprintf(fid, '=== ARM2.0 机械臂工作空间分析报告 ===\n');
fprintf(fid, '生成时间: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

fprintf(fid, '分析参数:\n');
fprintf(fid, '  采样分辨率: %.2f rad (%.1f°)\n', resolution, rad2deg(resolution));
fprintf(fid, '  计算点数: %d\n', size(reachable_points, 1));
fprintf(fid, '  计算时间: %.2f 秒\n\n', computation_time);

fprintf(fid, '工作空间范围:\n');
fprintf(fid, '  X: [%.3f, %.3f] m (跨度: %.3f m)\n', x_range(1), x_range(2), diff(x_range));
fprintf(fid, '  Y: [%.3f, %.3f] m (跨度: %.3f m)\n', y_range(1), y_range(2), diff(y_range));
fprintf(fid, '  Z: [%.3f, %.3f] m (跨度: %.3f m)\n', z_range(1), z_range(2), diff(z_range));

fprintf(fid, '\n到达距离:\n');
fprintf(fid, '  最大: %.3f m\n', max_reach);
fprintf(fid, '  最小: %.3f m\n', min_reach);
fprintf(fid, '  平均: %.3f ± %.3f m\n', avg_reach, std_reach);

if ~isnan(convex_volume)
    fprintf(fid, '\n凸包体积: %.6f m³\n', convex_volume);
end

fclose(fid);
fprintf('报告已保存到: %s\n', report_filename);

fprintf('\n=== 分析完成 ===\n');

%% 正向运动学函数
function T = forward_kinematics(theta, DH_params)
    % 基于DH参数的正向运动学
    % 输入: theta - 关节角度向量 [6x1]
    %       DH_params - DH参数矩阵 [6x4] [a, alpha, d, theta_offset]
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
