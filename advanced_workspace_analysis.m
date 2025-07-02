%% 高级机械臂工作空间分析工具
% 包含多种分析方法和可视化选项
% 基于URDF文件的6-DOF机械臂

clear all; close all; clc;

%% 机械臂配置类
classdef RobotArm < handle
    properties
        DH_params       % DH参数
        joint_limits    % 关节限制
        link_lengths    % 连杆长度
        name           % 机械臂名称
    end
    
    methods
        function obj = RobotArm()
            obj.name = 'ARM2.0_6DOF';
            obj.setupParameters();
        end
        
        function setupParameters(obj)
            % 基于URDF文件设置参数
            L1 = 0.1845; L2 = 0.1785; L3 = 0.39;
            L4_y = -0.159242809429263; L4_z = -0.149976330526953;
            L5 = 0.2665; L6 = 0.084;
            
            obj.link_lengths = [L1, L2, L3, sqrt(L4_y^2 + L4_z^2), L5, L6];
            
            % 关节限制
            obj.joint_limits = [
                -pi,     pi;      % j1
                -1.57,   1.57;    % j2
                -1.57,   1.57;    % j3
                -pi,     pi;      % j4
                -1.57,   1.57;    % j5
                -pi,     pi       % j6
            ];
            
            % DH参数 [a, alpha, d, theta_offset]
            obj.DH_params = [
                0,      -pi/2,  L1,     0;
                0,      pi/2,   0,      pi/2;
                L3,     0,      0,      0;
                0,      -pi/2,  sqrt(L4_y^2 + L4_z^2), 0;
                0,      pi/2,   0,      0;
                0,      0,      L6,     0
            ];
        end
        
        function T = forwardKinematics(obj, theta)
            % 正向运动学计算
            T = eye(4);
            for i = 1:6
                a = obj.DH_params(i, 1);
                alpha = obj.DH_params(i, 2);
                d = obj.DH_params(i, 3);
                theta_offset = obj.DH_params(i, 4);
                
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
        
        function positions = getAllLinkPositions(obj, theta)
            % 获取所有连杆位置
            positions = zeros(7, 3); % 包括基座
            T = eye(4);
            positions(1, :) = T(1:3, 4)'; % 基座位置
            
            for i = 1:6
                a = obj.DH_params(i, 1);
                alpha = obj.DH_params(i, 2);
                d = obj.DH_params(i, 3);
                theta_offset = obj.DH_params(i, 4);
                
                theta_i = theta(i) + theta_offset;
                
                T_i = [
                    cos(theta_i), -sin(theta_i)*cos(alpha),  sin(theta_i)*sin(alpha), a*cos(theta_i);
                    sin(theta_i),  cos(theta_i)*cos(alpha), -cos(theta_i)*sin(alpha), a*sin(theta_i);
                    0,             sin(alpha),               cos(alpha),              d;
                    0,             0,                       0,                       1
                ];
                T = T * T_i;
                positions(i+1, :) = T(1:3, 4)';
            end
        end
    end
end

%% 工作空间分析器类
classdef WorkspaceAnalyzer < handle
    properties
        robot           % 机械臂对象
        reachable_points % 可达点
        analysis_results % 分析结果
    end
    
    methods
        function obj = WorkspaceAnalyzer(robot)
            obj.robot = robot;
            obj.reachable_points = [];
            obj.analysis_results = struct();
        end
        
        function computeWorkspace(obj, resolution, max_points)
            % 计算工作空间
            fprintf('开始计算工作空间...\n');
            
            % 生成采样点
            joint_samples = cell(6,1);
            for i = 1:6
                joint_samples{i} = obj.robot.joint_limits(i,1):resolution:obj.robot.joint_limits(i,2);
            end
            
            obj.reachable_points = [];
            point_count = 0;
            
            tic;
            for i1 = 1:length(joint_samples{1})
                for i2 = 1:length(joint_samples{2})
                    for i3 = 1:length(joint_samples{3})
                        for i4 = 1:length(joint_samples{4})
                            for i5 = 1:length(joint_samples{5})
                                for i6 = 1:length(joint_samples{6})
                                    
                                    if point_count >= max_points, break; end
                                    
                                    theta = [joint_samples{1}(i1), joint_samples{2}(i2), ...
                                           joint_samples{3}(i3), joint_samples{4}(i4), ...
                                           joint_samples{5}(i5), joint_samples{6}(i6)];
                                    
                                    T_end = obj.robot.forwardKinematics(theta);
                                    end_pos = T_end(1:3, 4);
                                    obj.reachable_points = [obj.reachable_points; end_pos'];
                                    
                                    point_count = point_count + 1;
                                    
                                    if mod(point_count, 1000) == 0
                                        fprintf('已计算: %d 点\n', point_count);
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
            
            fprintf('计算完成，用时: %.2f 秒\n', toc);
            obj.analyzeWorkspace();
        end
        
        function analyzeWorkspace(obj)
            % 分析工作空间特性
            points = obj.reachable_points;
            
            % 基本统计
            obj.analysis_results.x_range = [min(points(:,1)), max(points(:,1))];
            obj.analysis_results.y_range = [min(points(:,2)), max(points(:,2))];
            obj.analysis_results.z_range = [min(points(:,3)), max(points(:,3))];
            
            % 到达距离
            distances = sqrt(sum(points.^2, 2));
            obj.analysis_results.max_reach = max(distances);
            obj.analysis_results.min_reach = min(distances);
            obj.analysis_results.avg_reach = mean(distances);
            
            % 工作空间体积估算
            try
                [~, V] = convhull(points);
                obj.analysis_results.volume = V;
            catch
                obj.analysis_results.volume = NaN;
            end
            
            % 密度分析
            obj.analysis_results.point_density = size(points, 1) / obj.analysis_results.volume;
            
            fprintf('\n=== 工作空间分析结果 ===\n');
            fprintf('点数: %d\n', size(points, 1));
            fprintf('X范围: [%.3f, %.3f] m\n', obj.analysis_results.x_range);
            fprintf('Y范围: [%.3f, %.3f] m\n', obj.analysis_results.y_range);
            fprintf('Z范围: [%.3f, %.3f] m\n', obj.analysis_results.z_range);
            fprintf('最大到达: %.3f m\n', obj.analysis_results.max_reach);
            fprintf('最小到达: %.3f m\n', obj.analysis_results.min_reach);
            if ~isnan(obj.analysis_results.volume)
                fprintf('体积: %.6f m³\n', obj.analysis_results.volume);
            end
        end
        
        function visualizeWorkspace(obj)
            % 可视化工作空间
            points = obj.reachable_points;
            distances = sqrt(sum(points.^2, 2));
            
            figure('Name', '机械臂工作空间分析', 'Position', [50, 50, 1400, 900]);
            
            % 3D工作空间
            subplot(2,3,1);
            scatter3(points(:,1), points(:,2), points(:,3), 2, distances, 'filled');
            colorbar; colormap(jet);
            title('3D工作空间');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            grid on; axis equal;
            
            % XY投影
            subplot(2,3,2);
            scatter(points(:,1), points(:,2), 2, points(:,3), 'filled');
            colorbar;
            title('XY平面投影');
            xlabel('X (m)'); ylabel('Y (m)');
            grid on; axis equal;
            
            % XZ投影
            subplot(2,3,3);
            scatter(points(:,1), points(:,3), 2, points(:,2), 'filled');
            colorbar;
            title('XZ平面投影');
            xlabel('X (m)'); ylabel('Z (m)');
            grid on; axis equal;
            
            % YZ投影
            subplot(2,3,4);
            scatter(points(:,2), points(:,3), 2, points(:,1), 'filled');
            colorbar;
            title('YZ平面投影');
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
            histogram(points(:,3), 50);
            title('高度分布');
            xlabel('Z (m)'); ylabel('频次');
            grid on;
        end
        
        function animateRobot(obj, theta_trajectory)
            % 动画显示机械臂运动
            figure('Name', '机械臂运动动画');
            
            for i = 1:size(theta_trajectory, 1)
                clf;
                positions = obj.robot.getAllLinkPositions(theta_trajectory(i, :));
                
                % 绘制连杆
                plot3(positions(:,1), positions(:,2), positions(:,3), 'b-o', 'LineWidth', 3, 'MarkerSize', 8);
                hold on;
                
                % 绘制工作空间边界
                if ~isempty(obj.reachable_points)
                    scatter3(obj.reachable_points(:,1), obj.reachable_points(:,2), obj.reachable_points(:,3), ...
                        0.5, [0.8 0.8 0.8], 'filled', 'MarkerFaceAlpha', 0.1);
                end
                
                % 标记末端执行器
                plot3(positions(end,1), positions(end,2), positions(end,3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
                
                xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
                title(sprintf('机械臂位姿 - 帧 %d/%d', i, size(theta_trajectory, 1)));
                grid on; axis equal;
                xlim([-1, 1]); ylim([-1, 1]); zlim([0, 1.5]);
                
                pause(0.1);
            end
        end
    end
end

%% 主程序
fprintf('=== 高级机械臂工作空间分析 ===\n');

% 创建机械臂对象
robot = RobotArm();
fprintf('机械臂: %s\n', robot.name);

% 创建工作空间分析器
analyzer = WorkspaceAnalyzer(robot);

% 计算工作空间
resolution = 0.4;  % 采样分辨率
max_points = 15000; % 最大点数
analyzer.computeWorkspace(resolution, max_points);

% 可视化结果
analyzer.visualizeWorkspace();

% 保存结果
save('advanced_workspace_results.mat', 'analyzer');

% 示例：生成一个简单的运动轨迹并动画显示
fprintf('\n生成示例运动轨迹...\n');
t = linspace(0, 2*pi, 50);
theta_traj = zeros(length(t), 6);
theta_traj(:, 1) = 0.5 * sin(t);      % j1摆动
theta_traj(:, 2) = 0.3 * cos(t);      % j2摆动
theta_traj(:, 3) = 0.2 * sin(2*t);    % j3摆动

% 播放动画
analyzer.animateRobot(theta_traj);

fprintf('\n=== 分析完成 ===\n');
