%% 机械臂工作空间分析主程序
% 一键运行完整的工作空间分析
% 基于URDF文件的ARM2.0机械臂

function run_workspace_analysis()
    clear all; close all; clc;
    
    fprintf('=== ARM2.0 机械臂工作空间分析 ===\n');
    fprintf('基于URDF文件数据\n\n');
    
    %% 1. 设置机械臂参数
    robot_params = setup_robot_parameters();
    display_robot_info(robot_params);
    
    %% 2. 用户选择分析选项
    analysis_options = get_analysis_options();
    
    %% 3. 计算工作空间
    fprintf('\n正在计算工作空间...\n');
    workspace_data = compute_reachable_workspace(robot_params, analysis_options);
    
    %% 4. 分析结果
    analysis_results = analyze_workspace_properties(workspace_data);
    display_analysis_results(analysis_results);
    
    %% 5. 可视化
    if analysis_options.enable_visualization
        visualize_complete_workspace(workspace_data, analysis_results);
    end
    
    %% 6. 保存结果
    save_results(workspace_data, analysis_results, robot_params);
    
    %% 7. 生成报告
    if analysis_options.generate_report
        generate_analysis_report(workspace_data, analysis_results, robot_params);
    end
    
    fprintf('\n=== 分析完成 ===\n');
    fprintf('结果已保存到当前目录\n');
end

%% 设置机械臂参数
function params = setup_robot_parameters()
    % 基于URDF文件的机械臂参数
    
    % 连杆长度 (从URDF文件提取)
    L1 = 0.1845;    % base_link到Link1
    L2 = 0.1785;    % Link1到Link2
    L3 = 0.39;      % Link2到Link3
    L4_y = -0.159242809429263; % Link3到Link4的y偏移
    L4_z = -0.149976330526953; % Link3到Link4的z偏移
    L4 = sqrt(L4_y^2 + L4_z^2); % Link3到Link4的距离
    L5 = 0.2665;    % Link4到Link5
    L6 = 0.084;     % Link5到Link6
    
    params.name = 'ARM2.0_6DOF';
    params.dof = 6;
    params.link_lengths = [L1, L2, L3, L4, L5, L6];
    
    % 关节限制 (从URDF文件提取)
    params.joint_limits = [
        -pi,     pi;      % j1: 基座旋转 (-180° to 180°)
        -1.57,   1.57;    % j2: 肩部俯仰 (-90° to 90°)
        -1.57,   1.57;    % j3: 肘部俯仰 (-90° to 90°)
        -pi,     pi;      % j4: 腕部旋转 (-180° to 180°)
        -1.57,   1.57;    % j5: 腕部俯仰 (-90° to 90°)
        -pi,     pi       % j6: 末端旋转 (-180° to 180°)
    ];
    
    % DH参数表 [a, alpha, d, theta_offset]
    params.DH_table = [
        0,      -pi/2,  L1,     0;      % Link1
        0,      pi/2,   0,      pi/2;   % Link2
        L3,     0,      0,      0;      % Link3
        0,      -pi/2,  L4,     0;      % Link4
        0,      pi/2,   0,      0;      % Link5
        0,      0,      L6,     0       % Link6
    ];
    
    % 关节名称
    params.joint_names = {'j1', 'j2', 'j3', 'j4', 'j5', 'j6'};
    
    % 连杆名称
    params.link_names = {'base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6'};
end

%% 显示机械臂信息
function display_robot_info(params)
    fprintf('机械臂信息:\n');
    fprintf('  名称: %s\n', params.name);
    fprintf('  自由度: %d\n', params.dof);
    fprintf('  连杆长度: ');
    fprintf('%.3f ', params.link_lengths);
    fprintf('(m)\n');
    
    fprintf('  关节限制:\n');
    for i = 1:params.dof
        fprintf('    %s: [%.2f, %.2f] rad ([%.1f°, %.1f°])\n', ...
            params.joint_names{i}, ...
            params.joint_limits(i,1), params.joint_limits(i,2), ...
            rad2deg(params.joint_limits(i,1)), rad2deg(params.joint_limits(i,2)));
    end
end

%% 获取分析选项
function options = get_analysis_options()
    fprintf('\n=== 分析选项设置 ===\n');
    
    % 默认选项
    options.resolution = 0.3;           % 关节角度采样分辨率 (弧度)
    options.max_points = 20000;         % 最大计算点数
    options.enable_visualization = true; % 启用可视化
    options.generate_report = true;     % 生成报告
    options.save_data = true;           % 保存数据
    options.compute_manipulability = false; % 计算可操作性 (耗时)
    
    % 用户可以在这里修改选项
    fprintf('当前设置:\n');
    fprintf('  采样分辨率: %.2f rad (%.1f°)\n', options.resolution, rad2deg(options.resolution));
    fprintf('  最大点数: %d\n', options.max_points);
    fprintf('  可视化: %s\n', bool2str(options.enable_visualization));
    fprintf('  生成报告: %s\n', bool2str(options.generate_report));
    
    % 询问用户是否要修改设置
    user_input = input('\n是否使用默认设置? (y/n) [y]: ', 's');
    if isempty(user_input)
        user_input = 'y';
    end
    
    if lower(user_input) == 'n'
        options = customize_options(options);
    end
end

%% 自定义选项
function options = customize_options(options)
    fprintf('\n=== 自定义设置 ===\n');
    
    % 采样分辨率
    new_res = input(sprintf('采样分辨率 [%.2f]: ', options.resolution));
    if ~isempty(new_res) && new_res > 0
        options.resolution = new_res;
    end
    
    % 最大点数
    new_max = input(sprintf('最大点数 [%d]: ', options.max_points));
    if ~isempty(new_max) && new_max > 0
        options.max_points = new_max;
    end
    
    % 可操作性分析
    comp_manip = input('计算可操作性? (y/n) [n]: ', 's');
    if strcmpi(comp_manip, 'y')
        options.compute_manipulability = true;
    end
end

%% 正向运动学函数
function T = forward_kinematics_dh(theta, DH_params)
    % 基于DH参数的正向运动学
    % 输入: theta - 关节角度 [6x1]
    %       DH_params - DH参数矩阵 [6x4] [a, alpha, d, theta_offset]
    % 输出: T - 末端执行器变换矩阵 [4x4]

    T = eye(4);

    for i = 1:size(DH_params, 1)
        a = DH_params(i, 1);
        alpha = DH_params(i, 2);
        d = DH_params(i, 3);
        theta_offset = DH_params(i, 4);

        theta_i = theta(i) + theta_offset;

        % DH变换矩阵
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
    % 计算几何雅可比矩阵
    n = length(theta);
    J = zeros(6, n);

    % 计算每个关节的变换矩阵
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

    % 末端执行器位置
    p_end = T{end}(1:3, 4);

    % 计算每列雅可比
    for i = 1:n
        % 关节i的z轴方向
        z_i = T{i}(1:3, 3);

        % 关节i的位置
        p_i = T{i}(1:3, 4);

        % 线速度部分
        J(1:3, i) = cross(z_i, p_end - p_i);

        % 角速度部分
        J(4:6, i) = z_i;
    end
end

%% 可操作性分析
function [manipulability, condition_number] = analyze_manipulability(theta, DH_params)
    % 分析机械臂的可操作性
    J = compute_jacobian(theta, DH_params);

    % 只考虑位置雅可比（前3行）
    J_pos = J(1:3, :);

    % 可操作性椭球体积
    manipulability = sqrt(det(J_pos * J_pos'));

    % 条件数
    condition_number = cond(J_pos);
end

%% 计算可达工作空间
function workspace_data = compute_reachable_workspace(params, options)
    % 生成关节角度采样点
    joint_samples = cell(params.dof, 1);
    total_combinations = 1;
    
    for i = 1:params.dof
        joint_samples{i} = params.joint_limits(i,1):options.resolution:params.joint_limits(i,2);
        fprintf('关节%d采样点数: %d\n', i, length(joint_samples{i}));
        total_combinations = total_combinations * length(joint_samples{i});
    end
    
    fprintf('理论总组合数: %d\n', total_combinations);
    fprintf('实际计算点数: %d\n', min(total_combinations, options.max_points));
    
    % 初始化结果存储
    workspace_data.reachable_points = [];
    workspace_data.joint_configs = [];
    workspace_data.manipulability_values = [];
    
    point_count = 0;
    progress_step = max(1, floor(options.max_points/20));
    
    fprintf('\n开始计算...\n');
    tic;
    
    % 嵌套循环采样关节空间
    for i1 = 1:length(joint_samples{1})
        for i2 = 1:length(joint_samples{2})
            for i3 = 1:length(joint_samples{3})
                for i4 = 1:length(joint_samples{4})
                    for i5 = 1:length(joint_samples{5})
                        for i6 = 1:length(joint_samples{6})
                            
                            if point_count >= options.max_points
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
                            T_end = forward_kinematics_dh(theta, params.DH_table);
                            end_pos = T_end(1:3, 4)';
                            
                            % 存储结果
                            workspace_data.reachable_points = [workspace_data.reachable_points; end_pos];
                            workspace_data.joint_configs = [workspace_data.joint_configs; theta];
                            
                            % 计算可操作性 (可选)
                            if options.compute_manipulability
                                [manip, ~] = analyze_manipulability(theta, params.DH_table);
                                workspace_data.manipulability_values = [workspace_data.manipulability_values; manip];
                            end
                            
                            point_count = point_count + 1;
                            
                            % 显示进度
                            if mod(point_count, progress_step) == 0
                                fprintf('进度: %d/%d (%.1f%%)\n', ...
                                    point_count, options.max_points, 100*point_count/options.max_points);
                            end
                            
                        end
                        if point_count >= options.max_points, break; end
                    end
                    if point_count >= options.max_points, break; end
                end
                if point_count >= options.max_points, break; end
            end
            if point_count >= options.max_points, break; end
        end
        if point_count >= options.max_points, break; end
    end
    
    computation_time = toc;
    fprintf('计算完成! 用时: %.2f 秒\n', computation_time);
    fprintf('实际计算点数: %d\n', size(workspace_data.reachable_points, 1));
    
    workspace_data.computation_time = computation_time;
    workspace_data.actual_points = size(workspace_data.reachable_points, 1);
end

%% 分析工作空间属性
function results = analyze_workspace_properties(workspace_data)
    points = workspace_data.reachable_points;
    
    fprintf('\n=== 工作空间属性分析 ===\n');
    
    % 基本统计
    results.point_count = size(points, 1);
    results.x_range = [min(points(:,1)), max(points(:,1))];
    results.y_range = [min(points(:,2)), max(points(:,2))];
    results.z_range = [min(points(:,3)), max(points(:,3))];
    
    % 到达距离分析
    distances = sqrt(sum(points.^2, 2));
    results.max_reach = max(distances);
    results.min_reach = min(distances);
    results.avg_reach = mean(distances);
    results.std_reach = std(distances);
    
    % 工作空间体积
    try
        [~, results.convex_volume] = convhull(points);
        fprintf('凸包体积计算成功\n');
    catch
        results.convex_volume = NaN;
        fprintf('凸包体积计算失败\n');
    end
    
    % 工作空间形状分析
    results.x_span = diff(results.x_range);
    results.y_span = diff(results.y_range);
    results.z_span = diff(results.z_range);
    results.aspect_ratio_xy = results.x_span / results.y_span;
    results.aspect_ratio_xz = results.x_span / results.z_span;
    results.aspect_ratio_yz = results.y_span / results.z_span;
    
    % 可操作性分析 (如果有数据)
    if ~isempty(workspace_data.manipulability_values)
        results.avg_manipulability = mean(workspace_data.manipulability_values);
        results.min_manipulability = min(workspace_data.manipulability_values);
        results.max_manipulability = max(workspace_data.manipulability_values);
    end
end

%% 显示分析结果
function display_analysis_results(results)
    fprintf('\n=== 分析结果 ===\n');
    fprintf('可达点数: %d\n', results.point_count);
    fprintf('工作空间范围:\n');
    fprintf('  X: [%.3f, %.3f] m (跨度: %.3f m)\n', results.x_range(1), results.x_range(2), results.x_span);
    fprintf('  Y: [%.3f, %.3f] m (跨度: %.3f m)\n', results.y_range(1), results.y_range(2), results.y_span);
    fprintf('  Z: [%.3f, %.3f] m (跨度: %.3f m)\n', results.z_range(1), results.z_range(2), results.z_span);
    
    fprintf('到达距离:\n');
    fprintf('  最大: %.3f m\n', results.max_reach);
    fprintf('  最小: %.3f m\n', results.min_reach);
    fprintf('  平均: %.3f ± %.3f m\n', results.avg_reach, results.std_reach);
    
    if ~isnan(results.convex_volume)
        fprintf('凸包体积: %.6f m³\n', results.convex_volume);
    end
    
    fprintf('形状比例:\n');
    fprintf('  X/Y: %.2f\n', results.aspect_ratio_xy);
    fprintf('  X/Z: %.2f\n', results.aspect_ratio_xz);
    fprintf('  Y/Z: %.2f\n', results.aspect_ratio_yz);
end

%% 完整可视化
function visualize_complete_workspace(workspace_data, results)
    points = workspace_data.reachable_points;
    distances = sqrt(sum(points.^2, 2));
    
    figure('Name', 'ARM2.0 工作空间完整分析', 'Position', [50, 50, 1600, 1000]);
    
    % 3D工作空间
    subplot(2,4,1);
    scatter3(points(:,1), points(:,2), points(:,3), 3, distances, 'filled');
    colorbar; colormap(jet);
    title('3D工作空间');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on; axis equal;
    
    % 各平面投影
    subplot(2,4,2);
    scatter(points(:,1), points(:,2), 2, points(:,3), 'filled');
    colorbar; title('XY投影'); xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
    
    subplot(2,4,3);
    scatter(points(:,1), points(:,3), 2, points(:,2), 'filled');
    colorbar; title('XZ投影'); xlabel('X (m)'); ylabel('Z (m)'); grid on; axis equal;
    
    subplot(2,4,4);
    scatter(points(:,2), points(:,3), 2, points(:,1), 'filled');
    colorbar; title('YZ投影'); xlabel('Y (m)'); ylabel('Z (m)'); grid on; axis equal;
    
    % 统计分析
    subplot(2,4,5);
    histogram(distances, 50);
    title('到达距离分布'); xlabel('距离 (m)'); ylabel('频次'); grid on;
    
    subplot(2,4,6);
    histogram(points(:,3), 50);
    title('高度分布'); xlabel('Z (m)'); ylabel('频次'); grid on;
    
    % 可操作性分析 (如果有数据)
    if ~isempty(workspace_data.manipulability_values)
        subplot(2,4,7);
        scatter3(points(:,1), points(:,2), points(:,3), 3, workspace_data.manipulability_values, 'filled');
        colorbar; title('可操作性分布'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        grid on; axis equal;
        
        subplot(2,4,8);
        histogram(workspace_data.manipulability_values, 50);
        title('可操作性分布'); xlabel('可操作性'); ylabel('频次'); grid on;
    else
        % 工作空间密度分析
        subplot(2,4,7);
        [N, centers] = hist3(points(:,1:2), [20, 20]);
        surf(centers{1}, centers{2}, N');
        title('XY平面密度'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('点密度');
        
        subplot(2,4,8);
        plot3(points(1:10:end,1), points(1:10:end,2), points(1:10:end,3), 'b.', 'MarkerSize', 1);
        title('工作空间点云'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        grid on; axis equal;
    end
    
    sgtitle('ARM2.0 机械臂工作空间完整分析');
end

%% 保存结果
function save_results(workspace_data, results, params)
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('ARM2_workspace_analysis_%s.mat', timestamp);
    
    save(filename, 'workspace_data', 'results', 'params');
    fprintf('结果已保存到: %s\n', filename);
end

%% 生成分析报告
function generate_analysis_report(workspace_data, results, params)
    timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    filename = sprintf('ARM2_analysis_report_%s.txt', datestr(now, 'yyyymmdd_HHMMSS'));
    
    fid = fopen(filename, 'w');
    
    fprintf(fid, '=== ARM2.0 机械臂工作空间分析报告 ===\n');
    fprintf(fid, '生成时间: %s\n\n', timestamp);
    
    fprintf(fid, '机械臂参数:\n');
    fprintf(fid, '  名称: %s\n', params.name);
    fprintf(fid, '  自由度: %d\n', params.dof);
    fprintf(fid, '  连杆长度: %.3f %.3f %.3f %.3f %.3f %.3f (m)\n', params.link_lengths);
    
    fprintf(fid, '\n分析结果:\n');
    fprintf(fid, '  计算点数: %d\n', results.point_count);
    fprintf(fid, '  计算时间: %.2f 秒\n', workspace_data.computation_time);
    fprintf(fid, '  工作空间范围:\n');
    fprintf(fid, '    X: [%.3f, %.3f] m\n', results.x_range);
    fprintf(fid, '    Y: [%.3f, %.3f] m\n', results.y_range);
    fprintf(fid, '    Z: [%.3f, %.3f] m\n', results.z_range);
    fprintf(fid, '  最大到达距离: %.3f m\n', results.max_reach);
    fprintf(fid, '  最小到达距离: %.3f m\n', results.min_reach);
    
    if ~isnan(results.convex_volume)
        fprintf(fid, '  凸包体积: %.6f m³\n', results.convex_volume);
    end
    
    fclose(fid);
    fprintf('报告已保存到: %s\n', filename);
end

%% 辅助函数
function str = bool2str(bool_val)
    if bool_val
        str = '是';
    else
        str = '否';
    end
end
