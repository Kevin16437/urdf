%% 机械臂运动学工具函数集
% 包含正向运动学、逆向运动学、雅可比矩阵等功能

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
        T_i = dh_transform(a, alpha, d, theta_i);
        T = T * T_i;
    end
end

%% DH变换矩阵
function T = dh_transform(a, alpha, d, theta)
    % 单个DH变换矩阵
    T = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),            d;
        0,           0,                      0,                     1
    ];
end

%% 雅可比矩阵计算
function J = compute_jacobian(theta, DH_params)
    % 计算几何雅可比矩阵
    % 输入: theta - 关节角度 [6x1]
    %       DH_params - DH参数矩阵 [6x4]
    % 输出: J - 雅可比矩阵 [6x6]
    
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
        T_i = dh_transform(a, alpha, d, theta_i);
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

%% 数值逆运动学求解
function [theta_sol, success] = inverse_kinematics_numerical(target_pose, DH_params, theta_init, options)
    % 数值方法求解逆运动学
    % 输入: target_pose - 目标位姿 [4x4] 或 [6x1] [x,y,z,rx,ry,rz]
    %       DH_params - DH参数矩阵
    %       theta_init - 初始关节角度
    %       options - 优化选项
    % 输出: theta_sol - 解关节角度
    %       success - 是否成功求解
    
    if nargin < 4
        options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-6);
    end
    
    % 目标位姿处理
    if size(target_pose, 1) == 4 && size(target_pose, 2) == 4
        target_pos = target_pose(1:3, 4);
        target_rot = target_pose(1:3, 1:3);
        target_euler = rotm2eul(target_rot, 'ZYX');
        target_vec = [target_pos; target_euler'];
    else
        target_vec = target_pose(:);
    end
    
    % 目标函数
    function error = objective_function(theta)
        T_current = forward_kinematics_dh(theta, DH_params);
        current_pos = T_current(1:3, 4);
        current_rot = T_current(1:3, 1:3);
        current_euler = rotm2eul(current_rot, 'ZYX');
        current_vec = [current_pos; current_euler'];
        
        % 位置误差权重更高
        pos_weight = 10;
        ori_weight = 1;
        
        error = [pos_weight * (target_vec(1:3) - current_vec(1:3));
                ori_weight * (target_vec(4:6) - current_vec(4:6))];
    end
    
    % 求解
    try
        [theta_sol, ~, exitflag] = fsolve(@objective_function, theta_init, options);
        success = (exitflag > 0);
    catch
        theta_sol = theta_init;
        success = false;
    end
end

%% 工作空间边界检测
function [boundary_points, boundary_normals] = detect_workspace_boundary(reachable_points, resolution)
    % 检测工作空间边界
    % 输入: reachable_points - 可达点集 [Nx3]
    %       resolution - 网格分辨率
    % 输出: boundary_points - 边界点
    %       boundary_normals - 边界法向量
    
    if nargin < 2
        resolution = 0.05;
    end
    
    % 创建3D网格
    x_range = [min(reachable_points(:,1)), max(reachable_points(:,1))];
    y_range = [min(reachable_points(:,2)), max(reachable_points(:,2))];
    z_range = [min(reachable_points(:,3)), max(reachable_points(:,3))];
    
    x_grid = x_range(1):resolution:x_range(2);
    y_grid = y_range(1):resolution:y_range(2);
    z_grid = z_range(1):resolution:z_range(2);
    
    [X, Y, Z] = meshgrid(x_grid, y_grid, z_grid);
    
    % 计算每个网格点到最近可达点的距离
    grid_points = [X(:), Y(:), Z(:)];
    distances = pdist2(grid_points, reachable_points);
    min_distances = min(distances, [], 2);
    
    % 重塑为网格形状
    distance_grid = reshape(min_distances, size(X));
    
    % 使用等值面提取边界
    threshold = resolution * 2;
    try
        [faces, vertices] = isosurface(X, Y, Z, distance_grid, threshold);
        boundary_points = vertices;
        
        % 计算法向量
        if nargout > 1
            boundary_normals = isonormals(X, Y, Z, distance_grid, vertices);
        end
    catch
        boundary_points = [];
        boundary_normals = [];
    end
end

%% 可操作性分析
function [manipulability, condition_number] = analyze_manipulability(theta, DH_params)
    % 分析机械臂的可操作性
    % 输入: theta - 关节角度
    %       DH_params - DH参数
    % 输出: manipulability - 可操作性指标
    %       condition_number - 条件数
    
    J = compute_jacobian(theta, DH_params);
    
    % 只考虑位置雅可比（前3行）
    J_pos = J(1:3, :);
    
    % 可操作性椭球体积
    manipulability = sqrt(det(J_pos * J_pos'));
    
    % 条件数
    condition_number = cond(J_pos);
end

%% 轨迹规划函数
function [trajectory, time_stamps] = plan_joint_trajectory(theta_start, theta_end, duration, dt)
    % 关节空间轨迹规划（五次多项式）
    % 输入: theta_start - 起始关节角度
    %       theta_end - 终止关节角度
    %       duration - 运动时间
    %       dt - 时间步长
    % 输出: trajectory - 轨迹 [N x 6]
    %       time_stamps - 时间戳
    
    if nargin < 4
        dt = 0.01;
    end
    
    time_stamps = 0:dt:duration;
    n_points = length(time_stamps);
    n_joints = length(theta_start);
    
    trajectory = zeros(n_points, n_joints);
    
    % 为每个关节规划五次多项式轨迹
    for j = 1:n_joints
        q0 = theta_start(j);
        qf = theta_end(j);
        
        % 五次多项式系数（假设起始和终止速度、加速度为0）
        a0 = q0;
        a1 = 0;
        a2 = 0;
        a3 = 10 * (qf - q0) / duration^3;
        a4 = -15 * (qf - q0) / duration^4;
        a5 = 6 * (qf - q0) / duration^5;
        
        % 计算轨迹点
        for i = 1:n_points
            t = time_stamps(i);
            trajectory(i, j) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
        end
    end
end

%% 碰撞检测（简化版）
function collision = check_self_collision(theta, DH_params, link_radii)
    % 简化的自碰撞检测
    % 输入: theta - 关节角度
    %       DH_params - DH参数
    %       link_radii - 连杆半径
    % 输出: collision - 是否发生碰撞
    
    if nargin < 3
        link_radii = 0.05 * ones(6, 1); % 默认半径
    end
    
    % 获取所有连杆位置
    positions = get_all_link_positions(theta, DH_params);
    
    collision = false;
    
    % 检查非相邻连杆间的距离
    for i = 1:size(positions, 1)-2
        for j = i+2:size(positions, 1)
            distance = norm(positions(i, :) - positions(j, :));
            min_distance = link_radii(min(i, length(link_radii))) + ...
                          link_radii(min(j, length(link_radii)));
            
            if distance < min_distance
                collision = true;
                return;
            end
        end
    end
end

%% 获取所有连杆位置
function positions = get_all_link_positions(theta, DH_params)
    % 获取所有连杆的位置
    positions = zeros(length(theta)+1, 3);
    T = eye(4);
    positions(1, :) = T(1:3, 4)'; % 基座位置
    
    for i = 1:length(theta)
        a = DH_params(i, 1);
        alpha = DH_params(i, 2);
        d = DH_params(i, 3);
        theta_offset = DH_params(i, 4);
        
        theta_i = theta(i) + theta_offset;
        T_i = dh_transform(a, alpha, d, theta_i);
        T = T * T_i;
        positions(i+1, :) = T(1:3, 4)';
    end
end

%% 工作空间体积计算
function volume = compute_workspace_volume(reachable_points, method)
    % 计算工作空间体积
    % 输入: reachable_points - 可达点集
    %       method - 计算方法 ('convhull', 'voxel', 'alpha_shape')
    % 输出: volume - 体积
    
    if nargin < 2
        method = 'convhull';
    end
    
    switch lower(method)
        case 'convhull'
            try
                [~, volume] = convhull(reachable_points);
            catch
                volume = NaN;
            end
            
        case 'voxel'
            % 体素化方法
            resolution = 0.05;
            x_range = [min(reachable_points(:,1)), max(reachable_points(:,1))];
            y_range = [min(reachable_points(:,2)), max(reachable_points(:,2))];
            z_range = [min(reachable_points(:,3)), max(reachable_points(:,3))];
            
            x_bins = x_range(1):resolution:x_range(2);
            y_bins = y_range(1):resolution:y_range(2);
            z_bins = z_range(1):resolution:z_range(2);
            
            % 统计每个体素中的点数
            [~, ~, ~, bin_counts] = histcn(reachable_points, x_bins, y_bins, z_bins);
            occupied_voxels = sum(bin_counts(:) > 0);
            volume = occupied_voxels * resolution^3;
            
        case 'alpha_shape'
            try
                alpha_shape_obj = alphaShape(reachable_points);
                volume = volume(alpha_shape_obj);
            catch
                volume = NaN;
            end
            
        otherwise
            error('未知的体积计算方法');
    end
end
