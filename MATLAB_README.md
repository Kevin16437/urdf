# ARM2.0 机械臂工作空间分析 - MATLAB工具包

基于URDF文件数据的6自由度机械臂运动学分析和工作空间可视化工具。

## 文件说明

### 主要文件

1. **`run_workspace_analysis.m`** - 主程序，一键运行完整分析
2. **`matlab_workspace_analysis.m`** - 基础工作空间分析脚本
3. **`advanced_workspace_analysis.m`** - 高级分析工具（面向对象）
4. **`kinematics_utils.m`** - 运动学工具函数集
5. **`MATLAB_README.md`** - 本说明文件

### 机械臂参数（基于URDF文件）

- **名称**: ARM2.0 6-DOF机械臂
- **自由度**: 6个旋转关节
- **连杆长度**: 
  - L1 = 0.1845m (base_link到Link1)
  - L2 = 0.1785m (Link1到Link2)
  - L3 = 0.39m (Link2到Link3)
  - L4 = 0.2665m (Link4到Link5)
  - L5 = 0.084m (Link5到Link6)

- **关节限制**:
  - j1: [-180°, 180°] (基座旋转)
  - j2: [-90°, 90°] (肩部俯仰)
  - j3: [-90°, 90°] (肘部俯仰)
  - j4: [-180°, 180°] (腕部旋转)
  - j5: [-90°, 90°] (腕部俯仰)
  - j6: [-180°, 180°] (末端旋转)

## 快速开始

### 方法1: 一键分析（推荐）

```matlab
% 运行完整的工作空间分析
run_workspace_analysis()
```

这将执行：
- 机械臂参数设置
- 用户选项配置
- 工作空间计算
- 结果分析
- 可视化显示
- 数据保存
- 报告生成

### 方法2: 基础分析

```matlab
% 运行基础工作空间分析
matlab_workspace_analysis
```

### 方法3: 高级分析

```matlab
% 使用面向对象的高级分析工具
advanced_workspace_analysis
```

## 功能特性

### 1. 工作空间计算
- 基于DH参数的正向运动学
- 关节空间采样
- 可达点集计算
- 计算时间优化

### 2. 工作空间分析
- 工作空间边界计算
- 到达距离统计
- 体积估算（凸包方法）
- 形状比例分析
- 可操作性分析（可选）

### 3. 可视化功能
- 3D工作空间点云
- 各平面投影图
- 距离分布直方图
- 高度分布分析
- 密度热图
- 机械臂运动动画

### 4. 数据输出
- MATLAB数据文件(.mat)
- 分析报告(.txt)
- 图形文件保存

## 参数配置

### 分析参数
```matlab
resolution = 0.3;      % 关节角度采样分辨率 (弧度)
max_points = 20000;    % 最大计算点数
enable_visualization = true;  % 启用可视化
generate_report = true;       % 生成报告
compute_manipulability = false; % 计算可操作性（耗时）
```

### 自定义参数
用户可以在运行时修改：
- 采样分辨率（影响精度和计算时间）
- 最大计算点数（影响完整性和计算时间）
- 可视化选项
- 可操作性分析开关

## 输出结果

### 1. 数值结果
- 可达点数量
- 工作空间边界 (X, Y, Z范围)
- 最大/最小/平均到达距离
- 工作空间体积
- 形状比例分析

### 2. 可视化图形
- 3D工作空间散点图
- XY, XZ, YZ平面投影
- 距离和高度分布直方图
- 可操作性分布图（如果启用）

### 3. 保存文件
- `ARM2_workspace_analysis_YYYYMMDD_HHMMSS.mat` - 完整数据
- `ARM2_analysis_report_YYYYMMDD_HHMMSS.txt` - 分析报告

## 高级功能

### 1. 运动学工具函数
```matlab
% 正向运动学
T = forward_kinematics_dh(theta, DH_params);

% 雅可比矩阵
J = compute_jacobian(theta, DH_params);

% 逆运动学（数值解）
[theta_sol, success] = inverse_kinematics_numerical(target_pose, DH_params, theta_init);

% 可操作性分析
[manipulability, condition_number] = analyze_manipulability(theta, DH_params);

% 轨迹规划
[trajectory, time_stamps] = plan_joint_trajectory(theta_start, theta_end, duration);
```

### 2. 工作空间边界检测
```matlab
[boundary_points, boundary_normals] = detect_workspace_boundary(reachable_points, resolution);
```

### 3. 体积计算方法
```matlab
% 凸包方法
volume = compute_workspace_volume(reachable_points, 'convhull');

% 体素化方法
volume = compute_workspace_volume(reachable_points, 'voxel');

% Alpha形状方法
volume = compute_workspace_volume(reachable_points, 'alpha_shape');
```

## 性能优化建议

### 1. 采样分辨率选择
- **粗糙分析**: 0.5弧度 (~29°) - 快速预览
- **标准分析**: 0.3弧度 (~17°) - 平衡精度和速度
- **精细分析**: 0.1弧度 (~6°) - 高精度，计算时间长

### 2. 计算点数限制
- **快速测试**: 5,000点
- **标准分析**: 20,000点
- **详细分析**: 50,000+点

### 3. 可选功能
- 可操作性分析会显著增加计算时间
- 建议先进行基础分析，再根据需要启用高级功能

## 故障排除

### 常见问题

1. **内存不足**
   - 减少max_points参数
   - 增加采样分辨率（减少采样点）

2. **计算时间过长**
   - 增加分辨率参数
   - 减少max_points
   - 关闭可操作性计算

3. **可视化问题**
   - 确保MATLAB版本支持3D绘图
   - 检查显卡驱动

4. **凸包计算失败**
   - 通常由于点数过少或分布不均
   - 增加采样点数或调整分辨率

## 扩展功能

### 1. 添加新的分析方法
用户可以在`kinematics_utils.m`中添加自定义函数

### 2. 修改机械臂参数
在`setup_robot_parameters()`函数中修改DH参数和关节限制

### 3. 自定义可视化
修改`visualize_complete_workspace()`函数添加新的图形

## 技术细节

### DH参数表
```
Link | a(m) | alpha(rad) | d(m)   | theta_offset(rad)
-----|------|------------|--------|------------------
  1  |  0   |   -π/2     | 0.1845 |        0
  2  |  0   |    π/2     |   0    |       π/2
  3  | 0.39 |     0      |   0    |        0
  4  |  0   |   -π/2     | 0.194  |        0
  5  |  0   |    π/2     |   0    |        0
  6  |  0   |     0      | 0.084  |        0
```

### 坐标系定义
- 基座坐标系：Z轴向上，X轴向前
- 末端执行器：Link6末端
- 单位：米(m)，弧度(rad)

## 版本信息
- 版本: 1.0
- 创建日期: 2025-07-02
- 兼容性: MATLAB R2018b及以上版本
- 依赖: 无特殊工具箱要求

## 联系信息
如有问题或建议，请参考代码注释或联系开发者。
