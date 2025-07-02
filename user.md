来为您详细介绍其他几个功能的用法：
# 重新编译
colcon build

# Source环境
source install/setup.bash



ros2 launch arm2_description test_display.launch.py


# 检查mock_components是否安装
ros2 pkg list | grep mock

# 如果没有安装，需要安装
sudo apt install ros-jazzy-ros2-control-test-assets


ros2 launch arm2_description minimal_control_test.launch.py



ros2 launch arm2_description test_display.launch.py



# 在另一个终端
source install/setup.bash
ros2 topic echo /joint_states

🚀 完整功能使用指南
1. 基础可视化与关节控制
# 启动基础可视化（推荐先从这个开始）
ros2 launch arm2_description display_with_gui.launch.py
功能说明：

在RViz中显示机器人模型
提供GUI滑条控制每个关节
实时显示机器人状态
2. 简化MoveIt演示
# 启动简化版MoveIt（如果完整版有问题）
ros2 launch arm2_description simple_moveit.launch.py
3. 笛卡尔路径规划功能
# 启动笛卡尔路径规划
ros2 launch arm2_description cartesian_planning.launch.py
可用服务命令：

使用流程：

先规划路径（直线或圆形）
在RViz中查看路径可视化
执行路径
4. 工作空间分析功能
分析工作空间：

查看结果：

在RViz中可视化：

在RViz中添加PointCloud2显示
设置Topic为 /reachable_workspace
设置Frame为 base_link
5. 交互式示教功能
使用方法：

在RViz中找到红色球形标记
拖拽标记到目标位置
释放鼠标，机器人会自动规划并移动
6. 完整功能演示
可选参数：

7. 系统测试与诊断
8. 监控和调试命令
9. 高级功能
查看机器人描述：

手动控制关节：

查看可达空间点云统计：

10. 故障排除命令
11. 性能调优
工作空间分析性能调整：

修改  scripts/workspace_analyzer.py 中的 resolution 参数
调整 max_points 限制计算量
路径规划精度调整：

修改  scripts/cartesian_path_planner.py 中的插值点数量
12. 实际使用建议
推荐使用顺序：

先运行 display_with_gui.launch.py 确保基础功能正常
再尝试 simple_moveit.launch.py 测试MoveIt集成
然后使用各个专门功能的启动文件
最后使用 complete_demo.launch.py 体验完整功能
常见问题解决：

如果RViz显示异常，重启RViz
如果服务调用失败，检查对应节点是否运行
如果路径规划失败，检查目标位置是否在可达范围内
这样您就可以充分体验机械臂仿真的所有功能了！