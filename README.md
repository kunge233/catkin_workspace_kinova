# catkin_workspace_kinova

此项目基于Kinova Gen3 Lite机器人。实现了以下功能：
1. 基于MoveIt！框架的物体抓取放置功能
2. 基于Kortex API的机械臂控制功能
3. 基于Kortex API的机械臂体感手套遥控功能
本项目已经在Ubuntu 20.04 与 ROS Noetic环境中完成测试。
## github address
https://github.com/kunge233/catkin_workspace_kinova.git

# 更新日志：
## 8月12日更新
1.程序功能增强：
    增加了位姿管理功能，可以保存当前的位姿（即路点）、移动到保存的位姿；
    优化抓取放置功能，增加了可供传入的夹爪位置参数、物体名称；
    增加了轨迹规划功能，根据保存的路点可以生成且保存轨迹计划，并执行；
    增加了步进功能，可以使用指令对机械臂进行微调；
    整合了一些辅助功能，包括机械臂停止、清除故障、控制夹爪等服务。
2.文档优化：
    新增了快速使用指南，快速开始机械臂的简单控制
    重新整理相关说明文档，增加了新的内容
## 9月11日更新
- 优化报错功能
- 新增初始化前场景重置
- 解决多次抓取放置时出现的bug，每次放置后更新物体状态
- 新增刷新物体状态服务，指令为rosservice call /my_gen3_lite/refresh_object "object_id: 'object'"