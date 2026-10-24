# 时间导航决策系统使用说明

## 📚 概述

本文档说明如何使用新创建的**时间导航决策系统**，实现根据比赛剩余时间自动导航到不同位置的功能。

## 🗂️ 文件结构

```
rm_bt/
├── behavior_tree/
│   └── time_navigation.xml          # 时间导航行为树配置
├── config/
│   └── time_nav_params.yaml         # 时间导航参数配置
├── include/rm_behavior_tree/plugins/condition/
│   └── check_remain_time.hpp        # 时间检查条件节点头文件
└── plugins/condition/
    └── check_remain_time.cpp        # 时间检查条件节点实现
```

## 🔧 编译方法

### 1. 进入工作空间
```bash
cd /home/dyx/桌面/sen_xin\ 2/sen_xin
```

### 2. 编译项目
```bash
colcon build --packages-select rm_bt
```

### 3. 加载环境
```bash
source install/setup.bash
```

## 🚀 运行方法

### 方式1: 使用现有启动文件（推荐测试）

修改配置文件指向新的行为树：
```bash
# 编辑 config/node_params.yaml
nano src/bt/rm_bt/config/node_params.yaml

# 修改第3行为：
tree_xml_file: "behavior_tree/time_navigation.xml"
```

然后运行：
```bash
ros2 launch rm_bt bt_tree.launch.py
```

### 方式2: 直接运行节点
```bash
ros2 run rm_bt rm_bt --ros-args --params-file src/bt/rm_bt/config/time_nav_params.yaml
```

## 📝 决策逻辑说明

当前实现的时间导航逻辑：

```
比赛开始后（game_status == 4）：
│
├─ 剩余时间 < 240秒 (4分钟)
│  └─> 导航到位置A: (2.0, 3.0)
│
├─ 240秒 <= 剩余时间 < 360秒 (6分钟)
│  └─> 导航到位置B: (1.0, 2.0)
│
└─ 剩余时间 >= 360秒
   └─> 导航到默认位置C: (0.0, 1.0)
```

## 🎯 自定义修改

### 1. 修改时间阈值

编辑 `behavior_tree/time_navigation.xml` 文件：

```xml
<!-- 修改目标时间（秒） -->
<CheckRemainTime remain_time="{remain_time}"
                target_time="240"        <!-- 改为你想要的时间 -->
                operator="less_than"/>
```

### 2. 修改导航位置

导航目标格式：`"x,y,z,qx,qy,qz,qw"`

```xml
<!-- x=2.0, y=3.0, 朝向为单位四元数 -->
<Nav2Pose goal="2.0,3.0,0,0,0,0,1"/>
```

**坐标系说明**：
- 坐标系：`map` 坐标系
- 单位：米
- 姿态：四元数表示 (qx, qy, qz, qw)

### 3. 添加更多时间段

在 `time_navigation.xml` 中添加新的 Sequence：

```xml
<!-- 策略4: 剩余时间 < 120秒时，导航到位置D -->
<Sequence>
  <CheckRemainTime remain_time="{remain_time}"
                  target_time="120"
                  operator="less_than"/>
  <Nav2Pose goal="3.0,4.0,0,0,0,0,1"/>
</Sequence>
```

**重要**：策略按照从上到下的顺序检查，第一个满足条件的会被执行。

## 🔍 调试方法

### 1. 查看日志输出

时间检查节点会输出调试信息：
```bash
# 查看节点日志
ros2 topic echo /rosout
```

输出示例：
```
[CheckRemainTime] 剩余时间(180) < 目标时间(240) ? 是
[Nav2Pose] navigating to [2.00, 3.00]
```

### 2. 监控剩余时间话题

```bash
ros2 topic echo /remain_time
```

### 3. 监控游戏状态

```bash
ros2 topic echo /game_status
```

### 4. 使用Groot可视化（推荐）

Groot是行为树可视化工具：

1. 下载Groot: https://www.behaviortree.dev/groot
2. 连接到端口 5556
3. 实时查看行为树执行状态

## 📊 话题接口

### 输入话题（从串口节点获取）

| 话题名 | 类型 | 说明 | 发布者 |
|--------|------|------|--------|
| `/remain_time` | `std_msgs/UInt16` | 比赛剩余时间（秒） | trans_node |
| `/game_status` | `std_msgs/UInt8` | 游戏状态（4=进行中） | trans_node |

### 输出话题（发送导航指令）

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `navigate_to_pose` | Action | Nav2导航动作 |

## ⚠️ 注意事项

1. **串口通信不变**：本决策系统只是上层逻辑，底层串口通信（trans_node）保持不变
2. **坐标系统**：确保导航目标坐标在地图范围内
3. **时间单位**：所有时间单位为**秒**
4. **比赛状态**：只有当 `game_status == 4` 时才会执行导航

## 🐛 常见问题

### Q1: 编译报错找不到头文件
**A**: 确保 `check_remain_time.hpp` 在正确的路径：
```
include/rm_behavior_tree/plugins/condition/check_remain_time.hpp
```

### Q2: 运行时找不到 CheckRemainTime 节点
**A**: 检查：
1. CMakeLists.txt 是否添加了编译配置
2. 是否重新编译并 source 了环境
3. 插件库是否正确安装

### Q3: 机器人不导航
**A**: 检查：
1. `game_status` 是否为 4
2. `remain_time` 话题是否正常发布
3. Nav2 导航堆栈是否启动
4. 查看日志确认条件是否满足

### Q4: 如何测试时不等真实比赛时间
**A**: 可以手动发布时间话题进行测试：
```bash
# 发布剩余时间为200秒
ros2 topic pub /remain_time std_msgs/msg/UInt16 "data: 200"

# 发布游戏状态为进行中
ros2 topic pub /game_status std_msgs/msg/UInt8 "data: 4"
```

## 📖 扩展阅读

- BehaviorTree.CPP 文档: https://www.behaviortree.dev/
- ROS2 Navigation: https://navigation.ros.org/
- 行为树设计模式: https://www.behaviortree.dev/bt_basics/

## 💡 示例：完整的测试流程

```bash
# 终端1: 启动导航系统（假设已配置好）
ros2 launch rm_nav_bringup bringup_sim.launch.py

# 终端2: 启动决策系统
ros2 launch rm_bt bt_tree.launch.py

# 终端3: 模拟发布比赛信息（测试用）
ros2 topic pub /game_status std_msgs/msg/UInt8 "data: 4"
ros2 topic pub /remain_time std_msgs/msg/UInt16 "data: 350"

# 终端4: 监控导航状态
ros2 topic echo /navigate_to_pose/status
```

## 🎓 进阶：创建自己的条件节点

如果你想创建其他类型的条件判断（比如根据血量、敌人位置等），可以参考 `check_remain_time.cpp` 的实现方式：

1. 创建头文件定义节点接口
2. 创建cpp文件实现判断逻辑
3. 在CMakeLists.txt中添加编译配置
4. 在XML文件中使用新节点

---

**作者**: 自动生成  
**日期**: 2026-09-27  
**版本**: 1.0
