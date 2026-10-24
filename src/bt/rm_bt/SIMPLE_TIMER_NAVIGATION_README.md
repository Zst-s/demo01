# 独立计时器导航系统 - 完整使用指南

## 🎯 系统概述

这是一个**完全独立**的时间导航决策系统，不依赖任何比赛相关的话题。系统内部维护一个计时器，根据启动后的运行时间自动导航到不同位置。

**核心特点**：
- ✅ 无需订阅任何比赛话题
- ✅ 内置独立计时器
- ✅ 启动即开始计时
- ✅ 下层串口通信完全不变
- ✅ 易于测试和调试

## 📁 创建的文件清单

```
rm_bt/
├── behavior_tree/
│   └── simple_timer_navigation.xml      # 简单计时器导航行为树
├── config/
│   └── simple_timer_nav.yaml           # 简化配置文件
├── include/rm_behavior_tree/plugins/
│   ├── action/
│   │   └── timer_action.hpp            # 计时器节点头文件
│   └── condition/
│       └── check_elapsed_time.hpp      # 时间检查条件节点头文件
└── plugins/
    ├── action/
    │   └── timer_action.cpp            # 计时器节点实现
    └── condition/
        └── check_elapsed_time.cpp      # 时间检查条件节点实现
```

## 🚀 快速开始

### 步骤1: 编译项目

```bash
# 进入工作空间
cd /home/dyx/桌面/sen_xin\ 2/sen_xin

# 编译 rm_bt 包
colcon build --packages-select rm_bt

# 加载环境
source install/setup.bash
```

### 步骤2: 运行决策系统

```bash
# 直接运行（推荐）
ros2 run rm_bt rm_bt --ros-args --params-file src/bt/rm_bt/config/simple_timer_nav.yaml
```

## 📝 决策逻辑说明

当前配置的时间导航逻辑：

```
启动后开始计时：
│
├─ 运行时间 > 60秒
│  └─> 导航到位置A: (2.0, 3.0)
│
├─ 30秒 < 运行时间 ≤ 60秒
│  └─> 导航到位置B: (1.0, 2.0)
│
└─ 运行时间 ≤ 30秒
   └─> 导航到默认位置C: (0.0, 1.0)
```

**行为说明**：
1. 程序启动后，计时器自动开始计时
2. 每个周期（50ms）检查一次当前运行时间
3. 根据运行时间执行相应的导航策略
4. 到达目标后会保持在该位置

## 🎯 自定义配置

### 1. 修改时间阈值

编辑 `behavior_tree/simple_timer_navigation.xml`：

```xml
<!-- 修改第1个时间点：60秒改为你想要的时间 -->
<CheckElapsedTime elapsed_time="{elapsed_time}"
                 target_time="60"        <!-- 改这里 -->
                 operator="greater_than"/>

<!-- 修改第2个时间点：30秒改为你想要的时间 -->
<CheckElapsedTime elapsed_time="{elapsed_time}"
                 target_time="30"        <!-- 改这里 -->
                 operator="greater_than"/>
```

### 2. 修改导航目标位置

格式：`"x,y,z,qx,qy,qz,qw"`（位置+四元数姿态）

```xml
<!-- 位置A: x=2.0米, y=3.0米 -->
<Nav2Pose goal="2.0,3.0,0,0,0,0,1"/>

<!-- 位置B: x=1.0米, y=2.0米 -->
<Nav2Pose goal="1.0,2.0,0,0,0,0,1"/>

<!-- 位置C: x=0.0米, y=1.0米 -->
<Nav2Pose goal="0.0,1.0,0,0,0,0,1"/>
```

### 3. 添加更多时间段

在 XML 的 `<ReactiveFallback>` 中添加新策略：

```xml
<!-- 新策略：运行 > 90秒，导航到位置D -->
<Sequence>
  <CheckElapsedTime elapsed_time="{elapsed_time}"
                   target_time="90"
                   operator="greater_than"/>
  <Nav2Pose goal="3.0,4.0,0,0,0,0,1"/>
</Sequence>
```

**⚠️ 重要**：策略从上到下检查，第一个满足条件的会被执行，所以**时间大的放在前面**！

### 4. 修改执行周期

编辑 `config/simple_timer_nav.yaml`：

```yaml
# 行为树执行周期（毫秒）
tick_period_milliseconds: 50  # 改为 100 会降低检查频率
```

## 🔍 调试方法

### 1. 查看计时器输出

计时器节点会每次输出当前时间：

```bash
# 查看日志
ros2 run rm_bt rm_bt --ros-args --params-file src/bt/rm_bt/config/simple_timer_nav.yaml

# 输出示例：
[TimerAction] 计时器启动
[TimerAction] 已运行: 0 秒
[CheckElapsedTime] 已运行(15秒) > 目标时间(60秒) ? 否
[CheckElapsedTime] 已运行(15秒) > 目标时间(30秒) ? 否
[Nav2Pose] navigating to [0.00, 1.00]
...
[TimerAction] 已运行: 35 秒
[CheckElapsedTime] 已运行(35秒) > 目标时间(30秒) ? 是
[Nav2Pose] navigating to [1.00, 2.00]
```

### 2. 可视化调试（可选）

启用 Groot 可视化工具：

1. 修改 `config/simple_timer_nav.yaml`：
```yaml
enable_groot: true  # 改为 true
```

2. 下载并运行 Groot：https://www.behaviortree.dev/groot

3. 连接到 `localhost:5556`

4. 实时观看行为树执行状态

## 📊 系统架构

```
┌─────────────────────────────────────────────┐
│         独立计时器导航决策系统                │
├─────────────────────────────────────────────┤
│                                             │
│  ┌──────────┐      ┌──────────────────┐    │
│  │  Timer   │─────>│ elapsed_time     │    │
│  │  计时器  │      │ (黑板变量)       │    │
│  └──────────┘      └──────────────────┘    │
│       │                     │               │
│       v                     v               │
│  ┌──────────────────────────────────┐      │
│  │  CheckElapsedTime 条件节点       │      │
│  │  (检查是否到达目标时间)          │      │
│  └──────────────────────────────────┘      │
│                     │                       │
│                     v                       │
│            ┌─────────────────┐              │
│            │   Nav2Pose      │              │
│            │   导航到目标    │              │
│            └─────────────────┘              │
│                     │                       │
└─────────────────────┼───────────────────────┘
                      │
                      v
            ┌──────────────────┐
            │  Nav2 导航堆栈   │
            └──────────────────┘
                      │
                      v
            ┌──────────────────┐
            │  底层运动控制    │
            │  (串口通信不变) │
            └──────────────────┘
```

## ⚙️ 完整示例配置

### 示例1: 简单的三段式导航

```xml
<!-- 0-30秒：巡逻区域1 -->
<!-- 30-60秒：巡逻区域2 -->
<!-- 60秒后：攻击位置 -->

<ReactiveFallback>
  <Sequence>
    <CheckElapsedTime elapsed_time="{elapsed_time}"
                     target_time="60"
                     operator="greater_than"/>
    <Nav2Pose goal="5.0,5.0,0,0,0,0,1"/>  <!-- 攻击位置 -->
  </Sequence>
  
  <Sequence>
    <CheckElapsedTime elapsed_time="{elapsed_time}"
                     target_time="30"
                     operator="greater_than"/>
    <Nav2Pose goal="2.0,2.0,0,0,0,0,1"/>  <!-- 巡逻区域2 -->
  </Sequence>
  
  <Sequence>
    <Nav2Pose goal="0.0,0.0,0,0,0,0,1"/>  <!-- 巡逻区域1 -->
  </Sequence>
</ReactiveFallback>
```

### 示例2: 五段式精细控制

```xml
<ReactiveFallback>
  <!-- 120秒后：最终位置 -->
  <Sequence>
    <CheckElapsedTime elapsed_time="{elapsed_time}" target_time="120" operator="greater_than"/>
    <Nav2Pose goal="6.0,6.0,0,0,0,0,1"/>
  </Sequence>
  
  <!-- 90-120秒：位置4 -->
  <Sequence>
    <CheckElapsedTime elapsed_time="{elapsed_time}" target_time="90" operator="greater_than"/>
    <Nav2Pose goal="4.5,4.5,0,0,0,0,1"/>
  </Sequence>
  
  <!-- 60-90秒：位置3 -->
  <Sequence>
    <CheckElapsedTime elapsed_time="{elapsed_time}" target_time="60" operator="greater_than"/>
    <Nav2Pose goal="3.0,3.0,0,0,0,0,1"/>
  </Sequence>
  
  <!-- 30-60秒：位置2 -->
  <Sequence>
    <CheckElapsedTime elapsed_time="{elapsed_time}" target_time="30" operator="greater_than"/>
    <Nav2Pose goal="1.5,1.5,0,0,0,0,1"/>
  </Sequence>
  
  <!-- 0-30秒：初始位置 -->
  <Sequence>
    <Nav2Pose goal="0.0,0.0,0,0,0,0,1"/>
  </Sequence>
</ReactiveFallback>
```

## 🐛 常见问题

### Q1: 编译时报错
**A**: 确保所有文件都在正确位置，然后：
```bash
rm -rf build install log
colcon build --packages-select rm_bt
source install/setup.bash
```

### Q2: 找不到 Timer 或 CheckElapsedTime 节点
**A**: 检查：
1. CMakeLists.txt 是否添加了编译配置
2. 是否重新编译并 source 了环境
3. 插件库文件是否在 `install/rm_bt/lib/` 目录下

### Q3: 机器人不导航
**A**: 检查：
1. Nav2 导航堆栈是否启动
2. 地图是否加载
3. 机器人定位是否正常
4. 查看日志确认计时器是否正常工作

### Q4: 如何重置计时器？
**A**: 计时器在节点启动时初始化，重启节点即可重置：
```bash
# Ctrl+C 停止节点，然后重新运行
ros2 run rm_bt rm_bt --ros-args --params-file src/bt/rm_bt/config/simple_timer_nav.yaml
```

### Q5: 可以同时使用多个计时器吗？
**A**: 可以！在XML中定义多个Timer节点，使用不同的输出端口名：
```xml
<Timer elapsed_time="{timer1}"/>
<Timer elapsed_time="{timer2}"/>
```

## 🎓 进阶技巧

### 1. 循环导航

如果想让机器人循环导航（比如每60秒循环一次），可以使用模运算：

创建一个新的条件节点 `CheckElapsedTimeModulo` 来实现周期性判断。

### 2. 结合其他条件

可以将时间条件和其他条件组合：

```xml
<Sequence>
  <CheckElapsedTime elapsed_time="{elapsed_time}" target_time="30" operator="greater_than"/>
  <CheckHp hp="{current_hp}" blood_threshold="200" full_blood="400"/>
  <Nav2Pose goal="1.0,1.0,0,0,0,0,1"/>  <!-- 时间>30秒且血量低时去补血 -->
</Sequence>
```

### 3. 动态调整导航策略

通过修改XML文件并重启节点，可以快速调整导航策略，无需重新编译。

## 📋 测试清单

在实际使用前，建议按此清单测试：

- [ ] 编译成功，无错误
- [ ] 节点启动正常
- [ ] 计时器输出正常
- [ ] 第一个时间点触发正常
- [ ] 第二个时间点触发正常
- [ ] 机器人能正常导航到目标位置
- [ ] 日志输出清晰易懂
- [ ] 坐标系正确（map坐标系）
- [ ] 串口通信不受影响

## 💡 完整测试命令

```bash
# 终端1: 启动导航系统（根据你的实际情况调整）
ros2 launch rm_nav_bringup bringup_sim.launch.py

# 终端2: 启动独立计时器决策系统
cd /home/dyx/桌面/sen_xin\ 2/sen_xin
source install/setup.bash
ros2 run rm_bt rm_bt --ros-args --params-file src/bt/rm_bt/config/simple_timer_nav.yaml

# 终端3: 监控日志（可选）
ros2 topic echo /rosout

# 终端4: 监控导航状态（可选）
ros2 action list
ros2 action info /navigate_to_pose
```

## ✅ 优势总结

相比于依赖比赛系统的方案，这个独立计时器方案有以下优势：

1. **完全独立** - 不依赖任何外部话题
2. **易于测试** - 启动即可测试，无需等待比赛开始
3. **灵活可控** - 时间点完全由你控制
4. **稳定可靠** - 不受串口通信影响
5. **易于调试** - 清晰的日志输出
6. **无侵入性** - 不修改任何底层代码

---

**创建日期**: 2026-09-27  
**版本**: 2.0 - 独立计时器版本  
**适用场景**: 不依赖比赛系统的自主导航测试
