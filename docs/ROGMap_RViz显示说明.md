# ROGMap 显示与下一步验证

## 2026-10-14 增量：分类原因与独立仿真

独立 `rog_map_debug.rviz` 新增 `/rog_map/classification_reasons` 的 MarkerArray，默认代替原二维分类图显示。黄色表示疑似中空，绿色表示薄表面，红色表示实心墙，紫色表示不确定障碍；均为原始分类原因，不是整车通行许可。原二维分类和 `/terrain/grid` 仍可手动打开。

此次有 C++ 发布逻辑更新，NUC 已有工作空间还需重新编译 `rog_map`、`sentry_terrain_bridge`、`rm_navigation`，只编译 RViz 所在包不会得到新话题。实车净空观察器默认不启用。

独立 Gazebo 实验与一键启动方法见 [隧道感知仿真_阶段2.md](隧道感知仿真_阶段2.md)。下文保留上一阶段的显示说明。

## 本次范围

本次只开启既有可视化发布、补齐 RViz 配置，不改变点云输入、地形分类阈值、SLAM 输入、TF、实车标定、串口或速度控制。

修改文件：

- `src/rm_nav_bringup/config/reality/rog_map_real.yaml`：`rog_map.visualization.enable: true`，可视化频率为 2 Hz。
- `src/rm_navigation/rm_navigation/rviz/rog_map_mapping.rviz`：建图视图增加三维占据和二维分类，将 SLAM 全局地图绘制在后面。
- `src/rm_navigation/rm_navigation/rviz/nav2.rviz`：增加三维占据和可选分类显示，默认关闭已停用的虚拟扫描显示。
- `src/rm_navigation/rm_navigation/rviz/rog_map_debug.rviz`：新增独立调试视图，固定坐标系为 `odom`，不依赖 SLAM 发布 `map -> odom`。

## 在 NUC 上使用

把这些源文件同步到 NUC 对应工作空间，不要复制本电脑的 `build/`、`install/`、`log/`。在 NUC 工作空间根目录、已经正确加载该机 ROS 环境的终端执行：

```bash
colcon build --symlink-install --packages-select rm_navigation rm_nav_bringup
source install/setup.bash
```

以上命令用于其余依赖已经编译成功的现有工作空间。如果是首次部署，需先编译依赖。复制来的旧 CMake 缓存和绝对符号链接可能指向原电脑，不能靠再次 source 修复。

关闭旧 RViz 并按原来的实车命令重启（保留你原来的 LIO、地图与定位选择）。建图使用 `mode:=mapping perception:=rog_map nav_rviz:=True` 时，启动文件会选择建图视图。导航视图也包含新显示项。

只想观察、不启动第二套导航或底盘节点时，在已有系统运行期间另开终端：

```bash
source install/setup.bash
rviz2 -d "$(ros2 pkg prefix --share rm_navigation)/rviz/rog_map_debug.rviz"
```

在仿真或使用 `/clock` 的 rosbag 回放中，RViz 也需要仿真时间：

```bash
rviz2 -d "$(ros2 pkg prefix --share rm_navigation)/rviz/rog_map_debug.rviz" --ros-args -p use_sim_time:=true
```

普通实车不加 `use_sim_time:=true`。独立视图只含观察工具，不含发送导航目标工具。机器人远离 odom 原点时，可用 Focus Camera 聚焦当前点云，或在 Views 中选择合适的 Target Frame；调整相机不改变 TF。

## 各个显示项到底是什么

| 显示/话题 | 含义 | 不能据此推断什么 |
|---|---|---|
| `/rog_map/raw_occupied` | 概率占据栅格中的占据体素中心；独立视图默认显示为橙色 | 不是原始雷达点云，也不是未经降采样的输入 |
| `/rog_map/occupied` | ROGMap 的占据查询结果，查询还考虑虚拟地板/天花板边界 | 不是全局 SLAM 地图；不一定每个点都对应真实回波 |
| `/rog_map/layer_type` | 三维数据投影后的二维分类；独立视图默认开启 | 不是 ESDF，也不是可靠的隧道安全认证 |
| `/terrain/grid` | 适配器交给 Nav2 TerrainLayer 的局部二维栅格 | 不是持久 `/map`，不会发布 `map -> odom` |
| `/scan` | 当前 SLAM 输入的扫描话题 | 不因为开启 ROGMap 显示就变成 ROGMap 的输出 |
| `/map` | SLAM 或 map_server 发布的全局占据栅格 | 不一定与局部三维分类结论相同 |

`layer_type` 借用了 OccupancyGrid 消息，但数据是分类编码，不是占据概率：

- `0`：UNKNOWN，未知。**在这个调试话题中 0 不是自由空间！**
- `33`：FREE。
- `66`：PASSABLE。
- `100`：OCCUPIED。

RViz 用 costmap 色表区分数值，不会自动显示上述语义标签。不要把 `layer_type` 接到 Nav2 当作标准占据概率地图。标准接口仍是 `/terrain/grid`，它单独处理未知值。

PASSABLE 包含薄表面等类别，不只包含 `HOLLOW_TUNNEL`。当前视图没有单独输出隧道原因标签，无法仅靠一种颜色区分“薄地面”和“疑似洞口”。当前 ESDF 与 field 仍关闭，没有新增虚假的 ESDF 显示项。

二维分类、局部地形、全局地图和代价地图重叠时，请逐个勾选比较。建图视图把 `/map` 放在后面，但多个半透明栅格叠加仍会混色；独立调试视图默认不叠加 `/terrain/grid`。

## 空白时按顺序检查

```bash
ros2 node info /sentry_terrain
ros2 param get /sentry_terrain rog_map.visualization.enable
ros2 topic info /rog_map/raw_occupied -v
ros2 topic info /rog_map/layer_type -v
ros2 topic info /terrain/grid -v
```

若有命名空间，以 `ros2 node list` 的实际节点名为准。先确认 `/terrain/grid` 有新数据，再检查 ROGMap 可视化。可视化节点只在有匹配订阅者且地图非空时生成相应数据；仅出现话题名不能证明有有效地图。

```bash
ros2 topic echo /rog_map/layer_type --once --field info --qos-reliability best_effort --qos-durability volatile
```

独立视图显示正常、导航视图空白时，检查 `map -> odom`，不要通过乱改 frame_id 冒充坐标变换。所有 ROGMap 调试显示采用 Best Effort + Volatile，与发布端匹配。

2 Hz 只是可视化上限，不保证在 NUC 满载时达到该频率，也不改变地图更新频率。查看完后关闭多余的三维显示和第二个 RViz；需要彻底停止调试发布时，将 visualization.enable 改回 false 并重启相应节点。

## 下一步优化方向（尚未在本次实现）

1. **先输出分类原因。** 在二维类别之外，增加薄表面、稀疏回波、疑似隧道、实心墙等原因，以及观测数量/置信度。用录包重复比较入口前、中、后的结果。
2. **再验证整车能否通过。** 使用实际车宽、车高、安全余量、地面支撑、连续已观测自由体素判断通道；未知不能等同于自由。单根射线穿过横梁下方，不代表整车能通过。
3. **最后做有条件的地图融合。** 当前 TerrainLayer 使用保守的最大代价融合，局部 FREE 不会清掉静态地图的墙。先做只显示、不影响控制的冲突层；验证后才考虑限范围、多帧一致、可撤销的静态假障碍修正，且不能覆盖新检测到的动态障碍。
4. **通过回放和仿真验收后再实车低速测试。** 至少覆盖可通过洞口、过低横梁、过窄入口、稀疏未观测区域、突然出现的障碍物。核对原始 SLAM 地图、局部分类、最终 costmap 和规划路径，不以“RViz 看见了洞”作为成功标准。

不优先更换 LIO、全局规划器或控制器；当前要解决的是“通行证据是否成立、是否真正进入规划地图”。ESDF 提供距离信息，但不能自动替代净空判断或清除 SLAM 假墙。

HERO 的公开 README 也将点云后端投影、局部代价地图和轨迹优化分为不同模块，可参考其架构，但不把其通行能力或车辆参数直接套到本车：[HERO_2026_Sentry_NAV](https://github.com/LiuJinbo1027/HERO_2026_Sentry_NAV)。上述步骤是针对本仓库当前实现的建议，不是已实现功能。

## 本地验证记录（2026-10-13）

- YAML 解析与话题/QoS/固定坐标系检查通过。
- 在独立临时 build/install 中构建 `rm_navigation`、`rog_map`、`sentry_terrain_bridge`、`rm_nav_bringup`，4 个包成功；未覆盖原 build/install。
- 现有 `test_projection_layer`、`test_grid_contract`、`test_grid_scan` 三个测试套件通过。
- 使用仅本机通信的独立 ROS Domain 217 发送合成静止墙面/地面点云与同时间戳里程计，测试结束已关闭测试节点：
  - `/rog_map/raw_occupied`、`/rog_map/occupied` 均收到非空点云，本次样本各 1451 个体素中心。
  - `/rog_map/layer_type` 收到 201×201 栅格，值包含 0、33、66、100。
  - `/terrain/grid` 收到 201×201 栅格，值包含 -1、0、100。
  - 输出坐标系均为 odom；没有启动控制、TF 或虚拟扫描发布者。
- 尚未在 NUC 上实测性能、未进行 RViz 图形界面的渲染验收、未验证真实洞口净空或自动穿洞。以上测试不能作为安全通行证明。
