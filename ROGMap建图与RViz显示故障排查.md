# ROGMap 感知、全局建图与 RViz 故障修复及检查记录

记录日期：2026-10-11；复查与修复：2026-10-12  
工作区：`/home/doublegun/zzs_rogMAP`  
启动组合：`mode:=mapping lio:=fastlio perception:=rog_map`

## 当前结论

**当前代码以文末“2026-10-12 架构复查”为准。** ROGMap 继续输出滚动局部 `/terrain/grid`，SLAM Toolbox 改回订阅真实点云生成的 `/scan` 来构建持久全局 `/map`；`/rog_map/scan` 不再用于建图。这样把两种地图的职责分开。下方旧 `/rog_map/scan` 频率、SLAM 静止累计和 6.6 秒首图等数字是改造前的历史验证记录，不应作为当前性能承诺。当前方案已完成构建和静态检查，尚未在洞口现场重新启动验证。

之前已经修正 ROGMap 启动等待和建图模式误启动 Nav2；当时实测局部图约 3.8 秒出现，全局图约 6.6 秒出现。两张图是不同算法的输出，尺寸和障碍数不应逐格相等。没有启动串口底盘控制。

用户反馈墙和箱体检测“有改善但仍漏检”。本记录中的验证只确认局部图障碍数增加、全局图持续累计，**没有确认现场每一处墙或箱体都被识别**；剩余漏检还需对照具体位置的点云和局部栅格检查。

## 故障现象

- 启动时 Nav2 报 `base_link` 到 `map` 的 TF 等待超时；随后 `/map` 一度为 `0×0`。
- RViz 看不到扫描点或点云，显示列表里只有原来的 `Map`，没有直观的 ROGMap 显示项。
- ROGMap 节点有时能短暂发布数据，随后反复报告“丢弃点云：时间差超过 0.5 s”，`/terrain/grid` 和 `/rog_map/scan` 停更。
- 后续虽然局部 `ROGMap Terrain` 持续显示墙/箱体，SLAM 全局 `/map` 只保留初始的一小片，看起来没有把局部障碍累积进全局图。

这里的几个话题分工不同：`/terrain/grid` 是 ROGMap 的局部二维地形栅格，`/cloud_registered` 是 FAST-LIO 配准后的点云；`/map` 是 SLAM Toolbox 生成的全局地图。旧方案另有从地形栅格反算的 `/rog_map/scan`，现已停用。**不会另有一个名叫 `/ROGMAP` 的地图话题。**

## 定位过程与原因

1. 检查话题与数据内容时，曾看到 `/terrain/grid` 为有效的 `201×201` 栅格，而 `/map` 为 `0×0`。这说明 ROGMap 地形输出和全局 SLAM 地图不是同一故障点。
2. 隔离复现表明：SLAM Toolbox 收到第一帧没有有效障碍回波的扫描后可能初始化出空地图；车静止时，后续扫描又因最小移动距离门限不被接纳，于是 `/map` 保持零尺寸。给隔离的 SLAM 输入有效扫描后，可以生成非零尺寸地图。
3. 后续停更与计算延迟有关。原配置 `decay.active_list_enable: false` 使每次更新遍历整个三维滚动地图；节点单线程处理期间，后续点云超过 `max_input_age: 0.5`，被安全检查丢弃。问题不能靠放宽点云时效门限掩盖。
4. 原 RViz 配置没有面向本组合的清晰显示：点云显示指向当前并不存在的 `/segmentation/ground`、`/segmentation/obstacle`，原扫描显示也不是 `/rog_map/scan`。
5. 后续出现过“点云可见，但 ROGMap Scan 不显示”：检查 ROS 图时，`/rog_map/scan` 只有 SLAM 订阅，没有 RViz 订阅。查看 RViz 当前窗口发现，名为 `ROGMap Scan` 的显示项被临时改到了 `/slam_toolbox/scan_visualization`；磁盘上的专用配置仍指向 `/rog_map/scan`。仅看显示项名称、勾选框和 `Status: Ok`，不能证明它订阅的是期望话题。
6. 墙和箱体虽然有点云回波，`ROGMap Terrain` 却偏白：原配置把垂直厚度不超过 `0.25 m` 的占据列一律当作可通行薄面。稀疏墙/箱体在单个 XY 格里可能只命中一个高度格，也会被误归入此类。现场约 `201×201` 的局部图只有约 300–370 个障碍格，而同帧扫描仍有约 200 条有效障碍回波。
7. 最后的“局部图没有进全局图”并非话题未连接：现场确认 `slam_toolbox` 已按 Best Effort 订阅 `/rog_map/scan`，该扫描持续发布，`/map` 也每 3 秒发布一次。问题是建图配置仍要求 `minimum_travel_distance: 0.1 m` 或 `minimum_travel_heading: 0.1 rad`；机器人基本静止时，SLAM 图仅有约 2 个扫描节点，之后即使 ROGMap 不断补出障碍，扫描也未被累计。现场对比：局部图为 `201×201`、约 482 个障碍格，而 `/map` 固定在 `65×84`、51 个障碍格。运行中用 `ros2 param set` 设为零没有让已有进程开始累计；修改启动配置并重启后才验证到持续入图。

虚拟扫描是从局部栅格按射线提取障碍终点，交给 SLAM Toolbox 建全局图，不是直接把整张 `201×201` 栅格复制进 `/map`。当前扫描每束遇到第一个占据格或未知格即停止；被遮挡的障碍不会从同一视点同时画进全局图，机器人移动并从其他视角观察后才可能补齐。

启动最初数十秒出现 `map` TF 等待信息可以是建图尚未初始化的暂态；判断故障应结合持续的扫描、地形图和 `/map` 尺寸，而非只看第一条超时日志。

## 早期修复记录（虚拟扫描方案已停用）

| 位置 | 改动 | 作用 |
| --- | --- | --- |
| `src/rm_perception/sentry_terrain_bridge/include/sentry_terrain_bridge/grid_scan.hpp`、`src/rm_perception/sentry_terrain_bridge/src/terrain_node.cpp` | 仅当扫描含有限且处于量程内的障碍回波时，才发布到 `/rog_map/scan`；无回波时继续发布地形图 | 防止空首帧把 SLAM 初始化成 `0×0` 地图 |
| `src/rm_nav_bringup/config/reality/rog_map_real.yaml` | 将 `decay.active_list_enable` 设为 `true`，保留 `max_input_age: 0.5` | 衰减时只遍历活跃体素，恢复持续更新，同时继续拒绝过期点云 |
| `src/rm_navigation/rm_navigation/rviz/rog_map_mapping.rviz` | 增加并启用 `ROGMap Terrain`、`ROGMap Scan`、`LiDAR Registered Cloud`，分别接 `/terrain/grid`、`/rog_map/scan`、`/cloud_registered` | 在 RViz 中分别观察局部栅格、扫描和实际点云 |
| 同一 RViz 配置 | 将扫描设为黄色、`0.06 m` 球点，并关闭建图视图中的 Global/Local Costmap 图层 | 让扫描点从绿色点云和代价地图颜色中明显区分出来 |
| `src/rm_perception/rog_map/src/rog_map/projection_layer.cpp`、配置与 `rog_map_real.yaml` | 增加可配置的“高处稀疏薄面判障碍”：仅当薄面高度达到雷达当前高度以上 `0.15 m` 时判占据；库默认关闭，实车配置开启 | 补出墙和箱体的稀疏回波，同时保留低处地面为可通行 |
| `src/rm_nav_bringup/launch/bringup_real.launch.py` | 仅在 `perception:=rog_map` 建图时覆盖 SLAM 最小位移、最小转角门限为 `0.0`；其他组合仍沿用 mapper YAML 的 `0.1`，时间间隔仍为 `0.2 s` | 机器人静止时仍让逐步完善的 ROGMap 虚拟扫描进入 SLAM 全局地图 |
| `src/rm_nav_bringup/launch/bringup_real.launch.py`、`src/rm_navigation/rm_navigation/launch/bringup_rm_navigation.py` | 建图且选择 ROGMap 时加载专用 RViz 配置 | 用原启动命令即可得到正确显示 |

同时补充了扫描有效性与投影分类测试。`rog_map` 的 CMake 增加 PCL 头文件路径，使清理旧构建缓存后的全新编译也能通过；桥接包的 CMake 包含路径也已修正。

## 早期重新构建与启动记录（检查命令含已停用话题）

先用 `Ctrl+C` 停止旧的 launch，避免重复启动雷达、LIO、SLAM 和 RViz。随后在终端执行：

```bash
cd /home/doublegun/zzs_rogMAP
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to sentry_terrain_bridge rm_nav_bringup rm_navigation
colcon test --packages-select rog_map sentry_terrain_bridge
colcon test-result --all --verbose
source install/setup.bash
ros2 launch rm_nav_bringup bringup_real.launch.py world:=my_map9 mode:=mapping lio:=fastlio lio_rviz:=False nav_rviz:=True perception:=rog_map
```

另开一个终端检查（同样先 `source /opt/ros/humble/setup.bash` 和工作区的 `install/setup.bash`）：

```bash
ros2 topic list | rg '/terrain/grid|/rog_map/scan|/cloud_registered|^/map$'
ros2 topic hz /terrain/grid
ros2 topic hz /rog_map/scan
ros2 topic hz /cloud_registered
ros2 topic echo --once /map --field info
ros2 topic info /rog_map/scan -v
ros2 param get /slam_toolbox scan_topic
ros2 param get /slam_toolbox minimum_travel_distance
ros2 param get /slam_toolbox minimum_travel_heading
```

`ros2 topic info /rog_map/scan -v` 应同时列出 `sentry_terrain` 发布者，以及 `slam_toolbox` 和 `rviz` 两个订阅者。如果没有 `rviz`，先展开 RViz 的 `ROGMap Scan → Topic`，确认它确实是 `/rog_map/scan`；当前会话里改过 Topic 时，可重新加载磁盘上的专用配置。不要只凭显示项名称判断。

上述检查中，`scan_topic` 应为 `/rog_map/scan`，两个最小移动门限在此启动组合下应均为 `0.0`。隔数秒重复检查 `/map` 的宽高和障碍格数量，可以判断全局图是否仍在累计；仅看 `/map` 发布频率不足以证明新扫描已入图。RViz 中 `ROGMap Terrain` 显示局部 `/terrain/grid`，原有 `Map` 显示 SLAM 的全局 `/map`。

`/rog_map/scan` 使用 Best Effort QoS。自行编写订阅程序时需选择兼容的 QoS；默认 Reliable 订阅可能收不到扫描，即使话题实际在发布。若要用 `ros2 topic echo` 查看扫描，可显式加 `--qos-reliability best_effort`。

## 验证结果与边界

- 最终启动后，`/cloud_registered`、`/terrain/grid`、`/rog_map/scan` 均持续约 **10 Hz**；RViz 已订阅这三个话题。
- 在后续复查中，恢复正确 Topic 后再次确认 `/rog_map/scan` 约 **10 Hz**、RViz 正在订阅；调整样式后，RViz 画面中可见独立的黄色扫描点。
- 对静态障碍进行隔离对比时，原规则约 350–370 个障碍格，新规则约 800–900 个；新增格主要沿墙和箱体轮廓分布。主建图重启后实测局部图 888 个障碍格、扫描 721 束中 299 束有效，全局 `/map` 也非零尺寸；`/terrain/grid` 仍约 **10 Hz**。新增的 ROGMap 分类回归测试通过。
- 修正静止扫描门限并重启后，机器人位姿基本不变，SLAM 图已累计到约 115 个扫描节点；`/map` 从修复前固定的 `65×84`、51 个障碍格，增长到 `147×119`、573 个障碍格，稍后又到 `147×120`、865 个障碍格。这验证了局部障碍正经由 `/rog_map/scan` 持续进入全局图；全局图的范围和障碍数不会与滚动局部栅格逐格相同。
- 将门限覆盖限定为 `perception:=rog_map` 后再次重启验证：`/slam_toolbox` 的两个运行时门限均为 `0.0`，扫描来源是 `/rog_map/scan`；静止时全局 `/map` 达 `162×108`、965 个障碍格，局部 `/terrain/grid` 为 `201×201`、734 个障碍格。共享 mapper YAML 仍保持原始 `0.1 m`、`0.1 rad` 门限。
- 抽样的 ROGMap 栅格为 `201×201`，约 514 个占据格；扫描每帧 721 束，其中约 258 束有有效回波。`/map` 已非零尺寸。用户确认 RViz 中相关显示已出现。
- `colcon test-result --all --verbose` 汇总为 **12 项测试、0 错误、0 失败、0 跳过**；包括 ROGMap 投影分类回归测试、桥接扫描/栅格测试及配置检查。建图 launch 的 Python 语法和 mapper YAML 解析也已检查。
- RViz 日志仍有一次 `indexed_8bit_image` 的 GLSL 链接错误；软件渲染试验也出现相同错误。目前话题、订阅和用户所见显示均正常，因此未修改系统图形驱动或 ROS 安装。如果日后地图图层再次空白，应单独检查 RViz 图层状态和图形渲染日志。
- 最终启动日志曾在地图初始化前短暂出现 `Robot is out of bounds of the costmap!` 和 `Can't update static costmap layer, no map received`，在本次日志中没有持续重复到建图稳定阶段；若导航时再次出现，应单独检查 Nav2 代价地图边界及 `map→odom→base_link` TF。

本次建图验证未启动串口底盘控制；上述结果只证明感知、建图和 RViz 显示链路，不代表导航运动控制已验证。

高处薄面门限依据当前场景雷达相对地面约 `0.27 m` 的实测高度设置。低于门限的箱体或坡地仍可能需要进一步标定；让机器人自主行驶前，需在现场核对障碍边界与底盘实际通过能力。

## 2026-10-12 复查：启动慢、建图误启动导航、静态障碍显示消失

### 启动约 20 秒无地图

本次启动日志表明，旧程序从 launch 启动到 ROGMap 配置加载完成约需 **44 秒**；SLAM Toolbox 直到这之后收到首个有效虚拟扫描。这不是 RViz 单独的显示延迟。代码检查发现，配置加载阶段即使不使用最近邻查询，也无条件生成并按距离排序半径 `5 m`、分辨率 `0.05 m` 的三维邻居偏移表，数量约四百万。已改成最近邻 API 首次调用时才生成。ROGMap 建图桥接不使用该 API，因此不再为其等待。

最终重启的独立话题计时：从 launch 命令执行开始，`/terrain/grid` 第一帧和 `/rog_map/scan` 第一帧分别在 **3.84 秒**、**3.84 秒**到达；含障碍的 `/map` 第一帧在 **6.64 秒**到达。ROGMap 配置日志在启动约 **0.62 秒**出现。雷达和 FAST-LIO 初始化仍需要时间，不能承诺启动瞬间就有地图。

### 建图模式误启动导航

旧 launch 无条件包含 `bringup_rm_navigation.py`，所以即使 `mode:=mapping`，也加载了 `nav2_container`、控制器、规划器、行为树和代价地图；它们又在地图产生前反复等待 `map` TF。已将 Nav2 include 限定为 `mode:=nav`，建图模式只独立启动 RViz。SLAM Toolbox 的 **mapping 节点本身负责实时位姿估计并发布 `map→odom`**，FAST-LIO 发布 `odom` 链路；建图时不应再运行一份 AMCL 或 SLAM 定位节点。最终进程检查只有雷达、LIO、ROGMap、`async_slam_toolbox_node` 和 RViz，没有 Nav2 控制器/规划器。`map→base_link` TF 已在现场查到连续输出。

### 车离开后静态障碍似乎从 `/map` 消失

先直接比对 `/map` 数据，而非只看 RViz 叠图：旧程序运行时，机器人移动约 `0.5 m` 的 22 秒间，第一帧地图中 **2,989** 个占据格有 **2,965** 个在末帧的相同全局坐标仍为占据（另有 64 个新增，24 个改变）。这不支持“SLAM 整体忘记了先前障碍”的判断。RViz 专用配置却把会随机器人滚动的局部 `ROGMap Terrain` 以 `0.65` 透明度画在全局图上方，并把全局图透明度设为 `0.6`；局部障碍随窗口离开时，画面很容易看成全局障碍也消失。

已将 RViz 固定坐标改为 `map`，把全局图作为前景不透明显示，并将局部 Terrain 设为背景、透明度 `0.35`。显示项改名为 `SLAM Global Map (persistent)` 和 `ROGMap Terrain (local)`，移除建图界面的 Navigation 2 面板和导航 Goal 工具。最终配置解析及运行进程检查通过；再次启动时全局 `/map` 随扫描继续增长，例如抽样为 `231×191`、1,728 个障碍格，局部图为 `201×201`、886 个障碍格。

这项定量验证覆盖了约 `0.5 m` 的现场移动。更新后的 RViz 启动后，用户现场确认：驶离此前扫描过的墙或箱体，`SLAM Global Map (persistent)` 上的黑色障碍轮廓**仍在**。另取 22 秒的全局图样本，原有 2,204 个占据格中有 2,197 个仍在相同全局坐标，另有 8 个新增；该段车辆基本静止。尚未对某一指定障碍做“驶出完整 10 m 局部窗口”后的逐格追踪，因此仍以现场全局图为准；目前没有证据支持修改 SLAM 扫描清空规则。

### 本轮构建与状态

- `colcon build --symlink-install --packages-up-to sentry_terrain_bridge rm_nav_bringup rm_navigation --parallel-workers 2` 完成；`colcon test-result --all --verbose` 为 **12 项、0 失败**，launch Python 语法和配置静态检查通过。
- 重启前已备份原有地图：`src/rm_nav_bringup/map/backups/my_map9_before_20261012_fix.posegraph`、`.data`、`.pgm`、`.yaml`。首次优化后的会话也另存 `.posegraph` 和 `.data`，可用于恢复位姿图。
- 当时建图组合已验证；后续进程状态以现场实际启动情况为准。RViz 仍可能记录 `indexed_8bit_image` GLSL 链接错误；当时 ROS 话题和地图数据正常，该图形驱动问题未修改。

## 2026-10-12 架构复查：ROGMap、SLAM 全局图与可穿过洞口

### 北理原工程怎样接感知

对照 [navi_minco_bit 总览](https://github.com/Walker152/navi_minco_bit/blob/master/README.md)、[ROGMap 模块说明](https://github.com/Walker152/navi_minco_bit/blob/master/src/perception/rog_map/README.md)、[原始地形分类](https://github.com/Walker152/navi_minco_bit/blob/master/src/perception/rog_map/src/rog_map/projection_layer.cpp) 与 [车上配置](https://github.com/Walker152/navi_minco_bit/blob/master/src/navigation/navi2_bringup/params/sentry1.yaml)：其主链路是双雷达稠密去畸变点云 `/cloud_registered_full` → 滚动三维概率 ROGMap → 单柱地形分类和 ESDF → MINCO 规划器内存查询。已有二维 Nav2 静态图作为**先验输入**，其中占据格只会保守地加入 ROGMap；Nav2 自身使用静态图和 STVL。在检查的原工程链路中没有发现把 ROGMap 的局部投影反算成 LaserScan 再交给 SLAM Toolbox 保存为全局图。原 README 也明示：当前分类仅做单柱统计，尚无坡面分割或通用地面模型，赛场隧道阈值与车体尺寸、外参有关。

本工程先前额外接了 `/terrain/grid` → `/rog_map/scan` → SLAM Toolbox `/map`。这使**滚动且会衰减的局部二维判断**成为**持久全局建图观测**：投影的 `PASSABLE/UNKNOWN`、高度分布和障碍时间性质在反算成单条射线时都丢失；静止时将 SLAM 位移门限设为零，又让多帧高度相关的局部判断被重复送入建图。两张图本就不应尺寸相同、逐格重合或有相同的障碍语义，不能靠叠图透明度解决这种结构性差异。

已将建图入口恢复为实测 `/scan`：`linefit_ground_segmentation_ros` 和 `pointcloud_to_laserscan` 在建图模式保持运行；SLAM Toolbox 始终订阅 `/scan`，使用原 mapper 位移/转角门限。ROGMap 仍独立发布滚动 `/terrain/grid`，供 RViz 与 Nav2 地形层消费；launch 强制关闭 `/rog_map/scan` 的生成。RViz 的黄色扫描显示改为 `SLAM Input Scan` → `/scan`。这是**改进建图语义隔离**，不是宣称原来的 `/scan` 一定能识别所有墙或洞口；现场需分别观察两条链路。

局部地形层叠加 Nav2 `static_layer` 时采取“障碍取较高代价”的保守规则：ROGMap 判断门洞可通行也**不能清除静态地图中已有的黑色墙格**。如果 `/map` 在门洞处为 occupied，须先检查实测 `/scan` 是否把门洞扫描成墙、SLAM 地图是否需修正，再判断 ROGMap 分类；仅调整 ROGMap 隧道参数无法让全局规划穿过静态黑墙。地形层现在按栅格消息时间查询 `map←odom`，避免 SLAM 回环修正时用最新 TF 投影旧地形快照导致局部错位。

### 可穿过洞口被判障碍：已定位的风险与尚缺的证据

北理原分类规则：XY 柱内占据高度跨度不超过 `surface_height_delta_max` 就归入薄面可通行；跨度与占据率符合 `tunnel_height_delta_*`、`tunnel_occupancy_ratio_max` 才归入隧道可通行。它并非门洞实例分割。为了补救本车稀疏墙/箱体回波，当前适配版新增 `elevated_surface_as_obstacle: true`，将雷达上方一定高度的**单层薄回波**改判为障碍。这也可能把门洞顶部、横梁或隧道顶的薄回波判成障碍；这是代码级冲突，不是北理参数配置错误。

目前没有包含该洞口的同步原始点云、里程计和每个栅格的分类原因记录，也没有本车实际净高/净宽的可靠标定，因此不能安全地直接把所有“高处薄回波”改回可通行：那会重现墙和箱体漏检。可靠的下一步应对洞口所在 XY 柱检查：地面到车体顶部的三维占据、顶部以下是否有真实的 free-ray 证据、两侧实体墙的连续性、洞口实际净空，以及 `/map` 的静态占据状态；再实现按本车轮廓判断的净空规则，并回归测试箱体、稀疏墙、门洞三种录制场景。单纯沿用北理的 `0.26–0.60 m` 隧道跨度不是本车可穿过性的证明。

另外仍需关注：本车用单雷达 FAST-LIO `/cloud_registered_body`（消息标签为 `livox_frame`，源码实际坐标为 IMU）配对 `/Odometry` 再变换到 `odom`，与北理的双雷达稠密 world 点云不同；切到旧 Point-LIO 时输入为稀疏 `/cloud_registered`，细障碍能力未验证；固定 `odom` 绝对 Z 截取在坡道/俯仰变化时可能漏扫或误扫；当前 `field`、`esdf` 和先验融合关闭，不能等同北理的三维安全检查。`/terrain/grid` 过期或 `map←odom` 无法按时间查询时，代价地图会标记非实时，需看 Nav2 costmap 的 current 状态。

本次完成 `rm_nav_bringup`、`rm_navigation`、`sentry_terrain_bridge` 构建；`rog_map` 与 `sentry_terrain_bridge` 共 12 项测试、0 失败；launch Python 语法检查通过。检查时未发现运行中的雷达/LIO/SLAM/ROGMap 进程，故本次没有对洞口或改造后的 `/scan` 建图链路作现场数据验证。现存按旧 `/rog_map/scan` 建出的 `.posegraph`/`.data`/`.pgm` 仍保留旧观测，代码修改不会自动清除其中的假障碍；验证应从新地图会话开始，旧地图按需保留备份。

### 下次现场验证

```bash
ros2 param get /slam_toolbox scan_topic   # 应是 /scan
ros2 topic hz /scan
ros2 topic hz /terrain/grid
ros2 topic echo --once /map --field info
```

RViz 中将 `SLAM Global Map (persistent)` 与 `ROGMap Terrain (local)` 分别单独显示，观察**同一门洞位置**：若 `/map` 为黑墙而局部图为白/未知，先查 `/scan` 和 SLAM；若 `/map` 为开口而局部图为黑障碍，再查 ROGMap 的三维分类；若两者都为黑墙，分别检查原始点云、地面分割和分类。此前文档中的 `/rog_map/scan` 频率、订阅与 SLAM 零位移门限属于旧方案的历史验证结果，**不再是当前版本的预期状态**。
