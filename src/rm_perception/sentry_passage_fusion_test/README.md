# 2026-10-20：正式洞口观测已取消净空证明前提

`passage_runtime.py memory` 现在订阅 `/terrain/passage/structure`（PointCloud2），
由 terrain_node 从 ROGMap 的 `HOLLOW_TUNNEL` 且最终 PASSABLE 的列直接生成。
每个点包含 x、y、z（占据跨度下端）、ceiling_z（上端）和 resolution。
普通空列、薄面、实墙和后处理仍判障碍的列不属于此观测；数据不包含车体净空或支撑认证。

正式模式不再订阅 `/terrain/clearance/verified_segment`，也不发布 fresh_permit。
bringup_real 的 passage_runtime 不再自动开启 ClearanceObserver；passage_observer
和 passage_virtual_preview 仍可显式开启诊断，诊断不会发布正式净空证明。

记忆复用原来的地图会话检查、栅格化、落盘和静态修正 mask 发布路径，新增独立结构记录；
含结构的磁盘记录为 version 3，旧 version 2 短段仍兼容读取及地图交接。
正式结构模式已改用观测时刻的完整三维 TF：转换列下边界的角点，再按 XY 覆盖区域栅格化。
地图关联检查比较三维平移与完整旋转，旧平面变换记录仍兼容；旧净空诊断保持平面约束。
入口 gate、命令输出、代价地图层序均未在这两步改造；
旧 gate 尚不能把结构记录当成已完成净空认证的通道，因此这一步不代表已可驱动车辆穿洞。
以下历史章节中的“正式模式必须净空通过”描述已被本段取代，隔离测试仍保留旧接口。


# 阶段 1：隔离静态清障融合实验

## 老车雷达的虚拟穿洞效果（只看图，不控制车）

新增 `live_virtual_preview.launch.py`：接收实车 ROGMap 的**虚拟车**短段净空证据，经过地图关联与会话内通道记忆，送到隔离的真实 Nav2 `StaticLayer → StaticCorrectionLayer → TerrainLayer → InflationLayer`。RViz 对比原始 `/map` 和隔离 `/passage_fusion_test/costmap`，可直观看到黑墙是否按已观测短段打开、实时障碍是否重新封闭。虚拟证据使用独立 topic 和 schema，不写磁盘、不发布入口许可；此启动文件不启动规划器、控制器、串口或底盘速度节点，也不改变实车默认 Nav2 配置。

在 NUC 原工作空间重新构建 `sentry_terrain_bridge`、`sentry_passage_fusion_test` 和 `rm_nav_bringup` 并 `source install/setup.bash`。先用原有建图启动命令加 `perception:=rog_map passage_virtual_preview:=true nav_rviz:=false`，确认 `/map`、`map → odom`、`/terrain/grid` 和 `/terrain/clearance/status` 都正常。在另一个终端启动：

```bash
ros2 launch sentry_passage_fusion_test live_virtual_preview.launch.py rviz:=true use_sim_time:=false
```

然后照 `sentry_terrain_bridge/README.md` 的探针示例，向**洞口前方一个短段中心**发送 `/terrain/clearance/probe`。在新 RViz 中默认看到最终代价地图；切换显示项可看原始 SLAM 墙和虚拟证据。`/passage/memory_shadow_status` 的 `virtual_preview=true`、`state=KNOWN_PASSAGE`、`costmap_override=true` 表示某些静态占据格被预览掩码选中；最终代价图中的缺口可能因 InflationLayer 呈灰色而非纯白，原始代价 `<254` 才表示该格不再是致命障碍。缺口只覆盖已经通过三维验证的短段，未知体素或缺地面支撑时保持黑墙。不能用这张图给老车下穿洞导航目标。若只看到黑墙，优先读 `/terrain/clearance/status` 的失败原因，而不是调低安全阈值。

该预览不代表真实整车可以通过；本轮隔离接线测试验证了虚拟证据能影响测试 Costmap、实时障碍能否决且不会发入口许可。NUC 现场点云和实际洞口仍需你运行后观察。

## 2026-10-16 控制器到入口门控的隔离联测

新增 `isolated_controller.yaml`、`controller_command_adapter.py` 和 `test_controller_gate_ros.py`。测试启动 ROS 2 Humble 的真实 Nav2 ControllerServer，向其 `FollowPath` action 发送路径；控制器的原始速度只进入测试命名空间内的适配器，再经现有入口门控进入带 0.2 秒超时停车的运动学测试车辆。测试检查三个速度话题各只有预期发布者，并确认隔离域中没有 `/cmd_vel`。

一次完整联测已通过：车辆先以控制器输出接近入口，无新许可时停在入口前；人工注入当前短许可后恢复前进；许可过期和实时障碍都使它停车；导航取消后停车；车辆运动中门控进程退出，由车辆端超时机制停车；门控重启后重新核对当前输入；控制器进程退出后，适配器不续发旧命令，门控因命令超时停车。这些检查测试的是执行链路和失效行为，通道记录与当前许可由隔离测试构造，车辆为运动学模型。

复跑：构建 `sentry_passage_fusion_test` 后，使用 `ctest --test-dir /home/zst/zzs_rogMAP/.passage_lab/build/sentry_passage_fusion_test -R '^nav2_controller_gate$' --output-on-failure`。该测试固定使用独立 ROS domain 225 和本机通信。它尚未串接同一时段的真实 ROGMap 点云证据，也未验证 Gazebo 底盘动力学、NUC 通信或实车唯一控制权。下一项联测应让实际点云证据驱动同一 ControllerServer→入口门控→测试车辆链路，并保持没有当前全车净空证据时入口停车。

## 当前交付点：连续通道与入口门控融合

`passage_entry_manager.py` 已将横向相交、方向/纵向范围/车体尺寸相同的验证短段联合检查，解决真实 Nav2 路径偏离单条观测中心线就被误判为越界的问题。不会填补短段之间的未知缝隙，也不会自动合并不同方向、不同深度或不同地面高度的区域。

历史区域与当前许可分别处理：历史连续区域可供接近和规划使用；当前许可必须来自尚未过期的原始观测，并覆盖当前路径的完整车体重叠范围。联合许可只在本次路径检查中使用，不发布、不落盘。新记录携带地面高度和验证车高；旧记录缺少这些字段时，不能据此组合出新的三维通行许可。记录身份也绑定精确扫掠几何，避免两个不同位置恰好落在相同栅格中时错误续期。

验证边界：此前用 Gazebo 雷达点云经 ROGMap 生成的四条历史记录，已完成“记忆进程重启 → 真实 Nav2 生成路径 → 无新许可时入口停车 → 动态障碍否决”的独立回放。该回放不发布三维证据或新许可，不驱动车辆；它不是 NUC 实车结果。后续新增高度字段后的最新版仍需保留记录复验，不能以旧回放结果代替最新版实测。

原回放数据放在 `/tmp/rogmap-lateral-20261016-b`，续跑时已不存在，因此无法直接重复该次回放。不要重新合成人工记录来冒充原数据。下次采集用 `test_live_rogmap_fusion.py --lateral-diagnostic --artifacts-dir <工作空间内的持久目录> <fusion_harness路径>` 保留原始记录，再将该目录传给 `test_recorded_rogmap_planner.py`。持久目录必须是尚不存在的新目录。

最终源码的隔离回归已运行完成：13/13 通过，用时 56.89 秒，包含真实 Nav2 规划接口、记忆恢复、入口门控、证据超时恢复及实时障碍否决。它不替代上述缺失数据的真实 ROGMap 记录回放。

**下一项功能任务**：连接真实 Nav2 ControllerServer 输出与隔离入口门控，并验证测试车辆只能消费门控后的速度；覆盖正常跟踪、许可过期、障碍出现和上游停止时停车。先验证控制权与逐段停车，不再增加单独的 RViz 演示。不改实车串口、标定和默认 bringup，不把测试输出接到实车。

## 2026-10-15 逐段通道联测进度

隔离测试现已连接：ROGMap 短段证据、可重启的全局记忆、真实 Nav2 测试代价地图、候选探测器与**仅在测试命名空间输出**的入口门控。新增 `static_route_guard.py`：对计划路径逐格检查原始静态地图；前一段已验证不代表后一段已验证。遇到下一处占据或未知格且没有对应修正掩码时，门控先限速，再按制动余量停车。未知格即使掩码为 100 仍不能清除。

`test_memory_fusion_ros.py` 增加第二道墙：入口短段放行后，小车仅消费门控输出，来到第二道未验证的墙前停车；探测器提出第二段候选并收到新短段证据后，第二处代价才降低且门控才重新放行。`test_static_route_guard.py` 覆盖撤销、未知格和地图外格的拒绝逻辑。

这**不是实车通行许可**，也不是完整隧道的实测穿越：两段式运动联测中的第二段证据仍由测试脚本按真实 ROGMap 消息格式注入；下面的真实点云联测虽已分别证明两段感知，却尚未与同一次车辆运动闭环合并。隔离门控尚未接 Nav2 控制器的真实输出，更没有证明它是底盘的唯一速度入口。地图/掩码/状态现已通过内容指纹绑定，但仍是跨 ROS 话题异步传输，不能宣称实车控制链路已具备原子交付或唯一控制权；不得将测试命名空间重映射到实车 `/cmd_vel`。下一项验收是将多段真实 ROGMap 证据与执行端逐段停车/重新验证合并。

2026-10-15 补充：`test_live_rogmap_fusion.py` 已在独立 ROS domain 219 的无 GUI Gazebo 中通过**两个不同短段**的真实点云验证：x=1.7 入口段与 x=2.6 洞内段分别产生 ROGMap `verified_short_segment_observation`，各自写入磁盘记忆并在 Nav2 测试 master 中只打开对应的静态墙；动态障碍仍能否决。第二段在洞外视角仍有 86 个内部未知体素，移动诊断传感器架到洞内三个位置后降为 34→6→0，才产生证据；扫描结束后传感器架已恢复，Gazebo/测试进程均退出。此测试**只移动诊断传感器架，不驱动车辆**，因此还不能证明机器人能够通过整个隧道。

复验命令（需已构建 `sentry_passage_sim` 与 `sentry_passage_fusion_test`）：

```bash
source /opt/ros/humble/setup.bash
source /home/zst/zzs_rogMAP/.passage_lab/install/setup.bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 /home/zst/zzs_rogMAP/src/rm_perception/sentry_passage_fusion_test/scripts/test_live_rogmap_fusion.py /home/zst/zzs_rogMAP/.passage_lab/build/sentry_passage_fusion_test/fusion_harness
```

`grid_fingerprint.py` 对静态地图内容/几何/载入时间和修正掩码分别计算内容指纹；记忆状态携带两者，隔离入口门控只有在当前收到的两张图都匹配时才会输出非零速度。旧地图、新掩码、旧掩码、新地图或输入超时均为零输出。新增单项验证及完整隔离回归 **12/12 通过**。

### 结构记忆与当前入口许可的不同新鲜度（2026-10-15）

原先 ROGMap 结构验证允许体素观测年龄最长 60 秒。这适合保留历史通道，却不能直接授权当前进入。当前实现将“历史结构”与“下一步运动许可”分开：结构证明仍要检查完整车体扫掠体积和地面支撑均已知、无占据、未超结构观测有效期；短时许可只检查前进方向最前的一个体素切片（上下车体净空 + 对应地面支撑）是否完整且近期被直接观测，且证据最新时间不超过 0.5 秒、许可最多保留 1 秒。这样不会要求雷达一次性重新看到整个隧道内部，但每前进一步仍需要对新的前缘重新取证。缺失或旧格式前缘证据一律不发许可。

真实移动 Gazebo 雷达联测仍未发出入口许可。近入口时传感器确实观察到大部分历史净空，但前缘上下车体/地面覆盖不完整；例如远侧观察的前缘上部为 28/48、地面支撑为 4/12，中心近距离观察还出现 11 个支撑格未被近期射线刷新。即使历史几何能通过，许可也继续关闭，Nav2 车辆没有穿门。这不是把阈值改松就能解决的问题：多视角的有效范围和直接地面回波必须覆盖同一个即将占用的前缘切片。下一步补充更合适的受控观测位/车头姿态，或者确认当前安装的雷达视场确实无法覆盖时，保持停车并调整传感器布置/扫描动作。

进一步按前进方向四段、每段上下两层统计：x=1.7 入口段的下半部为 192/192 近期观测，上半部为 96/192；x=2.6 洞内段下半部仍为 192/192，上半部为 **0/192**。因此不能仅以“下部没障碍”准许高 0.70 m 的车通过，低横梁仍是危险。记忆状态新增 `last_entry_observation_reason`，将这种情况明确报为 `UPPER_BODY_NOT_RECENTLY_OBSERVED`；它只解释最近一次结构证据，不代表当前许可。体素分区统计只作诊断，不会因某个百分比自动开放通行。

本包现有三维净空验证、全局通道记忆、真实 Nav2 全局规划联测和测试命名空间的入口门控；这些能力尚未形成实车可用的端到端安全闭环。人工掩码不是实车通行证据。**禁止将测试插件或本测试配置加入实车 bringup。**

### 真实 Nav2 全局规划器联测（2026-10-15）

新增 `test_nav2_planner_history.py`：在独立 ROS domain 223 启动真实 Nav2 PlannerServer 及其 `StaticLayer → StaticCorrectionLayer → TerrainLayer → InflationLayer`，但**不启动控制器或实车底盘**。测试确认：原始静态墙无法规划穿越；单条仅约一车宽的历史通道在膨胀后仍不可规划；三个相邻已验证短段组成足够宽的历史通道后，Nav2 可以从远处规划经过；实时障碍会否决路径，障碍撤除后恢复；记忆进程停止导致证据超时后原墙恢复，重新读取磁盘记录后路径重新出现。历史结构记录的 `entry_allowed=false`，不直接构成准入。

规划器返回的**实际 Path** 现由隔离测试转发到入口门控：无新鲜许可时可以以保留制动距离的速度接近，但到入口前必须输出零速度；仅中央车体短段收到新的、未超时证据后，门控才短暂输出非零速度；实时障碍及许可过期都会恢复零输出。修复了密集 Nav2 路径被误判为擦边的入口计算，以及相邻已验证短段的外围重叠互相否决的问题。路径偏离任何已验证整车扫掠区域时仍拒绝，不按地图格宽放宽车体尺寸。

这证明了全局规划与实验门控的接线及失效恢复，不证明真实洞口的三维证据已足够宽，也不证明入口门控是实车速度的唯一出口。此联测的证据由测试脚本合成；下一步需把真实 ROGMap 的连续可观测短段接到同一条规划—门控链，并证明控制器输出只能经门控抵达底盘。不能用该测试结果直接放开自动穿越。

## 运行对象

加载真实插件：
1. Nav2 StaticLayer：订阅合成静态地图。
2. 本包 StaticCorrectionLayer：在有效人工掩码内临时清除静态占据。
3. 原 sentry_terrain_bridge::TerrainLayer：模拟实时障碍有最终否决权。
4. Nav2 InflationLayer：重算障碍周围代价。

阶段 1 的 `fusion_harness` 只使用 Nav2 LayeredCostmap，不启动规划器或控制器。新增 `test_nav2_planner_history.py` 则专门启动隔离的真实 Nav2 PlannerServer 验证规划，不启动控制器、串口或实车速度发布者。

## 安全隔离

- 独立命名空间 `/passage_fusion_test`，以及 PlannerServer 的内部 costmap 命名空间 `/passage_fusion_test/global_costmap`；插件在其他命名空间初始化会失败。
- 插件默认 enabled=false，必须显式 allow_test_evidence=true。
- 基础融合测试使用 ROS_DOMAIN_ID=218，PlannerServer 联测使用 223，均要求 ROS_LOCALHOST_ONLY=1；不要让实车使用这些测试域。
- 修正层只接受 map 坐标系、轴对齐、与当前静态地图完全一致尺寸和原点的证据，不做 TF 投影。
- 严格检查 StaticLayer → 修正层 → TerrainLayer → InflationLayer 顺序。
- 不修改现有 launch、ROGMap 参数、TerrainLayer、实车标定、串口和控制代码。

这些是防误接措施，不是访问控制或可直接实车使用的安全认证。

## 编译

在已经能够正常编译 rog_map 与 sentry_terrain_bridge 的工作空间中：

```bash
cd /home/zst/zzs_rogMAP
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to sentry_passage_fusion_test
source install/setup.bash
```

在 NUC 使用该机实际工作空间路径。不要复制其他电脑的 build/install 缓存。如果本机旧缓存指向 /home/doublegun，应采用新的 build/install 目录验证，不要清空整个工作空间。

## 自动化验收（以退出码为准）

```bash
colcon test --packages-select sentry_passage_fusion_test --event-handlers console_direct+
colcon test-result --verbose
```

自动测试输出逐项 PASS，最终输出 ALL PASS；失败时退出非零，日志保存在测试工作目录 fusion_harness_test.log。测试使用 /costmap_raw 的 0..255 数值断言，不通过截图猜测。

覆盖：静态墙、指定区域开洞、动态障碍及其膨胀否决、动态障碍移除、证据移动时恢复旧区域、显式撤销、发布停止后的超时、reset、disable/enable、错误 frame、错误原点、错误长度、错误数值、过期/未来/重复时间戳、错误地图版本、未知格保护、新地图撤销旧许可、错误图层顺序。恢复时比较完整 Costmap，包括膨胀代价，而非只看缺口中心。

## RViz 演示

```bash
ros2 launch sentry_passage_fusion_test fusion_test.launch.py rviz:=true
```

演示运行相同测试步骤并放慢显示；结束后关闭测试节点和 RViz。launch 退出码不代替自动测试结果，请检查 ALL PASS / FAIL，正式验收用 colcon test。

只检查 launch 链路、不打开图形界面：

```bash
ros2 launch sentry_passage_fusion_test fusion_test.launch.py rviz:=false pause:=0
```

显示：
- Original Static Map：原始 /map，墙始终存在，**默认关闭**。
- Safe Input Preview：独立 `/passage_fusion_test/evidence_preview`，默认关闭。只预览当前持续发送的人工掩码，不代表插件已批准通行；单次非法证据不转发到此话题。
- Final Costmap：默认采用不透明的 `map` 配色，0 显示白色、100 显示黑色，其他已知代价显示灰度。开洞变白，撤销后恢复黑墙。
- Stage and Raw Costs：显示正在检查的步骤和 A/B 中心格的真实代价（读取 `/costmap_raw`）：0=自由、254=障碍、255=未知。
- 若要比较原始墙，请先关闭 Final Costmap，再单独打开 Original Static Map。不要把两者叠起来判断开洞。
- 如需彩色膨胀带，可把 Final Costmap 的 Color Scheme 改为 `costmap`，但应保持原始地图关闭；该色表的自由空间是透明色。

故障注入 `/passage_fusion_test/test_evidence` 包含故意构造的负时间戳、非法纳秒、错误长度等数据，**禁止直接添加为 RViz 显示话题**。安全预览从正常地图几何和新时间戳重新构造，不把非法输入“修好”后当成真实通行证据。最终是否清障，以 Final Costmap 和原始代价值为准。

RViz 提前退出时 launch 会停止整个演示并提示未完成；只有看到完整 ALL PASS 才是用例执行完成。窗口关闭、Ctrl+C 或部分 PASS 都不是完整验收。

### 包含 RViz 的额外回归

常规 CTest 不依赖显示服务器。需要复现图形回归时，在有图形会话的终端运行以下命令（先停止其他测试）：

```bash
test_prefix=$(ros2 pkg prefix sentry_passage_fusion_test)
ros2 run sentry_passage_fusion_test fusion_scenario.py \
  --harness "$test_prefix/lib/sentry_passage_fusion_test/fusion_harness" \
  --config "$test_prefix/share/sentry_passage_fusion_test/config/fusion_test.yaml" \
  --rviz-config "$test_prefix/share/sentry_passage_fusion_test/rviz/fusion_test.rviz"
```

该回归在临时 RViz 配置中强制打开安全预览，保留所有非法输入测试，并检查 RViz 全程存活、订阅安全预览及数值标签；任何提前退出都会使测试失败。测试结束会关闭它启动的进程。当前目录生成 `fusion_rviz_test.log` 和 `fusion_harness_test.log`。它不检查屏幕像素，因此仍应人工核对显示布局。

## 人工证据接口（仅本测试协议）

话题 /passage_fusion_test/test_evidence，类型 nav_msgs/msg/OccupancyGrid，完整快照：
- header.frame_id 必须为 map。
- header.stamp 是本次证据时间，严格递增；不能重复、超时或来自未来。
- info 与静态地图几何一致。
- info.map_load_time 必须匹配当前测试地图版本。
- data=100：此格获得测试清障授权。
- data=0 或 -1：无清障授权，不继承上一帧许可。
- 全零快照代表撤销全部许可；停止发送后 evidence_timeout 自动撤销。

这里 100 是“测试授权掩码”，**不是障碍概率**，不能接到 StaticLayer、TerrainLayer 或真正的三维验证接口。第一版仅一个全量证据源，不支持多通道 ID、多来源融合或持久化。

静态地图 data=100 且当前 Master=254 才能被修正。未知格不因人工证据变为自由。真实 TerrainLayer 随后仍可将同格改回障碍。

## 恢复机制

只改当前 Master，不改 StaticLayer 缓存或 /map。证据到期/撤销后，旧影响范围仍进入 updateBounds；Nav2 重置该区域后由 StaticLayer 写回当前原始地图，再由地形和膨胀层计算。测试小地图使用完整范围重算，暂不做增量优化。

新地图版本、内容或几何变化会撤销旧证据；时间回退也会撤销并要求重新接收地图。map_load_time 在本实验中作为显式版本标记，**真实 SLAM 更新不保证符合此协议**，后续接入必须另行设计地图会话和回环关联，不能直接套用。

## 尚未完成

本包已覆盖短段三维证据、离开局部窗口后的结构记忆、真实 Nav2 全局规划联测与隔离入口门控，但**不证明 NUC 实测洞口已完整可观测**，也不限制真实车辆越过入口。测试门控未成为实车速度的唯一出口，因此不允许自动穿越。下一步需要把规划器实际路径与逐段入口门控合并，并核对相邻短段的联合净空与控制权。

## 本地验收记录

2026-10-13：ROS 2 Humble / Nav2 1.1.20，独立临时 build/install 构建成功。自动集成测试通过 45 个阶段检查点，以及 2 项启动防误接检查；测试结果为 1 个集成测试、0 失败。恢复原地图时比对完整代价数组，新增测试还确认原始地图内容更新、原点移动后不会复用旧许可。

以上是合成输入下的真实插件测试，不是 Gazebo 车辆穿洞或 NUC 实车验证。错误输入和错误图层顺序测试会有预期 WARN/ERROR，最终是否通过以测试报告为准。

### 2026-10-13 演示修复回归

- 默认只显示不透明的最终地图，避免原始黑墙透过自由格；新增阶段及 A/B 原始代价值标签。
- RViz 仅订阅独立构造的安全预览，不接收非法证据；所有负时间戳等故障注入仍保留。
- 带真实 RViz 的完整回归通过 45 个数值检查点、2 项启动隔离检查，验证 221 帧预览；强制打开预览与数值标签后，RViz 全程未崩溃并正常退出。
- 补充显示配置回归，防止默认叠图或重新接入原始非法消息。控制/TF 隔离断言检查实际发布者，允许 RViz 自身订阅 TF。
- 回归验证了进程存活、订阅连接、消息结构与代价值，未做截图像素比对；不等于实车安全验证。

### 2026-10-16 ROGMap / 磁盘记忆 / Nav2 联合验证

- 使用 Gazebo Livox 点云实际进入 ROGMap，再将 ROGMap 输出的短段净空证据送入记忆适配器和隔离 Nav2 StaticCorrectionLayer；两个分离位置均得到三维证据，Nav2 全局 Costmap 按这些已验证格子临时修正静态墙。实时 TerrainLayer 障碍重新出现时，规划代价恢复为障碍。此次运行结果：`fresh_entry_permits=9`，左右观察偏移 `±0.25 m` 的诊断扫描均有独立证据。洞内段有 `192/384` 车体体素近期直接观测；这只证明局部体素证据链工作，不证明真实车辆能安全穿越。
- 用本次实际 ROGMap 证据写出的 `memory.json`，在单独新进程中恢复为 4 条结构记录；`map→odom` 平移 0.30 m 时记忆暂停；静态地图受影响区域变化时对应记录失效。该试验使用实验地图会话标识，不代表生产 SLAM 已提供可靠会话 ID。
- 14 项现有 CTest 全部通过，包含真实 Nav2 PlannerServer / 控制器门控联测及记忆、超时、障碍否决等检查。
- 另做车载传感器位置的真实 Nav2 移动模型试验：机器人能在安全区域按 Nav2 路径移动，左右重叠的 ROGMap 证据被记为结构候选，局部代价也只按已验证范围变化；前缘许可改为逐切片验证后，真实点云仍未同时刷新前缘车体与地面支撑（观察位间会互补，但不能在同一条许可窗口内凑成完整新证据）。`entry_allowed=false`，门控保持停止。**没有越门，不是穿越成功。** 不按仿真通车目标放宽前缘要求。
- 上述车载试验脚本将两条探测线设为 `y=±0.20 m`，让证据范围重叠，并要求每次即时刷新必须收到新的 ROGMap 确认证据；本次仍因支撑体素过期而按预期拒绝。脚本不会把历史候选当即时许可。
- 2026-10-16 补充：发现并修正一个入口许可时序缺口。历史结构证据仍可保留，但短时入口许可现在同时要求扫掠车体体素与地面支撑体素均有新鲜直接观测；老格式记录缺少支撑观测时间时只可用于结构记忆，不可发许可。`/terrain/clearance/status` 现报告最老/最新地面观测年龄，移动联测失败报告会附带当前缺失支撑体素坐标，供判断是点云覆盖、传感器遮挡还是探针地面高度不匹配。该修正只会收紧门控，不放宽通过条件。
- 同日补充：短时许可现在基于前缘切片而非全段旧体素时间戳，并按体素切片输出 body/floor 近期覆盖数。该调整是为了支持真实的分阶段扫描，不意味着对未知体素放行；切片必须全覆盖且结构验证必须先通过。最新多视角联测因此仍保持 fail-closed。
- 同日进一步核实并撤销：虽然 SDF 碰撞盒理论顶面为 `z=0`，Gazebo 雷射回波和当前 ROGMap 射线更新在 `z=0` 查询层呈现 48 个最新自由体素，而不是地面命中；`z=-0.025` 才落在实际地面回波层。不能仅凭几何盒顶面推断传感器回波栅格，仿真探针保留经数据验证的 `-0.025`。实车探针高度必须由其实测地面回波/地面估计提供，不可照搬实验常量。

截至本记录，功能级测试链已覆盖真实 ROGMap 消息、磁盘恢复、真实 Nav2 规划/控制器门控和动态障碍否决；**实车/NUC 的传感器闭环、真实 SLAM 地图会话关联，以及底盘唯一速度出口尚未接通或验证**。不要将本包测试命名空间、`publish_test_mask` 或测试控制器配置接入实车 bringup；在实车上保持现有导航方案不变。
