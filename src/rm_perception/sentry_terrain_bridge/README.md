# sen_xin：2027 地形感知接入

本次只接入 `/home/zst/2027-sentry-navi/src/perception/rog_map` 的概率体素、射线更新、地形分类、滚动窗口和障碍衰减。原项目把 ROGMap 放在 MINCO 规划器内部，因此这里新增独立节点和 Nav2 插件，**没有复制 MINCO/MPC**。

## 包和功能的边界

|功能|本次处理|
|---|---|
|`rog_map`|原样复制来源库及其版权注释；不改算法实现|
|`sentry_terrain_bridge`|新增独立运行、消息校验、OccupancyGrid 转换及 Nav2 适配|
|FAST-LIO / Point-LIO|保留本车的源码、IMU 外参、估计器和 TF；只利用已有点云输出|
|Livox 驱动|保留本车驱动/IP，不复制来源双雷达配置|
|SLAM Toolbox / AMCL / ICP / map_server|保留；地图路径和选择规则不变|
|Navfn / TEB / 速度平滑 / 串口 / decision|保留源码和参数；不添加速度发布者|
|linefit / pointcloud_to_laserscan|导航定位及三种建图模式均继续生成原 `/scan`；ROGMap 不接管 SLAM 输入|
|`msg_convert` / DBSCAN / 双雷达标定|不复制未启用的实验流程和双雷达工具；ROGMap 自带裁剪功能暂关闭|

注意：这不是把旧感知相关目录全部删除。导航定位和建图都保留原 `/scan`；全局位姿图和 `/map` 仍由 SLAM Toolbox 负责。不要把 ROGMap 的局部 `/terrain/grid` 误认为可保存的全局图。

## 数据链路与坐标

```text
原 Livox → 原 FAST-LIO → /cloud_registered_body（密集、IMU-body 数值）
                     → /Odometry（odom 中的 IMU 状态）
                                  ↓ 配准与时间校验
                              独立 ROGMap
                                  ↓
                         /terrain/grid（odom）
                                  ↓ 当前 map←odom TF
                        Nav2 TerrainLayer → 原 Navfn / TEB
```

目前实车建图仍使用原扫描链路：

```text
原点云 → linefit → /segmentation/obstacle
                   → pointcloud_to_laserscan → /scan
                   → SLAM Toolbox → /map、位姿图、map→odom
```

ROGMap 的局部地形与三维净空结果供旁路分析和后续导航融合使用；当前 launch 强制 `mapping_scan_enabled: false`，不要期待 `/rog_map/scan` 存在。正式地图仍需核验 `/scan`、TF、入口两侧墙体与中间间隙。

经核对本仓库 `publish_frame_body()`：FAST-LIO 的 body 点云已经应用雷达到 IMU 的外参，**数值在 IMU 系，但原消息 header 写的是 `livox_frame`**。适配器显式区分 `cloud_coordinates: imu`，用 `/Odometry` 的 IMU 位姿变换一次，不再乘雷达外参，也不通过这个标签再查一次 TF。雷达射线起点单独使用 `p_imu + R_imu * extrinsic_T`。外参由 launch 从所选 LIO 配置读取。

旧 Point-LIO 的 body 输出未做逐点运动补偿，所以该分支改用 `/cloud_registered`（已经逐点配准到 odom），`cloud_coordinates: odom`，不会再变换点坐标。但旧算法输出经过约 0.5 m 降采样，**细小障碍可能丢失**；该分支仅完成接口适配，必须先旁路验证。不要为获得密集点云直接把未经补偿的 body 输出整体变换。若需要来源项目 Point-LIO 的密集逐点输出，应另行移植那部分输出机制并测试。

本次不发布 `map→odom`、`odom→base_link` 或任何静态 TF。ROGMap 的先验地图融合关闭，地图继续归原定位模块管理。没有 `map→odom` 时新代价层会报警并保持非实时，不会用假静态 TF 掩盖错误。

## 三种运行方式

以下示例需要在运行设备上先编译并 source 新安装目录。本次只完成了当前机器上的隔离构建和单元测试，没有运行实车 launch 或启动串口。使用你自己的地图名称，示例中的 `my_map11` 不代表地图已经核验。

```bash
cd ~/zzs_rogMAP  # NUC 上按实际工作区路径替换。
source install/setup.bash

# 建议第一次先旁路比较：原避障工作，新算法仅发布地形供观察。
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=nav world:=my_map11 lio:=fastlio localization:=icp perception:=shadow

# 核验合格后，新地形接入局部/全局代价地图。
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=nav world:=my_map11 lio:=fastlio localization:=icp perception:=rog_map

# 切回原实时避障和原 Nav2 配置，不需要删除新功能包。
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=nav world:=my_map11 lio:=fastlio localization:=icp perception:=legacy
```

`bringup_real.launch.py` 默认 `perception:=rog_map`；现有 `nav.sh` 未改动，重编译后仍调用该 launch 时会采用新默认值。**第一次不要直接用会启动串口的 nav.sh 上车运动**；先单独启动 `perception:=shadow`，保持底盘驱动禁止输出，并准备急停。

仿真 launch、本车地图保存脚本和 sen_xin11 均未改。此接入暂仅覆盖实车启动文件，不能声称 Gazebo 已验证。

## 如何检查

建图试跑（底盘先禁止自主运动）：

```bash
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=mapping lio:=fastlio perception:=rog_map
ros2 topic hz /scan
ros2 param get /slam_toolbox scan_topic
```

最后一条应返回 `/scan`。RViz 分别显示 `/terrain/grid`、`/scan`、`/map`，重点观察入口两侧墙点及中间空隙。若扫描漏墙或 TF 不通，不要在未确认时直接保存并用于实车导航。

### 实车只读净空观察（默认关闭）

在现有启动命令末尾加 `passage_observer:=true`，例如 `mode:=mapping perception:=shadow lio:=fastlio passage_observer:=true`。这让 ROGMap 记录直接观测时间并发布 `/terrain/clearance/status` 和 `/terrain/clearance/markers`。当前实车启动仍针对高约 0.70 m、机械上不能过洞的老车；此观察仅用于感知诊断，**强制不发布已验证通道证据、不修正静态地图、不发通行许可或速度**，绝不让老车试穿。新车目标高度 0.32 m 记录在 `vehicle_geometry_new_pending.yaml`，雷达和 `base_link` 未标定前不能用其进行实车净空放行。先禁止自主运动，在洞口外静止采样。

若现在只想用老车雷达**预览一台高 0.32 m 的虚拟车**能否被 ROGMap 识别为可穿洞，启动时改用 `passage_virtual_preview:=true`（`passage_observer` 可省略）。打开 `rog_map_debug.rviz` 后看 `Virtual 0.32m Tunnel Probe (visual only)`，再按下文向 `/terrain/clearance/probe` 发送探针。绿色仅表示该虚拟尺寸的短段净空证据达到确认次数；红色、青色、品红色分别帮助查看未通过、内部未知和地面支撑缺失。`/terrain/clearance/status` 会以 `VIRTUAL_BODY_PREVIEW_ONLY` 开头并显示所用车高。这个预览**不会改变 `/map` 或 Nav2 Costmap，也不发布 `/terrain/clearance/verified_segment`，更不会让老车获得通行许可**。雷达仍装在老车上，视角不同于将来的新车，结果只能用来检查算法链路，不能据此推断新车最终可过洞。

这里说的“不改变 Nav2 Costmap”指**实车正式导航的 Costmap**。现在可另启 `sentry_passage_fusion_test/live_virtual_preview.launch.py`，在隔离 Costmap 中显示虚拟证据的开洞效果；该独立视图仍不发通行许可或速度。

对确定的短段中心发一个探针；`x/y` 是 `odom` 坐标，`z` 必须填写实测地面在 `odom` 中的高度，不能用地面分割的 `sensor_height` 或 `base_link` 的 z 值代替。朝向零度表示沿 `odom` 的 +X 方向。下面的 `GROUND_Z` 要先由现场测量后填入：

```bash
read -r -p '请输入实测地面在 odom 中的 z（米）: ' GROUND_Z
ros2 topic pub --once /terrain/clearance/probe geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: ${GROUND_Z}}, orientation: {w: 1.0}}}"
ros2 topic echo /terrain/clearance/status
```

记录 `reason`、`body_recent_by_slice`、`support_recent_by_slice`、`entry_edge_recent`，再从入口左右安全位置重复观察同一探针。`confirmed=3/3` 只是短段结构证据，**不是准许进洞**。观察不全、离开局部地图、TF 变化或地面高度选错都需要分别分析；不要通过放低验证门限让实车冒险通行。若启动后该话题不存在，先确认 `ros2 param get /sentry_terrain clearance.enabled` 为 `True`，以及使用的是重新编译并 source 后的工作空间。

在 NUC 上检查各环节是否有新数据，可在已启动上述观察模式的另一个终端运行（按实际工作空间路径 source）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sentry_terrain_bridge real_passage_diagnostics --lio fastlio --seconds 8
```

它只订阅原始雷达点云、地面分割结果、LIO 点云、里程计、`/scan`、`/terrain/grid`、`/map`、净空状态并查询 TF；打印 `OK`、`MISSING` 或 `STALE`，不发速度也不修改地图。若实际运行 Point-LIO，改用 `--lio pointlio`。请保留这段完整输出，特别是 `ground_z_median_in_cloud_frame`、`raw_z_lower_decile_in_cloud_frame` 和最后三条 TF。前两个 z 统计只用 0.7–4 m 范围内采样点作粗查；地面分割输出本身受现有 `sensor_height` 影响，不能单凭其结果反证参数正确。`measurement_params_real.yaml` 中的 `z=0.622` 是未被 xacro 使用的旧记录；URDF 固定写着雷达相对 `base_link` 的 `z=0.25 m`。用户描述老车 `base_link` 离地约 `0.23 m`、雷达再高约 `0.22 m`，两者相加约 `0.45 m`。地面分割源码中的 `sensor_height` 表示**雷达离地高度**，当前配置却是 `0.22 m`；这与上述尺寸不符，必须量取并核对原始点云地面 z 后才能改。新车的安装位置与 `base_link` 尚未确定，不能沿用老车的 TF、分割高度或 LIO 标定。

需要离线复盘时，在 NUC 上另外开一个终端，短时记录下面这些原始输入和验证结果；从洞口正前方、左侧、右侧各观察一段后按 Ctrl+C 正常结束录制。点云占用磁盘较快，先检查剩余空间。该记录不控制底盘。

```bash
ros2 bag record -o passage_probe_run \
  /cloud_registered_body /cloud_registered /Odometry \
  /terrain/clearance/probe /terrain/clearance/status \
  /terrain/grid /scan /tf /tf_static
```

FAST-LIO 主要回放 `/cloud_registered_body`，旧 Point-LIO 分支回放 `/cloud_registered`；如果某个话题在当前 LIO 模式下不存在，录包工具会等待它，不影响其他已有话题。将录包、当次 `rog_map_real.yaml`、LIO 配置和地面 Z 测量值一同保存，才能区分雷达遮挡、时间同步、坐标和参数问题。

RViz 的 Fixed Frame 使用现有的 `map`（TF 正常时）或用 `odom` 单独看感知。添加 Map 显示，Topic 选 `/terrain/grid`，Durability 用 Volatile。不要把这个话题替换成定位用的 `/map`。

```bash
ros2 topic hz /terrain/grid
ros2 topic echo /terrain/grid --once --field header
ros2 topic info /cloud_registered_body -v
ros2 topic info /Odometry -v
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /global_costmap/global_costmap plugins
ros2 run tf2_ros tf2_echo map odom
```

实际节点名有命名空间时以 `ros2 node list` 为准。rog_map 模式局部插件列表应为 terrain_layer、inflation_layer；全局应为 static_layer、terrain_layer、inflation_layer。先比较静止的墙、桌脚、低矮箱体，再在断开电机输出的条件下人工移动/缓慢转动车，检查障碍是否跟随错位、出现延迟或消失。不做高速自主导航验收。

输出 OccupancyGrid 正确区分：未知=-1、自由=0、障碍=100；不能把原库的 254/255 直接塞入 int8 栅格。新插件按最大代价叠加，不用观测自由空间擦掉先验地图的墙。输入超过 `max_input_age` 不处理；投影/TF 超过 `data_timeout`，代价层标记 `current=false`。这不是硬件急停，不能代替底盘命令超时保护。

## 参数与已知限制

- 配置在 `src/rm_nav_bringup/config/reality/rog_map_real.yaml`，新 Nav2 参数在 `nav2_params_rog_map_real.yaml`。原 `nav2_params_real.yaml` 原样保留。之后如果调 TEB 等公共参数，需同步两个 Nav2 文件；可运行下面的静态检查识别意外分歧。
- 来源项目的地形阈值（例如 surface_height_delta_max=0.25）是**一列体素的高度差分类门限，不是允许本车跨过 25 cm 台阶的保证**。你的车身高度、离地间隙、地面 Z、细墙和斜坡识别必须重新测量。
- `scan_z_min_abs/max_abs` 是 odom 中绝对高度。初始地面高度或爬坡改变后，固定高度范围可能裁掉地形；不要直接把 sensor_height 当成这两个参数。
- 悬崖/负障碍、遮挡、薄平面物体可能漏检。该接入没有新增悬崖检测或功能安全保护。
- 来源地图特定剔除框、双雷达偏移、先验图融合全部关闭。距离场未供现有 TEB/Navfn 使用，默认关闭以节省 CPU；现有 inflation_layer 保持不变。
- 只缓存最新点云，里程计按时间戳找近邻（默认允许 0.1 s）；这不是重新做 LIO 的运动补偿，也不能修复 LIO 漂移。时间重置后应一起重启感知和相关定位节点。
- ROGMap 累积多帧再衰减，瞬时遮挡后的保留/消失需要实测；不是单帧点云分类器。
- 约 10×10×3 m、5 cm 分辨率会增加 NUC 负载；目前未测 CPU、内存、端到端时延。
- `/cloud_registered`、原 LIO 估计器与串口节点未修改。FAST-LIO 仅确保其已有 body 输出打开，不更改原 world 输出密度。

## 验证与构建边界

```bash
python3 src/rm_perception/sentry_terrain_bridge/test/check_configuration.py ~/zzs_rogMAP
```

这是只读静态检查。实车只读观察开关已在本地临时目录构建并通过净空验证单元测试，但还没有在 NUC 上测量真实雷达覆盖、TF、CPU 负载及入口观测；不能据此启用自动静态清障或实车穿洞。
