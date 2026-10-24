# 2027 ROGMap 感知迁移到 sen_xin 改造记录

## User Intent

将 `/home/zst/2027-sentry-navi` 的感知/地形分析接到 `/home/zst/sen_xin`，注意功能包变化，其他模块不变。目标不是 sen_xin11。用户授权本次修改；没有授权编译或启动实车。

## Scope

引入 ROGMap 库、独立感知适配节点、Nav2 地形层，接入现有实车 launch，提供 legacy/shadow/rog_map 开关。保留原定位用 `/scan` 链路。

## Out of Scope

不替换 Livox 硬件驱动、雷达 IP、IMU 标定、FAST-LIO/Point-LIO 状态估计器、TF 发布、地图资源、SLAM/AMCL/ICP、TEB/Navfn、串口/decision。未修改仿真启动文件或 sen_xin11；不处理来源项目的 MINCO/MPC、实验性聚类和双雷达标定。不调整既有速度和膨胀参数。

## Explorer Findings

### Files inspected

- 来源 `src/perception/rog_map/`：Config、ROGMapROS、ROGMap、ProjectionLayer、QueryAdapter、MapSnapshot、CMake/package。
- 来源 `src/navigation/navi2_bringup/params/sentry1.yaml` 的 MincoPlanner.rog_map。
- 来源 Point-LIO 的密集点云实现和功能包依赖。
- 目标 `bringup_real.launch.py`、`nav2_params_real.yaml`、两种 LIO 的 reality YAML。
- 目标 FAST_LIO 与 point_lio 的 publish_frame_world/body、publish_odometry；Point-LIO 的 IMU_Processing.cpp。
- 目标 URDF、nav.sh、ICP 点云订阅和本机 Humble Nav2 Layer 接口。

### Active logic path

目标 nav.sh 当前选择 fastlio+icp，控制器为 TEB，规划器为 Navfn。来源 ROGMap 原本在 MINCO 插件内部创建，没有可直接启动的独立可执行程序。来源的双雷达驱动、camera_init、/aft_mapped_to_init 不能直接覆盖本车配置。

### Data flow

FAST-LIO：原 `/cloud_registered_body` + `/Odometry` → 显式 IMU-body 配准 → ROGMap → `/terrain/grid`（odom）→ TerrainLayer（变换至地图系）→ 原 Nav2。

Point-LIO：原 `/cloud_registered`（已配准、稀疏）+ `/Odometry` → ROGMap；禁止直接把该版本未逐点去畸变的 body 云做整帧变换。

### Risk notes

- 两种旧 LIO 的 body 消息写 livox_frame，但函数实际已把数值转换到 IMU 系。适配器通过 cloud_coordinates 明确语义，不靠改 frame_id 假装做变换。
- 旧 Point-LIO world 云经过约 0.5 m 降采样；细障碍识别未验收。
- 来源分类参数不等于本车实际跨越能力；薄物体、低矮障碍、悬崖、斜坡必须验证。
- 高度裁剪是 odom 绝对 Z；本次垂直窗口设为 3 m，不能保证适配所有地形。
- ROGMap 头文件有 LGPL 版权说明，package.xml 标 BSD；本次原样保留所有声明，未擅自重写许可。
- 本机未获编译授权，无法证明 C++ 链接、插件加载或 NUC 性能已通过。

### Recommended modification boundary

只增加感知适配，不迁移 MINCO/MPC，不用新静态 TF 填补已有 TF 问题。

## Modifier Changes

### Files changed

新增：

- `src/rm_perception/rog_map/`：原样复制来源库（文件级 diff 一致）。
- `src/rm_perception/sentry_terrain_bridge/package.xml`
- `src/rm_perception/sentry_terrain_bridge/CMakeLists.txt`
- `src/rm_perception/sentry_terrain_bridge/terrain_plugins.xml`
- `src/rm_perception/sentry_terrain_bridge/src/terrain_node.cpp`
- `src/rm_perception/sentry_terrain_bridge/src/terrain_layer.cpp`
- `src/rm_perception/sentry_terrain_bridge/include/sentry_terrain_bridge/grid_contract.hpp`
- `src/rm_perception/sentry_terrain_bridge/test/test_grid_contract.cpp`
- `src/rm_perception/sentry_terrain_bridge/test/check_configuration.py`
- `src/rm_perception/sentry_terrain_bridge/README.md`
- `src/rm_nav_bringup/config/reality/rog_map_real.yaml`
- `src/rm_nav_bringup/config/reality/nav2_params_rog_map_real.yaml`
- 本记录。

仅覆盖两个已有文件：

- `src/rm_nav_bringup/launch/bringup_real.launch.py`
- `src/rm_nav_bringup/package.xml`

原文件备份：`/home/zst/sen_xin/.perception-backup-20261005-WyVjcb/`。没有删除已有文件。

### Key changes

- 独立调用 ROGMap updateMap，使用不可变查询快照发布标准 OccupancyGrid；未知/障碍分别转换为 -1/100。
- 时间/坐标校验、最新帧缓冲、最近时间戳里程计匹配；不重写输入时间戳掩盖延迟。
- 外参从本车 LIO 配置读取；FAST-LIO 利用原有密集 body 输出，不改变原 world 云密度。Point-LIO 使用已配准的 world 云并提示密度局限。
- 新代价层支持 odom→map 平面变换、旧区域重绘清除、保守重采样、最大代价叠加和超时非实时标记。
- 局部 costmap 由 terrain_layer 代替 obstacle_layer；全局保留 static_layer，插入 terrain_layer；两者保留原 inflation_layer。

### Behavior preserved

原定位、串口、导航包、URDF 的 311 个文件哈希完全一致。8 个既有 launch 动作的 AST 一致（定位、建图、导航包含、robot_state_publisher、Livox、IMU filter、linefit、pointcloud_to_laserscan）。原 Nav2 YAML 未改。新 YAML 回退感知层两个局部变更后，与原 YAML 数据完全相等。

### Behavior intentionally adjusted

- 实车 launch 新默认 `perception:=rog_map`。`legacy` 切回原感知，`shadow` 新算法只旁路输出。
- 修正原 lio 默认拼写 fast_lio→fastlio，与现有条件一致；仅 fastlio/pointlio 两个合法选项。
- 修正 FAST-LIO use_sim_time 参数键为字符串。
- 新模式确保 FAST-LIO 已有 body 发布开关打开；legacy 尊重原 YAML 值。
- 新地图尺寸 10×10×3 m；不继承来源双雷达地图中心偏移和赛场剔除框。
- field/ESDF 默认关闭，因为原 TEB/Navfn 不消费它；保留 Nav2 膨胀层。

### Notes

默认改变在重新构建/加载新启动文件后生效。第一次不能直接用会启动串口的 nav.sh 做自动运动；先禁用底盘输出并单独使用 shadow。新包尚未构建，现有 install 不代表本次改动已经可执行。两份 Nav2 配置今后需要同步公共调参。

## Auditor Review

### Checks performed

- [x] 关键路径检查：实际 LIO 点云数值坐标、时间戳、源库 API、Nav2 插件接口。
- [x] diff 检查：复制的 rog_map 与来源完全一致；原算法/串口/导航/URDF 文件哈希不变。
- [x] grep 检查：话题与包名匹配，无新增 TF 或速度发布者。
- [x] XML / launch / YAML 检查：Python AST、YAML 解析、26 个包无重名、插件 XML 一致。
- [x] 只读检查脚本通过，Nav2 非感知参数深度比较一致。
- [x] 真实 ROS 2 Humble Python launch 对象生成成功，未执行动作；6 种 LIO×感知组合的节点数量、参数文件选择、参数类型通过。
- [x] cppcheck（warning/performance/portability、C++17）新适配源码及测试通过；不等同编译。
- [x] git diff --no-index --check 启动文件通过。
- [ ] 构建许可：尚未取得，未执行构建，因为 AGENTS.md 禁止未授权构建。
- [ ] C++ gtest：已提供但未编译执行。
- [ ] ROS 运行时消息/TF、插件加载、NUC 性能和实车低速验收：未进行。

### Issues found

只读 launch 检查第一次因尝试创建默认 ROS 日志目录被沙箱拒绝；已将诊断日志定向 `/tmp` 后通过，没有修改用户 ROS 环境文件。Point-LIO body 未去畸变这一问题在审阅阶段发现后，已改为 world 云入口。未通过提高滤波参数或修改 TF 规避这些问题。

### Final result

**NEEDS_FIX（待构建/运行验收）**。源码接入与静态检查完成，不能声明已经通过编译或可直接上车。下一步需用户明确允许构建 `rog_map`、`sentry_terrain_bridge`、`rm_nav_bringup`，再检查依赖/链接/插件加载，最后在隔离底盘输出条件下验证感知数据和时延。
