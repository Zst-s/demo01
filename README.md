# zzs_rogMAP

ROS 2 导航工作空间，包含 Point-LIO、ROGMap、地形分析、洞口结构记忆及 Nav2 静态层修正。此仓库是当前新车工程的源码快照。

## 获取与编译

SLAM 数据和文档动图使用 Git LFS，克隆前需安装 Git LFS。

```bash
git lfs install
git clone https://github.com/Zst-s/demo01.git
cd demo01
git lfs pull
source /opt/ros/humble/setup.bash
colcon build
```

本地工程使用 ROS 2 Humble。第三方系统依赖（如 Livox SDK2、G2O 等）需按对应包说明配置，仓库不包含已编译的系统库。`src/serial` 为随工程收录的源码依赖。

## 常用入口

- `mapping.sh`：建图。
- `save_passage_map.sh`：保存二维图、对应 PCD 及地图关联。
- `nav.sh 地图名 /dev/ttyACM0`：导航，开启串口后可能驱动车辆。
- `docs/`：实现与排查记录。

安装外参、传感器网络地址、底盘轮廓及串口协议均为当前车辆配置，使用其他车辆前应核对配置。已有二维地图及配套注册清单保持原文件内容。

按作者要求，仓库不发布 `.pcd` 点云地图。使用 ICP 导航前，需要从原车另行复制与二维图同次保存的同名 PCD 到 `src/rm_nav_bringup/PCD/`，并保持注册清单哈希一致；也可重新建图、配套保存。二维图和注册清单不能替代 ICP 所需的 PCD。

不包含 build/install/log、现场诊断构建、录包和历史备份。临时开机单目标验证位于工程之外，未混入本仓库。各第三方组件保留其原版权和许可证。

另排除 FAST-LIO 文档中约 50 MiB 的 `real_experiment2.gif` 演示动图，保留其余源码和文档。
