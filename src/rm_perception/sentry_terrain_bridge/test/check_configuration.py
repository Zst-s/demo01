#!/usr/bin/env python3
"""只读静态验收：不编译、不运行 launch、不连接底盘。"""
import argparse
import ast
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", type=Path)
    args = parser.parse_args()
    root = args.workspace.resolve()
    bringup = root / "src/rm_nav_bringup"
    config = bringup / "config/reality"
    old = load(config / "nav2_params_real.yaml")
    new = load(config / "nav2_params_rog_map_real.yaml")
    restored = copy.deepcopy(new)
    expected = {
        "local_costmap": ["terrain_layer", "inflation_layer"],
        "global_costmap": ["static_layer", "terrain_layer", "inflation_layer"],
    }
    for name, plugins in expected.items():
        params = restored[name][name]["ros__parameters"]
        assert params["plugins"] == plugins
        layer = params.pop("terrain_layer")
        assert layer["plugin"] == "sentry_terrain_bridge::TerrainLayer"
        assert layer["topic"] == "/terrain/grid"
        assert layer["data_timeout"] > 0
        params["plugins"] = old[name][name]["ros__parameters"]["plugins"]
    assert restored == old, "发现感知层之外的 Nav2 参数变化"
    print("PASS: 两套新车 Nav2 配置除感知插件外一致")

    cfg = load(config / "rog_map_real.yaml")["sentry_terrain"]["ros__parameters"]
    rog = cfg["rog_map"]
    assert rog["frame_id"] == "odom"
    assert not rog["ros_callback"]["enable"]
    assert not rog["projection"]["prior_map"]["enable"]
    assert not rog["load_pcd_en"]
    assert rog["decay"]["clear_time"] > rog["decay"]["keep_time"]
    assert cfg["cloud_topic"] == "/cloud_registered_body"
    assert cfg["cloud_coordinates"] == "imu"
    assert cfg["mapping_scan_enabled"] is False
    assert cfg["mapping_scan_frame"] == "livox_frame"
    assert 0 < cfg["mapping_scan_range_min"] < cfg["mapping_scan_range_max"] <= 5.0
    projection = rog["projection"]
    assert projection["wall_height_delta_min"] > projection["surface_height_delta_max"]
    assert projection["tunnel_height_delta_min"] > projection["surface_height_delta_max"]
    assert projection["tunnel_occupancy_ratio_max"] < projection["wall_occupancy_ratio_min"]
    print("PASS: 地形坐标、分类门限、衰减和输入配置自洽")

    for name in ["fastlio_mid360_real.yaml", "pointlio_mid360_real.yaml"]:
        lio = load(config / name)["/**"]["ros__parameters"]
        assert len(lio["mapping"]["extrinsic_T"]) == 3
        assert not lio["mapping"]["extrinsic_est_en"], "在线外参估计需要同步更新感知射线起点"
        assert lio["publish"]["scan_publish_en"]
    launch_source = (bringup / "launch/bringup_real.launch.py").read_text(encoding="utf-8")
    ast.parse(launch_source)
    assert "'scan_topic': '/scan'" in launch_source
    assert "'mapping_scan_enabled': False" in launch_source
    assert "'passage_observer', default_value='false'" in launch_source
    assert "'passage_virtual_preview', default_value='false'" in launch_source
    assert "'passage_evidence'" not in launch_source
    assert "'clearance.publish_verified_segments': False" in launch_source
    assert "'clearance.virtual_preview': ParameterValue(" in launch_source
    assert '"0.32 if \'", passage_virtual_preview' in launch_source
    pending = load(config / "vehicle_geometry_new_pending.yaml")
    assert pending["robot_height_m"] == .30
    assert pending["robot_length_m"] == .62 and pending["robot_width_m"] == .58
    assert pending["base_link_height_above_ground_m"] == .11
    assert pending["calibration_verified"] is False and pending["passage_motion_enabled"] is False
    assert pending["lidar_translation_from_base_link_m"] == [-.32, 0., .176]
    assert "vehicle_geometry_new_pending.yaml" not in launch_source
    footprint = "[[-0.32, -0.29], [0.30, -0.29], [0.30, 0.29], [-0.32, 0.29]]"
    for name in ("local_costmap", "global_costmap"):
        params = new[name][name]["ros__parameters"]
        assert "robot_radius" not in params
        assert params["footprint"] == footprint and params["footprint_padding"] == .01
    controller = new["controller_server"]["ros__parameters"]["FollowPath"]
    assert controller["footprint_model"] == dict(type="polygon", vertices=footprint)
    assert controller["min_turning_radius"] == 0 and controller["max_vel_y"] > 0
    segmentation = load(config / "segmentation_real.yaml")["ground_segmentation"]["ros__parameters"]
    assert segmentation["gravity_aligned_frame"] == "odom"
    assert abs(segmentation["sensor_height"]-(.11+.176)) < 1e-8
    for name in ("rog_map_lab.yaml", "rog_map_dense_lab.yaml"):
        lab = load(root / "src/rm_perception/sentry_passage_sim/config" / name)
        assert lab["sentry_terrain"]["ros__parameters"]["clearance"]["robot_height"] == 0.32
    print("PASS: 新车矩形轮廓/全向运动/地面分割高度一致，虚拟预览保持隔离")
    print("PASS: launch 使用原 /scan 建图；净空观察默认关闭、短段证据强制关闭")

    measured = load(config / "measurement_params_real.yaml")["base_link2livox_frame"]["xyz"]
    xacro = (bringup / "urdf/sentry_robot_real.xacro").read_text(encoding="utf-8")
    if "xacro:arg name=\"xyz\"" not in xacro or "$(arg xyz)" not in xacro:
        print("WARNING: measurement_params_real.yaml 的雷达 xyz 未被 URDF 使用；"
              f"当前传入值 {measured} 不能视为实际 TF，请在 NUC 上核对 base_link→livox_frame")

    packages = {}
    for manifest in (root / "src").rglob("package.xml"):
        if any((parent / "COLCON_IGNORE").exists() for parent in manifest.parents if parent != root):
            continue
        name = ET.parse(manifest).getroot().findtext("name")
        assert name not in packages, f"重复功能包 {name}: {manifest} 与 {packages.get(name)}"
        packages[name] = manifest
    assert "rog_map" in packages and "sentry_terrain_bridge" in packages
    # pluginlib accepts sibling <library> entries as well as a class_libraries root.
    plugin_text = (root / "src/rm_perception/sentry_terrain_bridge/terrain_plugins.xml").read_text()
    plugin = ET.fromstring("<class_libraries>"+plugin_text+"</class_libraries>")
    assert {c.get("type") for c in plugin.iter("class")} >= {
        "sentry_terrain_bridge::TerrainLayer", "sentry_terrain_bridge::StaticCorrectionLayer"}
    print(f"PASS: {len(packages)} 个功能包未重名，插件 XML 名称正确")
    print("注意：这些结果不是编译、ROS 通信、运动安全或实车验收通过。")


if __name__ == "__main__":
    main()
