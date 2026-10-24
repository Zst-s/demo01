// 无机器人、无控制器的 Nav2 图层宿主。使用真实插件，不手工模拟融合算法。
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/buffer.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<nav2_util::LifecycleNode>("fusion_harness", "/passage_fusion_test");
    if (std::string(node->get_namespace()) != "/passage_fusion_test") {
      throw std::runtime_error("Harness may only run in /passage_fusion_test");
    }
    // Refuse unconfigured startup before creating plugin callback groups. A
    // plugin-construction exception can otherwise unwind through Humble's
    // LifecycleNode callback-group teardown instead of reporting the guard.
    const auto &overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    const auto preview_flag = overrides.find("production_preview");
    const bool production_preview = preview_flag != overrides.end() &&
      preview_flag->second.get_type() == rclcpp::PARAMETER_BOOL &&
      preview_flag->second.get<bool>();
    const auto opt_in = overrides.find(production_preview ?
      "correction_layer.allow_verified_evidence" :
      "correction_layer.allow_test_evidence");
    const bool test_allowed = opt_in != overrides.end() &&
      opt_in->second.get_type() == rclcpp::PARAMETER_BOOL && opt_in->second.get<bool>();
    if (!test_allowed) {
      throw std::runtime_error("Explicit test opt-in required");
    }
    // StaticLayer 需要宿主提供的公共参数。明确关闭 footprint 清障避免干扰恢复测试。
    node->declare_parameter("track_unknown_space", true);
    node->declare_parameter("use_maximum", false);
    node->declare_parameter("lethal_cost_threshold", 100);
    node->declare_parameter("unknown_cost_value", 255);
    node->declare_parameter("trinary_costmap", true);
    node->declare_parameter("map_topic", std::string(production_preview ?
      "/map" : "/passage_fusion_test/map"));
    node->declare_parameter("transform_tolerance", 0.1);
    const std::vector<std::string> names = {"static_layer", "correction_layer", "terrain_layer", "inflation_layer"};
    const std::vector<std::string> types = {
      "nav2_costmap_2d::StaticLayer",
      production_preview ? "sentry_terrain_bridge::StaticCorrectionLayer" :
                           "sentry_passage_fusion_test::StaticCorrectionLayer",
      "sentry_terrain_bridge::TerrainLayer",
      "nav2_costmap_2d::InflationLayer"};
    tf2_ros::Buffer tf(node->get_clock());
    pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d", "nav2_costmap_2d::Layer");
    nav2_costmap_2d::LayeredCostmap layered("map", false, true);
    layered.resizeMap(80, 80, 0.1, -4.0, -4.0);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = -0.15; footprint[0].y = -0.15;
    footprint[1].x = 0.15; footprint[1].y = -0.15;
    footprint[2].x = 0.15; footprint[2].y = 0.15;
    footprint[3].x = -0.15; footprint[3].y = 0.15;
    layered.setFootprint(footprint); // 仅用于测试膨胀计算，不是实车尺寸。
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    for (size_t i = 0; i < names.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "Initializing isolated layer %s", names[i].c_str());
      node->declare_parameter(names[i] + ".plugin", types[i]);
      try {
        auto plugin = loader.createSharedInstance(types[i]);
        layered.addPlugin(plugin);
        plugin->initialize(&layered, names[i], &tf, node, group);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Layer %s failed: %s", names[i].c_str(), e.what());
        throw;
      }
      RCLCPP_INFO(node->get_logger(), "Initialized isolated layer %s", names[i].c_str());
    }
    auto raw_pub = node->create_publisher<nav2_msgs::msg::Costmap>("costmap_raw", rclcpp::QoS(1).reliable());
    auto grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", rclcpp::QoS(1).reliable());
    raw_pub->on_activate();
    grid_pub->on_activate();
    auto correction = (*layered.getPlugins())[1];
    // 故障注入接口只改变测试宿主，不发布车辆指令。
    auto command_sub = node->create_subscription<std_msgs::msg::String>(
      "test_command", rclcpp::QoS(10).reliable(), [&](std_msgs::msg::String::ConstSharedPtr msg) {
        auto &p = *layered.getPlugins();
        if (msg->data == "reset") { correction->reset(); }
        if (msg->data == "disable" || msg->data == "enable") {
          node->set_parameter(rclcpp::Parameter("correction_layer.enabled", msg->data == "enable"));
        }
        if (msg->data == "wrong_order" && p[1] == correction) { std::swap(p[1], p[2]); }
        if (msg->data == "restore_order" && p[2] == correction) { std::swap(p[1], p[2]); }
      });
    auto timer = node->create_wall_timer(std::chrono::milliseconds(50), [&]() {
      layered.updateMap(-3.0, -3.0, 0.0); // 固定虚拟位置，不需要 base_link TF。
      const auto *master = layered.getCostmap();
      nav2_msgs::msg::Costmap raw;
      raw.header.frame_id = "map";
      raw.header.stamp = node->now();
      raw.metadata.update_time = raw.header.stamp;
      raw.metadata.layer = "fusion_test_master";
      raw.metadata.resolution = master->getResolution();
      raw.metadata.size_x = master->getSizeInCellsX();
      raw.metadata.size_y = master->getSizeInCellsY();
      raw.metadata.origin.position.x = master->getOriginX();
      raw.metadata.origin.position.y = master->getOriginY();
      raw.metadata.origin.orientation.w = 1.0;
      const size_t size = static_cast<size_t>(raw.metadata.size_x) * raw.metadata.size_y;
      raw.data.assign(master->getCharMap(), master->getCharMap() + size);
      raw_pub->publish(raw); // 自动测试读原始 0..255 代价，不以 RViz 颜色代替断言。
      nav_msgs::msg::OccupancyGrid grid;
      grid.header = raw.header;
      grid.info.resolution = raw.metadata.resolution;
      grid.info.width = raw.metadata.size_x;
      grid.info.height = raw.metadata.size_y;
      grid.info.origin = raw.metadata.origin;
      grid.data.reserve(size);
      for (const auto c : raw.data) {
        grid.data.push_back(c == 255 ? -1 : c == 254 ? 100 : static_cast<int8_t>(c * 99 / 253));
      }
      grid_pub->publish(grid);
    });
    RCLCPP_INFO(node->get_logger(),
      "ISOLATED HARNESS: %s correction loaded; no planner, controller or cmd_vel publisher",
      production_preview ? "production" : "test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("fusion_harness"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
