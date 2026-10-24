#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <filesystem>
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include "rm_behavior_tree/plugins/action/stance_manager.hpp"
#include "rm_behavior_tree/plugins/action/nav2pose.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tree_exec_node");
  BT::BehaviorTreeFactory factory;
  // std::string xml;
  // int x = 2;
  // switch (x)
  // {
  //   case 1:
  //     xml = "behavior_tree/test.xml";
  //     break;
  //   case 2:
  //     xml = "behavior_tree/test_2.xml";
  //     break;
  //   case 3:
  //     xml = "behavior_tree/test_3.xml";
  //     break;
  //   default:
  //     xml = "behavior_tree/test.xml";
  //     break;
  // }

  node->declare_parameter<std::string>("tree_xml_file", "behavior_tree/test.xml");
  // auto tree_num_sub = node->create_subscription<std_msgs::msg::UInt8>(
  //     "/tree_num", 10, [node, &xml](const std_msgs::msg::UInt8::SharedPtr msg) {
  //       switch (msg->data)
  //       {
  //         case 1:
  //           xml = "behavior_tree/Full_Check_RMUC.xml";
  //           break;
  //         case 2:
  //           xml = "behavior_tree/test2.xml";
  //           break;
  //         case 3:
  //           xml = "behavior_tree/test3.xml";
  //           break;
  //         default:
  //           xml = "behavior_tree/test.xml";
  //           break;
  //       }
  //       // node->set_parameter(rclcpp::Parameter("tree_xml_file", xml));
  //     });

  node->declare_parameter("groot_port", 5556);
  node->declare_parameter<bool>("enable_groot", true);
  node->declare_parameter<int>("tick_period_milliseconds", 100);
  // nav2pose node's param
  node->declare_parameter("send_goal_timeout_ms", 5000);

  std::string xml_param = node->get_parameter("tree_xml_file").as_string();
  // std::string xml_param = xml;
  unsigned int groot_port = 5556;
  node->get_parameter("groot_port", groot_port);
  bool enable_groot = true;
  node->get_parameter("enable_groot", enable_groot);
  int tick_period_milliseconds;
  node->get_parameter("tick_period_milliseconds", tick_period_milliseconds);

  std::string pkg_shared = ament_index_cpp::get_package_share_directory("rm_bt");
  RCLCPP_INFO(node->get_logger(), "Package rm_bt share dir: %s", pkg_shared.c_str());
  std::string xml_path;
  if (xml_param.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "tree_xml_file参数没有啦");
    rclcpp::shutdown();
    return -1;
  }
  xml_path = pkg_shared + "/" + xml_param;

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = node;

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = node;
  params_send_goal.default_port_value = "navigate_to_pose";
  params_send_goal.wait_for_server_timeout = std::chrono::seconds(10);  // 等待 action server 10秒
  params_send_goal.server_timeout = std::chrono::seconds(120);          // action 执行超时 120秒

  BT::RosNodeParams params_pub_nav;
  params_pub_nav.nh = node;
  params_pub_nav.default_port_value = "goal_pose";

  BT::RosNodeParams params_pub_super_ele;
  params_pub_super_ele.nh = node;
  params_pub_super_ele.default_port_value = "/super_ele";

  BT::RosNodeParams params_pub_intensify;
  params_pub_intensify.nh = node;
  params_pub_intensify.default_port_value = "/intensify";

  BT::RosNodeParams params_pub_if_qifu;
  params_pub_if_qifu.nh = node;
  params_pub_if_qifu.default_port_value = "/if_qifu";

  const std::vector<std::string> msg_update_plugins_libs = { "sub_game_status",
                                                            //  "sub_mid_if",
                                                             "sub_midhigh_if",
                                                             "sub_fort_if",
                                                             "sub_outpost_if",
                                                             "sub_remain_time",
                                                             "sub_BothCampHp",
                                                             "sub_current_stance",
                                                             "sub_enemydetected",
                                                             "sub_RobotStatus",
                                                             "pub_current_location"  };

  const std::vector<std::string> bt_plugins_libs = { "restores_health",
                                                     "randomselector",
                                                     "check_hp",
                                                     "check_pill",
                                                     "timer_action",
                                                     "check_elapsed_time" };

  factory.registerNodeType<rm_behavior_tree::Nav2Pose>("Nav2Pose", node);
  factory.registerNodeType<rm_behavior_tree::UpdateStanceTime>("UpdateStanceTime");

  for (const auto& p : msg_update_plugins_libs)
  {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }
  for (const auto& p : bt_plugins_libs)
  {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  BT::NodeBuilder builder = [node](const std::string& name,
                                   const BT::NodeConfig& config) -> std::unique_ptr<BT::TreeNode> {
    return std::make_unique<rm_behavior_tree::SwitchStance>(name, config, node);
  };

  factory.registerBuilder<rm_behavior_tree::SwitchStance>("SwitchStance", builder);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_nav2_goal"), params_pub_nav);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_super_ele"), params_pub_super_ele);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_intensify"), params_pub_intensify);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_if_qifu"), params_pub_if_qifu);
  BT::Tree tree = factory.createTreeFromFile(xml_path.c_str());

  // 调试信息
  if (tree.subtrees.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Tree has no subtrees!");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Tree loaded with %zu subtrees", tree.subtrees.size());
    for (const auto& subtree : tree.subtrees)
    {
      RCLCPP_INFO(node->get_logger(), "Subtree has %zu nodes", subtree->nodes.size());
    }
  }

  // Groot2Publisher 必须在任何 tick 之前创建
  std::unique_ptr<BT::Groot2Publisher> groot_publisher;
  if (enable_groot)
  {
    try
    {
      groot_publisher = std::make_unique<BT::Groot2Publisher>(tree, groot_port);
      RCLCPP_INFO(node->get_logger(), "Groot2Publisher started on port %u", groot_port);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to start Groot2Publisher: %s", e.what());
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok())
  {
    tree.tickOnce();
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_milliseconds));
  }

  // while(rclcpp::ok()){                 //2222222222
  //     tree.tickOnce();
  //     rclcpp::spin_some(node);  // 处理 ROS 回调
  //     std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_milliseconds));
  // }

  // rclcpp::Rate rate(1000.0 / tick_period_milliseconds);  // 转换为频率            3333333333333

  // while(rclcpp::ok()){
  //     tree.tickOnce();
  //     rclcpp::spin_some(node);  // 让 ROS 节点处理回调
  //     rate.sleep();
  // }

  rclcpp::shutdown();
  return 0;
}