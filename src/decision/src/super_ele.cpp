#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pub_super_ele");

  // Publish target stance values (1, 2, 3) on /target_stance
  auto pub = node->create_publisher<std_msgs::msg::UInt8>("/super_ele", 10);

  RCLCPP_INFO(node->get_logger(), "target_stance publisher started. Type 0 or 1 and press Enter to publish. Ctrl+D to exit.");

  // Run input loop in a simple blocking fashion. It's fine for a small utility.
  std::string line;
  while (rclcpp::ok()) {
  // Prompt
  std::cout << "Enter 0 or 1: " << std::flush;
    if (!std::getline(std::cin, line)) {
      // EOF or error
      RCLCPP_INFO(node->get_logger(), "stdin closed, exiting");
      break;
    }
    if (line.empty()) continue;
    // trim
    auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) continue;
    char c = line[first];
    if (c == '1' || c == '0') {
      auto msg = std_msgs::msg::UInt8();
      msg.data = static_cast<uint8_t>(c - '0');
      pub->publish(msg);
      RCLCPP_INFO(node->get_logger(), "Published /target_stance: %u", msg.data);
    } else {
      std::cout << "Invalid input, please type 0 or 1" << std::endl;
    }
    // allow ROS callbacks (not strictly necessary here)
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
