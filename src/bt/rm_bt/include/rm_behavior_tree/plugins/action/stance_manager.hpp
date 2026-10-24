#ifndef RM_BEHAVIOR_TREE_PLUGINS_ACTION_STANCE_MANAGER_HPP_
#define RM_BEHAVIOR_TREE_PLUGINS_ACTION_STANCE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include <chrono>
#include <memory>
#include <optional>

namespace rm_behavior_tree
{
enum class Stance : uint8_t
{
  ATTACK = 1,
  DEFENSE = 2,
  MOVE = 3
};

class StanceTimeManager
{
public:
  static StanceTimeManager& getInstance()
  {
    static StanceTimeManager instance;
    return instance;
  }

  void init()  //初始化
  {
    current_stance_ = Stance::MOVE;
    last_switch_time_ = std::chrono::steady_clock::now();
    last_update_time_ = std::chrono::steady_clock::now();
    attack_time_ = 0.0;
    defense_time_ = 0.0;
    move_time_ = 0.0;
    last_hp_ = 400;
    initialized_ = true;
  }

  void updateAccumulatedTime()  //更新姿态累计时间
  {
    if (!initialized_)
      init();

    auto now = std::chrono::steady_clock::now();
    double delta = std::chrono::duration<double>(now - last_update_time_).count();
    last_update_time_ = now;

    switch (current_stance_)
    {
      case Stance::ATTACK:
        attack_time_ += delta;
        break;
      case Stance::DEFENSE:
        defense_time_ += delta;
        break;
      case Stance::MOVE:
        move_time_ += delta;
        break;
    }
  }

  bool isCooldownReady() const  //判断冷却好了没
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_switch_time_).count();
    return elapsed >= 5.0;
  }

  double getCooldownRemaining() const
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_switch_time_).count();
    return std::max(0.0, 5.0 - elapsed);
  }

  bool switchStance(Stance new_stance)  //切换姿态
  {
    if (new_stance == current_stance_)
      return true;
    // if (!isCooldownReady())
    //   return false;

    current_stance_ = new_stance;
    last_switch_time_ = std::chrono::steady_clock::now();
    return true;
  }

  void syncStance(Stance new_stance)  // 从外部同步姿态
  {
    if (new_stance != current_stance_)
    {
      current_stance_ = new_stance;
      last_switch_time_ = std::chrono::steady_clock::now();  // 重置冷却时间
    }
  }

  Stance getCurrentStance() const
  {
    return current_stance_;
  }
  uint8_t getCurrentStanceInt() const
  {
    return static_cast<uint8_t>(current_stance_);
  }

  double getAttackTime() const
  {
    return attack_time_;
  }
  double getDefenseTime() const
  {
    return defense_time_;
  }
  double getMoveTime() const
  {
    return move_time_;
  }

  bool isStanceTimeout(Stance stance) const  //判断姿态累计时间是否超时
  {
    switch (stance)
    {
      case Stance::ATTACK:
        return attack_time_ >= 175.0;
      case Stance::DEFENSE:
        return defense_time_ >= 175.0;
      case Stance::MOVE:
        return move_time_ >= 175.0;
    }
    return false;
  }

  bool hasTimeRemaining(Stance stance) const
  {
    return !isStanceTimeout(stance);
  }

  bool updateHpAndCheckDamage(uint16_t current_hp)  //判断受击
  {
    bool damaged = (current_hp < last_hp_ - 8);
    last_hp_ = current_hp;
    return damaged;
  }

private:
  StanceTimeManager() : initialized_(false){};

  bool initialized_;
  Stance current_stance_;
  std::chrono::steady_clock::time_point last_switch_time_;
  std::chrono::steady_clock::time_point last_update_time_;

  double attack_time_;
  double defense_time_;
  double move_time_;
  uint16_t last_hp_;
};

/**
 * @brief 更新姿态时间的 Action 节点
 * @param[in] current_hp 当前血量
 * @param[in] current_stance 当前姿态
 * @param[out] cooldown_ready 冷却是否完成
 * @param[out] is_damaged 是否被打
 * @param[out] attack_time 进攻姿态累计时间
 * @param[out] defense_time 防御姿态累计时间
 * @param[out] move_time 移动姿态累计时间
 */
class UpdateStanceTime : public BT::SyncActionNode
{
public:
  UpdateStanceTime(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::uint16_t>("current_hp"),
      BT::InputPort<std::uint8_t>("current_stance", "当前姿态"),
      BT::OutputPort<bool>("cooldown_ready", "冷却是否完成"),
      BT::OutputPort<bool>("is_damaged", "是否受击"),
      BT::OutputPort<double>("attack_time"),
      BT::OutputPort<double>("defense_time"),
      BT::OutputPort<double>("move_time"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto& manager = StanceTimeManager::getInstance();

    // 从端口获取当前姿态并同步
    uint8_t current_stance_from_port;
    if (getInput<std::uint8_t>("current_stance", current_stance_from_port))
    {
      manager.syncStance(static_cast<Stance>(current_stance_from_port));
    }

    manager.updateAccumulatedTime();

    uint16_t current_hp = 400;
    getInput<std::uint16_t>("current_hp", current_hp);
    bool is_damaged = manager.updateHpAndCheckDamage(current_hp);

    // setOutput("current_stance", manager.getCurrentStanceInt());
    setOutput("cooldown_ready", manager.isCooldownReady());
    setOutput("is_damaged", is_damaged);
    setOutput("attack_time", manager.getAttackTime());
    setOutput("defense_time", manager.getDefenseTime());
    setOutput("move_time", manager.getMoveTime());

    return BT::NodeStatus::SUCCESS;
  }
};

class SwitchStance : public BT::SyncActionNode
{
public:
  SwitchStance(const std::string& name, const BT::NodeConfig& config, const rclcpp::Node::SharedPtr& nh)
    : BT::SyncActionNode(name, config), node_(nh)
  {
    target_stance_pub = nh->create_publisher<std_msgs::msg::UInt8>("/target_stance", 10);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("target_stance") };
  }

  BT::NodeStatus tick() override
  {
    int target = 3;
    getInput<int>("target_stance", target);

    auto& manager = StanceTimeManager::getInstance();
    Stance target_stance = static_cast<Stance>(target);

    // auto pickFallbackStance = [&manager]() -> std::optional<Stance> {
    //   // Priority order: DEFENSE -> ATTACK -> MOVE
    //   if (manager.hasTimeRemaining(Stance::DEFENSE))
    //     return Stance::DEFENSE;
    //   if (manager.hasTimeRemaining(Stance::ATTACK))
    //     return Stance::ATTACK;
    //   if (manager.hasTimeRemaining(Stance::MOVE))
    //     return Stance::MOVE;
    //   return std::nullopt;
    // };

    if (manager.getCurrentStance() == target_stance)
    {
      return BT::NodeStatus::SUCCESS;
    }

    // 看目标姿态还有没有余量
    // if (!manager.hasTimeRemaining(target_stance))
    // {
    //   auto fallback = pickFallbackStance();
    //   if (!fallback.has_value())
    //   {
    //     return BT::NodeStatus::FAILURE;
    //   }
    //   target_stance = fallback.value();
    // }

    bool success = manager.switchStance(target_stance);

    if (success)
    {
      //发送切换消息给trans_node.cpp
      std_msgs::msg::UInt8 msg;
      msg.data = static_cast<uint8_t>(target_stance);
      target_stance_pub->publish(msg);

      const char* stance_name = (target == 1) ? "进攻" : (target == 2) ? "防御" : "移动";
      RCLCPP_INFO(node_->get_logger(), "已切换到目标姿态: %s(%d)", stance_name, target);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr target_stance_pub;
};

}  // namespace rm_behavior_tree
#endif  // RM_BEHAVIOR_TREE_PLUGINS_ACTION_STANCE_MANAGER_HPP_