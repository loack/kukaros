#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateSimNode : public rclcpp::Node
{
public:
  JointStateSimNode()
  : Node("kvp_joint_state_sim_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&JointStateSimNode::timer_callback, this));
    joint_names_ = {"remus_joint_a1", "remus_joint_a2", "remus_joint_a3", "remus_joint_a4", "remus_joint_a5", "remus_joint_a6"};
    t_ = 0.0;
  }

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
      msg.position[i] = std::sin(t_ + i);
    publisher_->publish(msg);
    t_ += 0.05;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  double t_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSimNode>());
  rclcpp::shutdown();
  return 0;
}