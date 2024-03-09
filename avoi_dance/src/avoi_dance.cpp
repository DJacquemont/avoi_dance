#include <rclcpp/rclcpp.hpp>

class HelloWorldNode : public rclcpp::Node
{
public:
  HelloWorldNode()
  : Node("hello_world_node")
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Hello world avoi_dance package");
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloWorldNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
