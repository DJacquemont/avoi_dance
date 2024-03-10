#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

class RobotNav : public rclcpp::Node
{
public:
    RobotNav(const std::string& namespace_, const std::string& odometry_topic)
    : Node("robot_nav", namespace_)
    {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 10,
            std::bind(&RobotNav::odometry_callback, this, std::placeholders::_1));
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received odometry: position x: '%.2f', y: '%.2f'",
                    msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <namespace> <odometry_topic>\n";
        return 1;
    }

    std::string namespace_ = argv[1];
    std::string odometry_topic = argv[2];

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNav>(namespace_, odometry_topic);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
