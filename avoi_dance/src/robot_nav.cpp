/**
 * @file robot_nav.cpp
 * @author Dimitri JACQUEMONT (jacquemont.dim@gmail.com)
 * @brief This file contains the implementation of the RobotNav class.
 * @details This class is used to control the autonomous navigation of 
 *          the robot and the avoidance of the other robot.
 * 
 * @version 2
 * @date 2024-03-12 
 */


#include <cstdlib>
#include <ctime>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_msgs/action/rotate_angle.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define M_PI 3.14159265358979323846
#define LIMIT_X 1.5
#define LIMIT_Y 1.5
#define PROB_RIGHT 0.85
#define PROB_LEFT 0.15

/**
 * @brief Enum class for the robot's behavior state
 * 
 */
enum class BehaviorState {
    IDLE,
    ROTATING_RIGHT,
    ROTATING_LEFT,
    FORWARD
};

/**
 * @brief Class for the autonomous navigation of the robot and avoidance of the other robot
 * 
 */
class RobotNav : public rclcpp::Node {
public:
    struct Position {
        double x;
        double y;
        double yaw;
    };
    RobotNav() : Node("robot_nav") {

        // Declare all necessary parameters
        this->declare_parameter<std::string>("odometry_topic_self", "");
        this->declare_parameter<std::string>("odometry_topic_other", "");
        this->declare_parameter<int>("seed", 42);
        this->declare_parameter<std::string>("cmd_vel_topic", "");
        this->declare_parameter<double>("initial_x_self", 0.0);
        this->declare_parameter<double>("initial_y_self", 0.0);
        this->declare_parameter<double>("initial_yaw_self", 0.0);
        this->declare_parameter<double>("initial_x_other", 0.0);
        this->declare_parameter<double>("initial_y_other", 0.0);
        this->declare_parameter<double>("initial_yaw_other", 0.0);

        this->get_parameter("odometry_topic_self", odometry_topic_self_);
        this->get_parameter("odometry_topic_other", odometry_topic_other_);
        this->get_parameter("seed", seed_);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        this->get_parameter("initial_x_self", initial_x_self_);
        this->get_parameter("initial_y_self", initial_y_self_);
        this->get_parameter("initial_yaw_self", initial_yaw_self_);
        this->get_parameter("initial_x_other", initial_x_other_);
        this->get_parameter("initial_y_other", initial_y_other_);
        this->get_parameter("initial_yaw_other", initial_yaw_other_);

        // Subscribe to the robot's odometry topic
        odometry_subscription_self_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_self_, 1,
            std::bind(&RobotNav::odometry_callback_self, this, std::placeholders::_1));

        // Subscribe to the other robot's odometry topic
        odometry_subscription_other_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_other_, 1,
            std::bind(&RobotNav::odometry_callback_other, this, std::placeholders::_1));

        // Publish the robot's velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);

        // Create a timer to publish the velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotNav::publish_velocity, this));

        // Initialize gloabl variables
        position_self_ = {initial_x_self_, initial_y_self_, initial_yaw_self_};
        position_other_ = {initial_x_other_, initial_y_other_, initial_yaw_other_};
        avoidance_mode_ = false;
        behavior_state_ = BehaviorState::IDLE;

        // Set the random seed
        srand(seed_);

        // Log the initial global positions and orientations
        RCLCPP_INFO(
            this->get_logger(), 
            "Initial global position of robot_self: x = %f, y = %f, yaw = %f radians",
            position_self_.x, position_self_.y, position_self_.yaw);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Initial global position of robot_other: x = %f, y = %f, yaw = %f radians",
            position_other_.x, position_other_.y, position_other_.yaw);
    }

private:
    /**
     * @brief Callback function for the robot's odometry topic
     * @details The new odometry coordinates of "robot_self" are transformed in global coordinates
     * 
     * @param[in] msg 
     */
    void odometry_callback_self(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position_self_.x = initial_x_self_ + msg->pose.pose.position.x;
        position_self_.y = initial_y_self_ + msg->pose.pose.position.y;
        position_self_.yaw = quaternion_to_yaw(msg->pose.pose.orientation);
        position_self_.yaw = normalize_angle(position_self_.yaw);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Global position of robot_self: x = %f, y = %f, yaw = %f radians",
            position_self_.x, position_self_.y, position_self_.yaw);
    }

    /**
     * @brief Callback function for the other robot's odometry topic
     * @details The new odometry coordinates of "robot_other" are transformed in global coordinates
     * 
     * @param[in] msg 
     */
    void odometry_callback_other(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position_other_.x = initial_x_other_ + msg->pose.pose.position.x;
        position_other_.y = initial_y_other_ + msg->pose.pose.position.y;
        position_other_.yaw = quaternion_to_yaw(msg->pose.pose.orientation);
        position_other_.yaw = normalize_angle(position_other_.yaw);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Global position of robot_other: x = %f, y = %f, yaw = %f radians",
            position_other_.x, position_other_.y, position_other_.yaw);

        // Check if the other robot is too close
        if (std::pow(std::pow(position_other_.x - position_self_.x, 2) + 
            std::pow(position_other_.y - position_self_.y, 2), 0.5) < 1.0 && avoidance_mode_ == false) {
            avoidance_mode_ = true;

            RCLCPP_DEBUG(
                this->get_logger(),
                "Robot_other is too close! Avoidance mode activated.");
        }
        else {
            avoidance_mode_ = false;
        }
    }

    /**
     * @brief Select the robot's behavior based on the other robot's position
     * @details The robot's behavior is based on both robot's position and orientation
     * 
     * @return BehaviorState 
     */
    BehaviorState behaviour_selector()
    {
        if (!avoidance_mode_)
        {
            // Check if the robot is close to the limits of the environment
            // If the robot is close to the limits, the robot should rotate
            if (position_self_.x >= LIMIT_X || position_self_.x <= -LIMIT_X || 
                position_self_.y >= LIMIT_Y || position_self_.y <= -LIMIT_Y)
            {
                if (position_self_.x >= LIMIT_X && (position_self_.yaw >= 3 * M_PI / 2 || position_self_.yaw <= M_PI / 2)) {
                    return BehaviorState::ROTATING_RIGHT;
                }
                else if (position_self_.x <= -LIMIT_X && (position_self_.yaw >= M_PI / 2 && position_self_.yaw <= 3 * M_PI / 2)) {
                    return BehaviorState::ROTATING_RIGHT;
                }
                else if (position_self_.y >= LIMIT_Y && position_self_.yaw <= M_PI) {
                    return BehaviorState::ROTATING_RIGHT;
                }
                else if (position_self_.y <= -LIMIT_Y && position_self_.yaw >= M_PI) {
                    return BehaviorState::ROTATING_RIGHT;
                }
                else {
                    return BehaviorState::FORWARD;
                }
            } 
            else {
                // compute a random number between 0 and 1 more based on a random seed
                double random_number = (double)rand() / RAND_MAX;
                if (random_number > PROB_RIGHT) {
                    return BehaviorState::ROTATING_RIGHT;
                } 
                else if (random_number < PROB_LEFT) {
                    return BehaviorState::ROTATING_LEFT;
                }
                else {
                    return BehaviorState::FORWARD;
                }
            }
        }
        else
        {
            // compute the angle between the two robots
            double angle = atan2(position_other_.y - position_self_.y, position_other_.x - position_self_.x);
            angle = normalize_angle(angle);

            RCLCPP_DEBUG(
                this->get_logger(),
                "Angle between the two robots: %f radians",
                angle);

            // compute the angle between the robot's orientation and the angle between the two robots
            double angle_diff = normalize_angle(angle - position_self_.yaw);

            // avoidance of the other robot
            if (angle_diff < M_PI / 2) {
                return BehaviorState::ROTATING_RIGHT;
            }
            else if (angle_diff > 3 * M_PI / 2) {
                return BehaviorState::ROTATING_LEFT;
            }
            else {
                return BehaviorState::FORWARD;
            }
        }
        return BehaviorState::IDLE;
    }

    /**
     * @brief Publish the robot's velocity commands
     * @details The robot's velocity is based on the desired robot's behaviour.
     */
    void publish_velocity()
    {
        double x_speed, yaw_speed;

        behavior_state_ = behaviour_selector();

        // Robot is going forward
        if (behavior_state_ == BehaviorState::FORWARD) {
            x_speed = 1.0;
            yaw_speed = 0.0;
        }
        // Robot is ROTATING_RIGHT
        else if (behavior_state_ == BehaviorState::ROTATING_RIGHT) {
            x_speed = 0.0;
            yaw_speed = -1.0;
            behavior_state_ = BehaviorState::FORWARD;
        }
        // Robot is ROTATING_LEFT
        else if (behavior_state_ == BehaviorState::ROTATING_LEFT) {
            x_speed = 0.0;
            yaw_speed = 1.0;
            behavior_state_ = BehaviorState::FORWARD;
        }
        // Robot is idle
        else {
            x_speed = 0.0;
            yaw_speed = 0.0;
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "Robot's behavior: %d, x_speed = %f, yaw_speed = %f",
            (int)behavior_state_, x_speed, yaw_speed);

        // Publish the velocity commands
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x_speed;
        msg.angular.z = yaw_speed;
        cmd_vel_publisher_->publish(msg);
    }

    /**
     * @brief Convert a quaternion to a yaw angle
     * 
     * @param[in] quat 
     * @return double 
     */
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    /**
     * @brief Normalize an angle to the range [0, 2*PI)
     * 
     * @param[in] angle 
     * @return double 
     */
    double normalize_angle(double angle) {
        while (angle < 0) angle += 2.0 * M_PI;
        while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
        return angle;
    }

    // Declare the subscribers, publishers and the timer
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_self_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_other_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Declare the variables
    std::string odometry_topic_self_, odometry_topic_other_, cmd_vel_topic_;
    double initial_x_self_, initial_y_self_, initial_yaw_self_;
    double initial_x_other_, initial_y_other_, initial_yaw_other_;
    BehaviorState behavior_state_;
    Position position_self_, position_other_;
    bool avoidance_mode_;
    int seed_;
};

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNav>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}