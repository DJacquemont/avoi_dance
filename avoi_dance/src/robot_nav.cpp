#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <irobot_create_msgs/action/rotate_angle.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <geometry_msgs/msg/twist.hpp>
#include <cstdlib>
#include <ctime>
#include <memory>

#define M_PI 3.14159265358979323846

/**
 * @brief Enum class for the robot's behavior state
 * 
 */
enum class BehaviorState {
    IDLE,
    ROTATING,
    FORWARD
};

/**
 * @brief Class for the autonomous navigation of the robot and avoidance of the other robot
 * 
 */
class RobotNav : public rclcpp::Node {
public:
    RobotNav() : Node("robot_nav") {

        // Declare all necessary parameters
        this->declare_parameter<std::string>("odometry_topic_self", "");
        this->declare_parameter<std::string>("odometry_topic_other", "");
        this->declare_parameter<std::string>("hazard_detection_topic", "");
        this->declare_parameter<std::string>("cmd_vel_topic", "");
        this->declare_parameter<double>("initial_x_self", 0.0);
        this->declare_parameter<double>("initial_y_self", 0.0);
        this->declare_parameter<double>("initial_yaw_self", 0.0);
        this->declare_parameter<double>("initial_x_other", 0.0);
        this->declare_parameter<double>("initial_y_other", 0.0);
        this->declare_parameter<double>("initial_yaw_other", 0.0);

        this->get_parameter("odometry_topic_self", odometry_topic_self);
        this->get_parameter("odometry_topic_other", odometry_topic_other);
        this->get_parameter("hazard_detection_topic", hazard_detection_topic);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);
        this->get_parameter("initial_x_self", initial_x_self);
        this->get_parameter("initial_y_self", initial_y_self);
        this->get_parameter("initial_yaw_self", initial_yaw_self);
        this->get_parameter("initial_x_other", initial_x_other);
        this->get_parameter("initial_y_other", initial_y_other);
        this->get_parameter("initial_yaw_other", initial_yaw_other);

        // Subscribe to the robot's odometry topic
        odometry_subscription_self_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_self, 1,
            std::bind(&RobotNav::odometry_callback_self, this, std::placeholders::_1));

        // Subscrine to the other robot's odometry topic
        odometry_subscription_other_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_other, 1,
            std::bind(&RobotNav::odometry_callback_other, this, std::placeholders::_1));

        // Subscribe to the robot's hazard detection topic
        hazard_detection_subscription_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            hazard_detection_topic, 1,
            std::bind(&RobotNav::hazard_detection_callback, this, std::placeholders::_1));

        // Publish the robot's velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);

        // Create a timer to publish the velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotNav::publish_velocity, this));

        // Initialize gloabl variables
        behavior_state = BehaviorState::FORWARD;
        x_self_ = initial_x_self;
        y_self_ = initial_y_self;
        yaw_self_ = initial_yaw_self;

        // Log the initial global positions and orientations
        RCLCPP_INFO(
            this->get_logger(), 
            "Initial global position of robot_self: x = %f, y = %f, yaw = %f radians",
            initial_x_self, initial_y_self, initial_yaw_self);

        RCLCPP_INFO(
            this->get_logger(),
            "Initial global position of robot_other: x = %f, y = %f, yaw = %f radians",
            initial_x_other, initial_y_other, initial_yaw_other);
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
        yaw_self_old_ = yaw_self_;

        x_self_ = initial_x_self + msg->pose.pose.position.x;
        y_self_ = initial_y_self + msg->pose.pose.position.y;
        yaw_self_ = quaternion_to_yaw(msg->pose.pose.orientation);
        yaw_self_ = normalize_angle(yaw_self_);
        

        RCLCPP_INFO(
            this->get_logger(),
            "Global position of robot_self: x = %f, y = %f, yaw = %f radians",
            x_self_, y_self_, yaw_self_);
    }

    /**
     * @brief Callback function for the other robot's odometry topic
     * @details The new odometry coordinates of "robot_other" are transformed in global coordinates
     * 
     * @param[in] msg 
     */
    void odometry_callback_other(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_other_ = initial_x_other + msg->pose.pose.position.x;
        y_other_ = initial_y_other + msg->pose.pose.position.y;
        yaw_other_ = quaternion_to_yaw(msg->pose.pose.orientation);
        yaw_other_ = normalize_angle(yaw_other_);

        RCLCPP_INFO(
            this->get_logger(),
            "Global position of robot_other: x = %f, y = %f, yaw = %f radians",
            x_other_, y_other_, yaw_other_);


        // // Check if the other robot is too close (FEATURE NOT WARKING YET)
        // if (std::pow(std::pow(x_self_ - x_other_, 2) + std::pow(y_self_ - y_other_, 2), 0.5) < 1.0) {

        //     if (last_action_was_rotation) {
        //         yaw_goal = 0.0;
        //         last_action_was_rotation = false;
        //     }
        //     else {
        //         yaw_goal = M_PI;
        //         last_action_was_rotation = true;
        //     }

        //     RCLCPP_INFO(
        //         this->get_logger(),
        //         "Robot_other is too close! The robot_self should rotate 180 degrees");
        // }
    }

    /**
     * @brief Callback function for the robot's hazard detection topic (NOT WORKING YET - FIX THE HAZARD DETECTION TOPIC)
     * @details The robot's behavior is updated based on the detected hazards.
     *          If a bump is detected, the robot should rotate. This function works only for one robot, however the second 
     *          one is not working yet (namespace issue ?).
     * 
     * @param[in] msg 
     */
    void hazard_detection_callback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg) {
        for (const auto& detection : msg->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {

                behavior_state = BehaviorState::ROTATING;

                RCLCPP_INFO(this->get_logger(),
                            "Bump detected! The robot_self should rotate 180 degrees.");
                break;
            }
        }
    }

    /**
     * @brief Publish the robot's velocity commands
     * @details The robot's velocity is based on the desired robot's behaviour.
     */
    void publish_velocity()
    {
        double x_speed;
        double yaw_speed;

        // Robot is going forward
        if (behavior_state == BehaviorState::FORWARD) {
            x_speed = 1.0;
            yaw_speed = 0.0;
        }
        // Robot is rotating
        else if (behavior_state == BehaviorState::ROTATING) {
            x_speed = 0.0;
            yaw_speed = 1.0;
            behavior_state = BehaviorState::FORWARD;
        }
        // Robot is idle
        else {
            x_speed = 0.0;
            yaw_speed = 0.0;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "x_speed = %f, yaw_speed = %f",
            x_speed, yaw_speed);

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
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_detection_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Retrieve the parameter values
    std::string odometry_topic_self, odometry_topic_other, hazard_detection_topic, cmd_vel_topic;
    double initial_x_self, initial_y_self, initial_yaw_self, initial_x_other, initial_y_other, initial_yaw_other;

    // Declare the global variables
    enum BehaviorState behavior_state;
    double x_self_, y_self_, yaw_self_, yaw_self_old_; 
    double x_other_, y_other_, yaw_other_;
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