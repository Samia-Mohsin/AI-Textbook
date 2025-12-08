#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class HumanoidController : public rclcpp::Node
{
public:
    HumanoidController() : Node("humanoid_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Humanoid Controller node initialized");
        
        // Publisher for joint commands
        joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);
        
        // Subscriber for joint states
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&HumanoidController::jointStateCallback, this, std::placeholders::_1));
        
        // Subscriber for velocity commands
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HumanoidController::cmdVelCallback, this, std::placeholders::_1));
        
        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&HumanoidController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Humanoid Controller setup complete");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Store current joint states
        current_joint_states_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Received joint states with %zu joints", msg->name.size());
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Store velocity command
        current_cmd_vel_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Received velocity command: linear.x=%f, angular.z=%f", 
                    msg->linear.x, msg->angular.z);
    }

    void controlLoop()
    {
        // This is a simplified control loop
        // In a real implementation, this would contain more sophisticated control logic
        
        // Example: Send a simple joint command
        auto command_msg = std_msgs::msg::Float64MultiArray();
        command_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 8 joint positions
        
        joint_command_publisher_->publish(command_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published joint command in control loop");
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    sensor_msgs::msg::JointState current_joint_states_;
    geometry_msgs::msg::Twist current_cmd_vel_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanoidController>());
    rclcpp::shutdown();
    return 0;
}