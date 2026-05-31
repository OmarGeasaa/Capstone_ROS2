#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <custom_interfaces/msg/actuator_command.hpp>
#include "biped_kinematics/ankle_math.hpp"

class AnkleIKNode : public rclcpp::Node {
public:
    AnkleIKNode() : Node("ankle_ik_node") {
        // 1. Get the prefix parameter (defaults to "left")
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        // 2. Dynamically build the topic names
        std::string command_topic = "/" + prefix_ + "_leg/commands";
        std::string target_topic = "/walking_pattern/" + prefix_ + "_ankle_pitch";

        // 3. Publisher: Sends the calculated ballscrew length to the STM32 via micro-ROS
        command_pub_ = this->create_publisher<custom_interfaces::msg::ActuatorCommand>(
            command_topic, 10);

        // 4. Subscriber: Listens to your high-level walking algorithm for the target angle dynamically
        target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            target_topic, 10,
            std::bind(&AnkleIKNode::target_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ankle IK Node operational for [%s] leg. Translating angles to ballscrew lengths.", prefix_.c_str());
    }

private:
    std::string prefix_; // Store the prefix so the callback can use it!

    void target_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // 1. Hardware Safety Clamp: Limit the requested angle to your physical CAD limits
        // Assuming roughly -10 degrees (-0.1745 rad) to +15 degrees (0.2618 rad)
        double target_rad = std::max(-0.1745, std::min(0.2618, (double)msg->data));

        // 2. Solve the triangle for distance B-C using your ankle_math.hpp
        double required_length = biped_kinematics::calculate_required_actuator_length(target_rad);

        // 3. Prepare the command for the micro-ROS CAN bus
        auto cmd_msg = custom_interfaces::msg::ActuatorCommand();
        cmd_msg.joint_id = 0; // ID 0 routes this specifically to the Ankle SEA loop on the STM32
        cmd_msg.target_length = static_cast<float>(required_length);
        
        // Feed-forward velocity (can be updated later if your walking algorithm calculates it)
        cmd_msg.target_velocity = 0.0f; 
        
        // Virtual stiffness adjustment for the SEA (50.0 is a placeholder for normal walking)
        cmd_msg.impedance_kp = 50.0f; 

        // 4. Fire the command down to the hardware
        command_pub_->publish(cmd_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "[%s Ankle] Target Ang: %.3frad -> Commanded Len: %.4fm", 
                     prefix_.c_str(), target_rad, required_length);
    }

    rclcpp::Publisher<custom_interfaces::msg::ActuatorCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnkleIKNode>());
    rclcpp::shutdown();
    return 0;
}