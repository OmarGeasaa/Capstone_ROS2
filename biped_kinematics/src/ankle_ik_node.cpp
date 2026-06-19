#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <custom_interfaces/msg/actuator_command.hpp>
#include "biped_kinematics/ankle_math.hpp"

class AnkleIKNode : public rclcpp::Node {
public:
    AnkleIKNode() : Node("ankle_ik_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        std::string command_topic = "/" + prefix_ + "_leg/commands";
        std::string target_topic = "/walking_pattern/" + prefix_ + "_ankle_pitch";

        command_pub_ = this->create_publisher<custom_interfaces::msg::ActuatorCommand>(
            command_topic, 10);

        // Subscribes to the Swing trajectory target
        target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            target_topic, 10,
            std::bind(&AnkleIKNode::target_callback, this, std::placeholders::_1));

        // Subscribes to the Master Clock
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10,
            std::bind(&AnkleIKNode::gait_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ankle IK Node operational for [%s] leg.", prefix_.c_str());
    }

private:
    std::string prefix_;
    int current_gait_phase_ = 0; // Defaults to Double Support

    // Master Clock Updater
    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;
    }

    // Identity Translation Logic
    bool is_swing() {
        if (prefix_ == "left" && current_gait_phase_ == 1) return true;
        if (prefix_ == "right" && current_gait_phase_ == 2) return true;
        return false;
    }

    void target_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        double target_rad;
        float current_impedance;

        if (is_swing()) {
            // SWING PHASE: Track the cubic spline to clear the floor, make the SEA stiff
            target_rad = std::max(-0.1745, std::min(0.2618, (double)msg->data));
            current_impedance = 80.0f; 
        } else {
            // STANCE PHASE: Ignore the swing trajectory. Lock flat to the floor, soften SEA for impact
            target_rad = 0.0;
            current_impedance = 20.0f;
        }

        // Solve the triangle for distance B-C
        double required_length = biped_kinematics::calculate_required_actuator_length(target_rad);

        // Prepare the command for the USB Serial Bridge
        auto cmd_msg = custom_interfaces::msg::ActuatorCommand();
        
        // The Gateway STM32 uses this ID to figure out where to route the signal
        cmd_msg.joint_id = 0; 
        
        cmd_msg.target_length = static_cast<float>(required_length);
        cmd_msg.target_velocity = 0.0f; 
        cmd_msg.impedance_kp = current_impedance; 

        // Fire the command to the micro-ROS agent (which transmits it over the USB cable)
        command_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<custom_interfaces::msg::ActuatorCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnkleIKNode>());
    rclcpp::shutdown();
    return 0;
}