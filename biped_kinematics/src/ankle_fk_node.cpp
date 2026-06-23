#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/msg/actuator_state.hpp>
#include "biped_kinematics/ankle_math.hpp"

class AnkleFKNode : public rclcpp::Node {
public:
    AnkleFKNode() : Node("ankle_fk_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        std::string state_topic = "/" + prefix_ + "_leg/states";

        // Publisher: Broadcasts standard Joint States for RViz and TF Tree
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Subscriber: Listens to the Gateway STM32 via the USB Serial Bridge
        state_sub_ = this->create_subscription<custom_interfaces::msg::ActuatorState>(
            state_topic, 10,
            std::bind(&AnkleFKNode::state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ankle FK Node Started for [%s] leg. Receiving USB Gateway telemetry.", prefix_.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<custom_interfaces::msg::ActuatorState>::SharedPtr state_sub_;
    
    std::string prefix_;

    // Determine expected Hardware ID based on the leg
    int get_hardware_id() {
        return (prefix_ == "left") ? 0 : 10;
    }

    void state_callback(const custom_interfaces::msg::ActuatorState::SharedPtr msg) {
        
        // HARDWARE SAFETY FILTER: Ignore data from the wrong leg or the hips
        if (msg->joint_id != get_hardware_id()) return;

        // 1. Execute the pure Math to find the Rigid Angle
        double rigid_angle_rad = biped_kinematics::calculate_rigid_ankle_angle(msg->motor_length);

        // 2. Calculate actual spring deflection
        double spring_deflection = rigid_angle_rad - msg->external_angle;

        // 3. Package and publish the Standard ROS 2 JointState
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        
        joint_msg.name.push_back(prefix_ + "_ankle_joint"); 
        joint_msg.position.push_back(msg->external_angle); 
        joint_msg.effort.push_back(msg->actual_current);

        joint_pub_->publish(joint_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "[%s Ankle] Motor Len: %.4fm -> Rigid Ang: %.3frad | Ext Ang: %.3frad | Deflection: %.3frad", 
            prefix_.c_str(), msg->motor_length, rigid_angle_rad, msg->external_angle, spring_deflection);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnkleFKNode>());
    rclcpp::shutdown();
    return 0;
}