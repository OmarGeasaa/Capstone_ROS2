#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/msg/actuator_state.hpp>
#include "biped_kinematics/ankle_math.hpp"

class AnkleFKNode : public rclcpp::Node {
public:
    AnkleFKNode() : Node("ankle_fk_node") {
        // 1. Get the prefix parameter (defaults to "left")
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        // 2. Dynamically build the subscriber topic
        std::string state_topic = "/" + prefix_ + "_leg/states";

        // 3. Publisher: Broadcasts standard Joint States for RViz and TF Tree
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // 4. Subscriber: Listens to the STM32 via the micro-ROS CAN bus dynamically
        state_sub_ = this->create_subscription<custom_interfaces::msg::ActuatorState>(
            state_topic, 10,
            std::bind(&AnkleFKNode::state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ankle FK Node Started for [%s] leg. Translating ballscrew lengths to joint angles.", prefix_.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<custom_interfaces::msg::ActuatorState>::SharedPtr state_sub_;
    
    // Store the prefix here so the callback below can access it!
    std::string prefix_;

    void state_callback(const custom_interfaces::msg::ActuatorState::SharedPtr msg) {
        // We only care about the ankle (ID 0) in this specific node
        if (msg->joint_id != 0) return;

        // 1. Execute the pure Math to find the Rigid Angle
        double rigid_angle_rad = biped_kinematics::calculate_rigid_ankle_angle(msg->motor_length);

        // 2. Calculate actual spring deflection (Rigid Angle vs External Encoder Angle)
        double spring_deflection = rigid_angle_rad - msg->external_angle;    // for debugging

        // 3. Package and publish the Standard ROS 2 JointState
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        
        // 4. DYNAMIC Joint Name!
        // Will evaluate to "left_ankle_joint" or "right_ankle_joint" based on the launch file
        joint_msg.name.push_back(prefix_ + "_ankle_joint"); 
        
        // We publish the external angle because that's where the physical foot actually is
        joint_msg.position.push_back(msg->external_angle); 
        
        // We also publish the torque current so Nav2 knows how hard the foot is pushing
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