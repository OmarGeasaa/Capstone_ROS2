#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp> // Update 1
#include <custom_interfaces/msg/actuator_command.hpp>
#include "biped_kinematics/hip_3upr.hpp"
#include <cmath> 

class HipIKNode : public rclcpp::Node {
public:
    HipIKNode() : Node("hip_ik_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        std::string prefix = this->get_parameter("leg_prefix").as_string();

        std::string command_topic = "/" + prefix + "_leg/commands";
        
        // Wired directly to the LQR brain
        std::string target_topic = "/target_pelvis_pose"; 

        command_pub_ = this->create_publisher<custom_interfaces::msg::ActuatorCommand>(
            command_topic, 10);

        // Update 3: Expecting a Point message
        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            target_topic, 10,
            std::bind(&HipIKNode::target_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Hip IK Node active for [%s] leg. 3-UPR limits enforced.", prefix.c_str());
    }

private:
    // Callback accepts a Point message
    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        
        // The Mathematical Bridge
        // Extract the Cartesian target pose from the LQR
        double lqr_x = msg->x;
        double lqr_y = msg->y;
        double target_z = msg->z; // Typically 0.6m

        // Convert LQR Cartesian (X,Y) shift into 3-UPR Tilt (Pitch, Roll)
        // Using the inverted pendulum triangle: Angle = atan(Offset / Height)
        double target_pitch = std::atan2(lqr_x, target_z); 
        double target_roll  = std::atan2(lqr_y, target_z);

        Eigen::Vector3d required_lengths = biped_kinematics::hip_3upr::calculate_IK(
            target_z, target_roll, target_pitch);

        for (int i = 0; i < 3; i++) {
            double safe_length = required_lengths(i);

            if (safe_length < biped_kinematics::hip_3upr::L_MIN) {
                RCLCPP_WARN(this->get_logger(), "Actuator %d hit MIN limit! Clamping to 0.168m", i+1);
                safe_length = biped_kinematics::hip_3upr::L_MIN;
            } else if (safe_length > biped_kinematics::hip_3upr::L_MAX) {
                RCLCPP_WARN(this->get_logger(), "Actuator %d hit MAX limit! Clamping to 0.268m", i+1);
                safe_length = biped_kinematics::hip_3upr::L_MAX;
            }

            auto cmd_msg = custom_interfaces::msg::ActuatorCommand();
            cmd_msg.joint_id = i + 1; 
            cmd_msg.target_length = static_cast<float>(safe_length);
            cmd_msg.target_velocity = 0.0f;
            cmd_msg.impedance_kp = 100.0f; 

            command_pub_->publish(cmd_msg);
        }

        RCLCPP_DEBUG(this->get_logger(), 
            "LQR [X:%.3f, Y:%.3f] -> Tilt [R:%.3f, P:%.3f] -> Len [L1:%.3f, L2:%.3f, L3:%.3f]",
            lqr_x, lqr_y, target_roll, target_pitch, 
            required_lengths(0), required_lengths(1), required_lengths(2));
    }

    rclcpp::Publisher<custom_interfaces::msg::ActuatorCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HipIKNode>());
    rclcpp::shutdown();
    return 0;
}