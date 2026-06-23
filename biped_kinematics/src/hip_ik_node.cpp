#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <custom_interfaces/msg/actuator_command.hpp>
#include "biped_kinematics/hip_3upr.hpp"
#include <cmath>

class HipIKNode : public rclcpp::Node {
public:
    HipIKNode() : Node("hip_ik_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        std::string command_topic = "/" + prefix_ + "_leg/commands";
        
        // Publisher: Sends the actuator commands to the micro-ROS Agent (USB Serial Bridge)
        command_pub_ = this->create_publisher<custom_interfaces::msg::ActuatorCommand>(
            command_topic, 10);

        // Subscriber 1: The LQR Balancing Brain
        lqr_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_pelvis_pose", 10,
            std::bind(&HipIKNode::lqr_callback, this, std::placeholders::_1));

        // Subscriber 2: The Flight Path Generator
        swing_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/swing_foot_pose", 10,
            std::bind(&HipIKNode::swing_callback, this, std::placeholders::_1));

        // Subscriber 3: The Master Clock
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10,
            std::bind(&HipIKNode::gait_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Hip IK Node active for [%s] leg. Routing data via USB Gateway.", prefix_.c_str());
    }

private:
    std::string prefix_;
    int current_gait_phase_ = 0;

    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;
    }

    bool is_stance() {
        if (current_gait_phase_ == 0) return true; // Double Support
        if (prefix_ == "left" && current_gait_phase_ == 2) return true;
        if (prefix_ == "right" && current_gait_phase_ == 1) return true;
        return false;
    }

    bool is_swing() {
        if (prefix_ == "left" && current_gait_phase_ == 1) return true;
        if (prefix_ == "right" && current_gait_phase_ == 2) return true;
        return false;
    }

    // Assigns distinct hardware IDs so the Gateway STM32 knows which side of the robot to talk to
    int get_hardware_id(int actuator_index) {
        if (prefix_ == "left") {
            return actuator_index + 1; // Left Hip 3-UPR uses IDs 1, 2, 3
        } else {
            return actuator_index + 4; // Right Hip 3-UPR uses IDs 4, 5, 6
        }
    }

    // Core Execution Function (Keeps your hardware clamps in one place)
    void execute_3upr_kinematics(double target_z, double target_roll, double target_pitch) {
        Eigen::Vector3d required_lengths = biped_kinematics::hip_3upr::calculate_IK(
            target_z, target_roll, target_pitch);

        for (int i = 0; i < 3; i++) {
            double safe_length = required_lengths(i);

            if (safe_length < biped_kinematics::hip_3upr::L_MIN) {
                safe_length = biped_kinematics::hip_3upr::L_MIN;
            } else if (safe_length > biped_kinematics::hip_3upr::L_MAX) {
                safe_length = biped_kinematics::hip_3upr::L_MAX;
            }

            auto cmd_msg = custom_interfaces::msg::ActuatorCommand();
            
            // Generate the unique Hardware ID for this specific Actuonix L16
            cmd_msg.joint_id = get_hardware_id(i); 
            cmd_msg.target_length = static_cast<float>(safe_length);
            
            // Explicitly zero out unused fields for memory safety over the USB bridge
            cmd_msg.target_velocity = 0.0f;
            cmd_msg.impedance_kp = 0.0f;

            // Fire the command down the USB cable to the Gateway STM32
            command_pub_->publish(cmd_msg);
        }
    }

    // ----------------------------------------------------
    // The Traffic Cops (Routing data based on phase)
    // ----------------------------------------------------

    void lqr_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!is_stance()) return; // Ignore LQR if this leg is flying!

        double lqr_x = msg->x;
        double lqr_y = msg->y;
        double target_z = msg->z; 

        // Balance Math: Convert Cartesian shift to Tilt
        double target_pitch = std::atan2(lqr_x, target_z); 
        double target_roll  = std::atan2(lqr_y, target_z);

        execute_3upr_kinematics(target_z, target_roll, target_pitch);
    }

    void swing_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!is_swing()) return; // Ignore Flight Path if this leg is balancing!

        double swing_x = msg->x;
        double swing_y = msg->y;
        double swing_z = msg->z; // This will dynamically change to lift the foot!

        // Flight Math: Convert the X/Y target into swing tilt
        // We use an assumed 0.6m pendulum length for the angle calculation, 
        // but pass the actual contracted swing_z to lift the mechanism.
        double target_pitch = std::atan2(swing_x, 0.6); 
        double target_roll  = std::atan2(swing_y, 0.6);

        execute_3upr_kinematics(swing_z, target_roll, target_pitch);
    }

    rclcpp::Publisher<custom_interfaces::msg::ActuatorCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr lqr_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr swing_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HipIKNode>());
    rclcpp::shutdown();
    return 0;
}