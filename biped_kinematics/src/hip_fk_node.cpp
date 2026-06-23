#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/msg/actuator_state.hpp>
#include "biped_kinematics/hip_3upr.hpp"

class HipFKNode : public rclcpp::Node {
public:
    HipFKNode() : Node("hip_fk_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        std::string state_topic = "/" + prefix_ + "_leg/states";

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Subscriber: Listens to the Gateway STM32 via the USB Serial Bridge
        state_sub_ = this->create_subscription<custom_interfaces::msg::ActuatorState>(
            state_topic, 10,
            std::bind(&HipFKNode::state_callback, this, std::placeholders::_1));

        // CRITICAL: Initialize the cache to the exact middle of the L16 stroke (218mm)
        current_lengths_ << 0.218, 0.218, 0.218;
        current_pose_guess_ << 0.0, 0.0, 0.0;

        RCLCPP_INFO(this->get_logger(), "Hip FK Node active for [%s] leg. USB Telemetry and Newton-Raphson solver initialized.", prefix_.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<custom_interfaces::msg::ActuatorState>::SharedPtr state_sub_;

    std::string prefix_;
    Eigen::Vector3d current_lengths_;
    Eigen::Vector3d current_pose_guess_;

    void state_callback(const custom_interfaces::msg::ActuatorState::SharedPtr msg) {
        
        // 1. Establish the valid Hardware IDs for this specific leg
        int expected_min_id = (prefix_ == "left") ? 1 : 4;
        int expected_max_id = (prefix_ == "left") ? 3 : 6;

        // HARDWARE SAFETY FILTER: Ignore the ankle and the opposite leg
        if (msg->joint_id < expected_min_id || msg->joint_id > expected_max_id) return;

        // Convert the Hardware ID (e.g., 4, 5, 6) down to an Array Index (0, 1, 2)
        int index = msg->joint_id - expected_min_id;

        // 2. Hardware Safety Check on incoming sensor data
        double reported_length = msg->motor_length;
        if (reported_length < biped_kinematics::hip_3upr::L_MIN) reported_length = biped_kinematics::hip_3upr::L_MIN;
        if (reported_length > biped_kinematics::hip_3upr::L_MAX) reported_length = biped_kinematics::hip_3upr::L_MAX;

        // 3. Update the specific leg in our cache
        current_lengths_(index) = reported_length;

        // 4. Run the Newton-Raphson Solver
        Eigen::Vector3d solved_pose = biped_kinematics::hip_3upr::calculate_FK(
            current_lengths_, current_pose_guess_);

        current_pose_guess_ = solved_pose;

        // 5. Package and publish the Virtual Joint States for RViz
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        
        joint_msg.name.push_back(prefix_ + "_hip_z_joint");
        joint_msg.name.push_back(prefix_ + "_hip_roll_joint");
        joint_msg.name.push_back(prefix_ + "_hip_pitch_joint");
        
        joint_msg.position.push_back(solved_pose(0)); // Z translation
        joint_msg.position.push_back(solved_pose(1)); // Roll
        joint_msg.position.push_back(solved_pose(2)); // Pitch

        joint_pub_->publish(joint_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HipFKNode>());
    rclcpp::shutdown();
    return 0;
}