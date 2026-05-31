#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/msg/actuator_state.hpp>
#include "biped_kinematics/hip_3upr.hpp"

class HipFKNode : public rclcpp::Node {
public:
    HipFKNode() : Node("hip_fk_node") {
        // 1. Get the prefix parameter (defaults to "left")
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        // 2. Dynamically build the subscriber topic
        std::string state_topic = "/" + prefix_ + "_leg/states";

        // Publisher: Broadcasts the solved Z, Roll, Pitch to RViz and Nav2
        // (Note: /joint_states is a global ROS 2 topic, so it doesn't get a prefix)
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Subscriber: Listens to the raw STM32 ADC potentiometer readings dynamically
        state_sub_ = this->create_subscription<custom_interfaces::msg::ActuatorState>(
            state_topic, 10,
            std::bind(&HipFKNode::state_callback, this, std::placeholders::_1));

        // CRITICAL: Initialize the cache to the exact middle of the L16 stroke (218mm)
        // If we initialize to 0.0, the math will instantly explode on boot.
        current_lengths_ << 0.218, 0.218, 0.218;
        
        // Initialize our solver's guess to standing perfectly straight
        current_pose_guess_ << 0.0, 0.0, 0.0;

        RCLCPP_INFO(this->get_logger(), "Hip FK Node active for [%s] leg. Newton-Raphson solver initialized.", prefix_.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<custom_interfaces::msg::ActuatorState>::SharedPtr state_sub_;

    // Store the prefix here so the callback below can access it!
    std::string prefix_;

    // The State Caches
    Eigen::Vector3d current_lengths_;
    Eigen::Vector3d current_pose_guess_;

    void state_callback(const custom_interfaces::msg::ActuatorState::SharedPtr msg) {
        // 1. Filter and Map the Data
        // Hip Actuators are IDs 1, 2, and 3. Ignore the ankle (ID 0).
        if (msg->joint_id < 1 || msg->joint_id > 3) return;

        // Convert Joint ID (1-3) to Array Index (0-2)
        int index = msg->joint_id - 1;

        // 2. Hardware Safety Check on incoming sensor data
        // Even if the ADC sends electrical noise, we cannot let impossible numbers crash the solver
        double reported_length = msg->motor_length;
        if (reported_length < biped_kinematics::hip_3upr::L_MIN) reported_length = biped_kinematics::hip_3upr::L_MIN;
        if (reported_length > biped_kinematics::hip_3upr::L_MAX) reported_length = biped_kinematics::hip_3upr::L_MAX;

        // 3. Update the specific leg in our cache
        current_lengths_(index) = reported_length;

        // 4. Run the Newton-Raphson Solver
        // We pass in our 'current_pose_guess_' so the solver starts looking right where it left off
        Eigen::Vector3d solved_pose = biped_kinematics::hip_3upr::calculate_FK(
            current_lengths_, current_pose_guess_);

        // Update the guess memory for the next millisecond
        current_pose_guess_ = solved_pose;

        // 5. Package and publish the Virtual Joint States for RViz
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        
        // 6. DYNAMIC Joint Names! 
        // These will evaluate to "left_hip_z_joint" or "right_hip_z_joint" automatically
        joint_msg.name.push_back(prefix_ + "_hip_z_joint");
        joint_msg.name.push_back(prefix_ + "_hip_roll_joint");
        joint_msg.name.push_back(prefix_ + "_hip_pitch_joint");
        
        // Solved Pose maps to [Z, Roll, Pitch] based on our math file
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