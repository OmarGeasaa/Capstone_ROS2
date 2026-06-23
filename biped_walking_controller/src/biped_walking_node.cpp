#include <memory>
#include <chrono>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // ADDED: For reading ankle effort

// Include your custom math engines
#include "biped_walking_controller/footstep_planner.hpp"
#include "biped_walking_controller/zmp_generator.hpp"

using namespace std::chrono_literals;

class BipedWalkingNode : public rclcpp::Node {
private:
    // ROS 2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; // ADDED: Sensor feedback
    
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pelvis_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr phase_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr footprint_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_100hz_;
    
    // Core Math Modules
    FootstepPlanner footstep_planner_;
    ZmpGenerator zmp_generator_;

    // Thread Safety for Async Nav2 Data
    std::mutex cmd_vel_mutex_;
    double current_cmd_vx_ = 0.0;
    double current_cmd_vy_ = 0.0;

    // Contact Detection Threshold (e.g., 2.5 Amps indicates a hard floor strike)
    const double IMPACT_THRESHOLD = 2.5; 

    // ------------------------------------------------------------------
    // 1. The Sensor Interrupt Callback (Fires when the ankle touches the floor)
    // ------------------------------------------------------------------
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        int current_phase = footstep_planner_.get_current_gait_phase();
        
        // If both feet are already planted, we don't care about impacts
        if (current_phase == 0) return; 

        for (size_t i = 0; i < msg->name.size(); i++) {
            std::string joint_name = msg->name[i];
            double effort = std::abs(msg->effort[i]); // Get absolute torque/current

            // Check Left Leg Impact during Left Swing (Phase 1)
            if (current_phase == 1 && joint_name == "left_ankle_joint" && effort > IMPACT_THRESHOLD) {
                RCLCPP_WARN(this->get_logger(), "EARLY CONTACT: Left foot struck the ground! Interrupting timer.");
                footstep_planner_.force_early_contact(); 
                break;
            }
            
            // Check Right Leg Impact during Right Swing (Phase 2)
            if (current_phase == 2 && joint_name == "right_ankle_joint" && effort > IMPACT_THRESHOLD) {
                RCLCPP_WARN(this->get_logger(), "EARLY CONTACT: Right foot struck the ground! Interrupting timer.");
                footstep_planner_.force_early_contact();
                break;
            }
        }
    }

    // ------------------------------------------------------------------
    // 2. The Asynchronous Callback (Triggered whenever Nav2 speaks)
    // ------------------------------------------------------------------
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        current_cmd_vx_ = msg->linear.x;
        current_cmd_vy_ = msg->linear.y;
    }

    // ------------------------------------------------------------------
    // 3. The Synchronous Hardware Loop (Fires exactly every 10ms)
    // ------------------------------------------------------------------
    void timerCallback() {
        double target_vx, target_vy;
        
        // A. Safely grab the latest velocity without stalling the timer
        {
            std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
            target_vx = current_cmd_vx_;
            target_vy = current_cmd_vy_;
        }

        // B. Generate the 1.5-second future footprint window
        double current_ref_x, current_ref_y;
        std::vector<double> preview_x, preview_y;
        
        footstep_planner_.get_preview_window(
            target_vx, target_vy, 
            current_ref_x, current_ref_y, 
            preview_x, preview_y
        );

        // Publish the Master Clock Gait Phase
        auto phase_msg = std_msgs::msg::Int32();
        phase_msg.data = footstep_planner_.get_current_gait_phase();
        phase_pub_->publish(phase_msg);

        // C. Run the 3D-LIPM LQR Physics Engine
        PelvisTarget target = zmp_generator_.update(
            current_ref_x, current_ref_y, 
            preview_x, preview_y
        );

        // D. Publish the exact required pelvis coordinate to biped_kinematics
        auto msg = geometry_msgs::msg::Point();
        msg.x = target.x;
        msg.y = target.y;
        msg.z = 0.60; // Assuming 0.6m is your STANCE_PELVIS_Z
        pelvis_pub_->publish(msg);
        
        // Publish the upcoming trajectory targets for the swing phase
        if (!preview_x.empty() && !preview_y.empty()) {
            auto footprint_msg = std_msgs::msg::Float64MultiArray();
            footprint_msg.data.push_back(preview_x.back());
            footprint_msg.data.push_back(preview_y.back());
            footprint_pub_->publish(footprint_msg);
        }
    }

public:
    BipedWalkingNode() : Node("biped_walking_controller") {
        
        // FIXED: Initialize all your publishers properly so the node doesn't crash!
        pelvis_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_pelvis_pose", 10);
        phase_pub_ = this->create_publisher<std_msgs::msg::Int32>("/gait_phase", 10);
        footprint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/upcoming_footprints", 10);

        // Subscribe to Nav2 and Sensors
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&BipedWalkingNode::cmdVelCallback, this, std::placeholders::_1));
            
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&BipedWalkingNode::jointStateCallback, this, std::placeholders::_1));

        // Lock the physics engine to a strict 100Hz
        timer_100hz_ = this->create_wall_timer(
            10ms, std::bind(&BipedWalkingNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Biped Walking Controller Online. 100Hz Physics Loop & Floor Contact Interrupts Active.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BipedWalkingNode>());
    rclcpp::shutdown();
    return 0;
}