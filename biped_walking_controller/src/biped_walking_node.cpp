#include <memory>
#include <chrono>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

// Include your custom math engines
#include "biped_walking_controller/footstep_planner.hpp"
#include "biped_walking_controller/zmp_generator.hpp"

using namespace std::chrono_literals;

class BipedWalkingNode : public rclcpp::Node {
private:
    // ROS 2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pelvis_pub_;
    rclcpp::TimerBase::SharedPtr timer_100hz_;

    // Core Math Modules
    FootstepPlanner footstep_planner_;
    ZmpGenerator zmp_generator_;

    // Thread Safety for Async Nav2 Data
    std::mutex cmd_vel_mutex_;
    double current_cmd_vx_ = 0.0;
    double current_cmd_vy_ = 0.0;

    // 1. The Asynchronous Callback (Triggered whenever Nav2 speaks)
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        current_cmd_vx_ = msg->linear.x;
        current_cmd_vy_ = msg->linear.y;
    }

    // 2. The Synchronous Hardware Loop (Fires exactly every 10ms)
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

        // C. Run the 3D-LIPM LQR Physics Engine
        PelvisTarget target = zmp_generator_.update(
            current_ref_x, current_ref_y, 
            preview_x, preview_y
        );

        // D. Publish the exact required pelvis coordinate to biped_kinematics
        auto msg = geometry_msgs::msg::Point();
        msg.x = target.x;
        msg.y = target.y;
        msg.z = 0.6; // Passing your Zc height for the IK node's reference
        
        pelvis_pub_->publish(msg);
    }

public:
    BipedWalkingNode() : Node("biped_walking_controller") {
        // Subscribe to Nav2
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&BipedWalkingNode::cmdVelCallback, this, std::placeholders::_1));

        // Publish to your Kinematics Node
        pelvis_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_pelvis_pose", 10);

        // Lock the physics engine to a strict 100Hz
        timer_100hz_ = this->create_wall_timer(
            10ms, std::bind(&BipedWalkingNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Biped Walking Controller Online. Running at 100Hz.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BipedWalkingNode>());
    rclcpp::shutdown();
    return 0;
}