#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <algorithm>

class SwingTrajectoryGenerator : public rclcpp::Node {
public:
    SwingTrajectoryGenerator() : Node("swing_trajectory_generator") {
        
        // 1. Publishers to the Kinematic Spinal Cord
        swing_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/swing_foot_pose", 10);
        left_ankle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/walking_pattern/left_ankle_pitch", 10);
        right_ankle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/walking_pattern/right_ankle_pitch", 10);

        // 2. Subscribers from the LQR Brain
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10, std::bind(&SwingTrajectoryGenerator::gait_callback, this, std::placeholders::_1));
            
        footprint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/upcoming_footprints", 10, std::bind(&SwingTrajectoryGenerator::footprint_callback, this, std::placeholders::_1));

        // 3. The 100Hz Real-Time Loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&SwingTrajectoryGenerator::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Swing Trajectory Generator Online. 5th-Order Polynomials active.");
    }

private:
    int current_gait_phase_ = 0;
    int previous_gait_phase_ = 0;
    
    // Timing Variables
    double swing_timer_ = 0.0;
    const double T_SWING = 2.83; // Total swing time based on your 3.33s step (0.5s double support)

    // Trajectory Boundaries
    double start_x_ = 0.0, start_y_ = 0.0;
    double target_x_ = 0.0, target_y_ = 0.0;
    
    // Hardware Constraints (Based on your physical limits)
    const double PEAK_Z_CLEARANCE = 0.05; // 5cm lift off the ground
    const double STANCE_PELVIS_Z = 0.60;  // Normal standing height
    const double MAX_DORSIFLEXION = 0.2094; // 12 degrees (Safely below your 15 deg hard-stop)

    // ----------------------------------------------------
    // MATHEMATICAL ENGINES
    // ----------------------------------------------------

    // Standard 5th-Order Minimum Jerk (Used for X and Y Translation)
    // Guarantees V=0 and A=0 at start and end.
    double min_jerk_spline(double t, double T, double start, double end) {
        if (t <= 0.0) return start;
        if (t >= T) return end;
        
        double tau = t / T; // Normalize time (0.0 to 1.0)
        double S = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        
        return start + (end - start) * S;
    }

    // Bell Curve Minimum Jerk (Used for Z clearance and Ankle pitch)
    // Starts at 0, peaks smoothly in the middle, lands at 0.
    double min_jerk_bump(double t, double T, double peak_val) {
        if (t <= 0.0 || t >= T) return 0.0;
        
        double tau = t / T;
        // 64 * tau^3 * (1-tau)^3 creates a perfect bell curve that peaks at exactly 1.0 when tau=0.5
        return peak_val * 64.0 * std::pow(tau, 3) * std::pow(1.0 - tau, 3);
    }

    // ----------------------------------------------------
    // CALLBACKS
    // ----------------------------------------------------

    void footprint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            target_x_ = msg->data[0];
            target_y_ = msg->data[1];
        }
    }

    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;

        // Detect the EXACT moment a foot lifts off the ground
        if (current_gait_phase_ != 0 && previous_gait_phase_ == 0) {
            swing_timer_ = 0.0; // Reset the flight clock
            
            // In a fully closed-loop system, start_x_ would come from Odometry.
            // For now, we assume we are stepping from 0 offset to the new target.
            start_x_ = 0.0; 
            start_y_ = 0.0; 
        }
        previous_gait_phase_ = current_gait_phase_;
    }

    // ----------------------------------------------------
    // 100Hz REAL-TIME GENERATION
    // ----------------------------------------------------

    void timer_callback() {
        if (current_gait_phase_ == 0) return; // Do nothing if both feet are on the floor

        // 1. Advance the flight clock
        swing_timer_ += 0.01; // 10ms loop
        if (swing_timer_ > T_SWING) swing_timer_ = T_SWING; // Clamp at landing

        // ==========================================
        // 2. HIP POSITION TRAJECTORY (3-UPR)
        // ==========================================
        double current_x = min_jerk_spline(swing_timer_, T_SWING, start_x_, target_x_);
        double current_y = min_jerk_spline(swing_timer_, T_SWING, start_y_, target_y_);
        
        // Z Clearance: The 3-UPR must contract by this amount
        double z_lift = min_jerk_bump(swing_timer_, T_SWING, PEAK_Z_CLEARANCE);
        
        // Subtract lift from the stance height to cause the hip IK to contract
        double current_z = STANCE_PELVIS_Z - z_lift; 

        auto hip_msg = geometry_msgs::msg::Point();
        hip_msg.x = current_x;
        hip_msg.y = current_y;
        hip_msg.z = current_z; 
        swing_pose_pub_->publish(hip_msg);

        // ==========================================
        // 3. ANKLE ORIENTATION TRAJECTORY (SEA)
        // ==========================================
        // Pitch the toe up safely to MAX_DORSIFLEXION at mid-swing to clear the floor
        double current_pitch = min_jerk_bump(swing_timer_, T_SWING, MAX_DORSIFLEXION);
        
        auto ankle_msg = std_msgs::msg::Float32();
        ankle_msg.data = static_cast<float>(current_pitch);

        // Route the pitch angle to the correct flying leg
        if (current_gait_phase_ == 1) { // Left leg is swinging
            left_ankle_pub_->publish(ankle_msg);
        } else if (current_gait_phase_ == 2) { // Right leg is swinging
            right_ankle_pub_->publish(ankle_msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr swing_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_ankle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_ankle_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr footprint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingTrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}