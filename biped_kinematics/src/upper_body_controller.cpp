#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/msg/actuator_command.hpp>

class UpperBodyController : public rclcpp::Node {
public:
    UpperBodyController() : Node("upper_body_controller") {
        
        // Publisher to the USB Gateway STM32
        command_pub_ = this->create_publisher<custom_interfaces::msg::ActuatorCommand>(
            "/upper_body/commands", 10);

        // Subscriber to the Nav2 Waypoint Task Executor
        target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/upper_body/targets", 10,
            std::bind(&UpperBodyController::target_callback, this, std::placeholders::_1));

        // Subscriber to the LQR Master Clock
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10,
            std::bind(&UpperBodyController::gait_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Upper Body Controller Online. Ready for Nav2 Waypoint tasks.");
    }

private:
    int current_gait_phase_ = 0; 
    
    // THE TASK BUFFER: Stores the Nav2 command until the legs are safely planted
    sensor_msgs::msg::JointState::SharedPtr pending_task_ = nullptr;

    // ----------------------------------------------------
    // 1. The Callbacks
    // ----------------------------------------------------

    void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Nav2 has arrived at a waypoint and published a task!
        pending_task_ = msg;
        
        if (current_gait_phase_ != 0) {
            RCLCPP_INFO(this->get_logger(), "Task buffered. Waiting for legs to enter Double Support...");
        } else {
            // We are already stopped. Execute immediately.
            execute_pending_task();
        }
    }

    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;

        // The moment the clock strikes 0, check if Nav2 left us a job to do
        if (current_gait_phase_ == 0 && pending_task_ != nullptr) {
            RCLCPP_INFO(this->get_logger(), "Double Support achieved. Executing buffered Nav2 task.");
            execute_pending_task();
        }
    }

    // ----------------------------------------------------
    // 2. Hardware Execution & Routing
    // ----------------------------------------------------

    void execute_pending_task() {
        if (pending_task_ == nullptr) return;

        for (size_t i = 0; i < pending_task_->name.size(); i++) {
            auto cmd_msg = custom_interfaces::msg::ActuatorCommand();
            
            std::string joint_name = pending_task_->name[i];
            cmd_msg.joint_id = map_name_to_hardware_id(joint_name); 

            if (cmd_msg.joint_id == 99) {
                RCLCPP_WARN(this->get_logger(), "Unknown joint: %s. Skipping.", joint_name.c_str());
                continue;
            }

            // Route the data based on simple motor types
            if (cmd_msg.joint_id == 30) {
                // DC MOTOR (Continuous Wrist Rotation)
                cmd_msg.target_length = 0.0f; 
                if (pending_task_->velocity.size() > i) {
                    cmd_msg.target_velocity = static_cast<float>(pending_task_->velocity[i]);
                }
                cmd_msg.impedance_kp = 0.0f;  
            } 
            else {
                // BASIC PWM SERVOS (Neck, Shoulders, Gripper)
                // Pass the requested angle into the target_length field
                cmd_msg.target_length = static_cast<float>(pending_task_->position[i]); 
                
                // Pure positional servos ignore velocity and impedance
                cmd_msg.target_velocity = 0.0f;
                cmd_msg.impedance_kp = 0.0f; 
            }

            // Send to the Gateway STM32 via USB
            command_pub_->publish(cmd_msg);
        }

        // Wipe the buffer clean so we don't repeat the movement
        pending_task_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "Task complete. Awaiting next Nav2 Waypoint.");
    }

    // Maps the human-readable names to the Hardware Routing IDs for the Gateway STM32
    int map_name_to_hardware_id(const std::string& name) {
        
        // Basic Servos
        if (name == "neck_pan") return 20;
        if (name == "shoulder_pitch") return 21;
        if (name == "gripper_servo")  return 31; 
        
        // Custom End-Effector Hardware
        if (name == "wrist_dc_motor") return 30; 
        
        return 99; 
    }

    rclcpp::Publisher<custom_interfaces::msg::ActuatorCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UpperBodyController>());
    rclcpp::shutdown();
    return 0;
}