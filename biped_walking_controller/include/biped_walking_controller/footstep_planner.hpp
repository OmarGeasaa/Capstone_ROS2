#pragma once

#include <vector>

class FootstepPlanner {
private:
    // 1. Hardware & Gait Parameters (Matching your MATLAB/SEA math)
    const double T_STEP = 3.33;      // Step duration in seconds
    const double TS = 0.01;          // 100Hz Control Loop timer
    const double STEP_WIDTH = 0.20;  // 20cm stance width
    const int PREVIEW_TICKS = 150;   // 1.5 second LQR preview window (1.5 / 0.01)

    // 2. Internal State Tracking
    double current_foot_x;
    double current_foot_y;
    int current_leg;                 // 1 for Left, -1 for Right
    double step_timer;               // Tracks where we are in the 3.33s cycle

public:
    FootstepPlanner() {
        // Robot boots up standing perfectly still at the origin
        current_foot_x = 0.0;
        current_foot_y = 0.0;
        current_leg = 1;             // Start by planning the first step with the Left leg
        step_timer = 0.0;
    }

    // 3. The 100Hz Dynamic Array Generator
    // This perfectly mimics the plan_footsteps.m function, but runs infinitely in real-time
    void get_preview_window(
        double cmd_vel_x, 
        double cmd_vel_y, 
        double& out_current_ref_x, 
        double& out_current_ref_y,
        std::vector<double>& out_preview_x, 
        std::vector<double>& out_preview_y) 
    {
        // Clear the buffers for the fresh 100Hz calculation
        out_preview_x.clear();
        out_preview_y.clear();
        out_preview_x.reserve(PREVIEW_TICKS);
        out_preview_y.reserve(PREVIEW_TICKS);
        
        // A. Advance the physical timer by one clock tick (10ms)
        step_timer += TS;

        // B. If 3.33 seconds have passed, "plant" the foot physically and swap legs
        if (step_timer >= T_STEP) {
            step_timer = 0.0;
            current_foot_x += cmd_vel_x * T_STEP;
            current_foot_y += cmd_vel_y * T_STEP;
            current_leg *= -1;
        }

        // C. Output the CURRENT ZMP target (Where the planted foot is right now)
        out_current_ref_x = current_foot_x;
        out_current_ref_y = current_foot_y + (current_leg * (STEP_WIDTH / 2.0));

        // D. Project the FUTURE 1.5 seconds (The Sliding Window)
        double sim_timer = step_timer;
        double sim_foot_x = current_foot_x;
        double sim_foot_y = current_foot_y;
        int sim_leg = current_leg;

        for (int i = 0; i < PREVIEW_TICKS; i++) {
            sim_timer += TS;
            
            // If the simulated future crosses a 3.33s boundary, shift the math target
            if (sim_timer >= T_STEP) {
                sim_timer -= T_STEP;
                sim_foot_x += cmd_vel_x * T_STEP;
                sim_foot_y += cmd_vel_y * T_STEP;
                sim_leg *= -1;
            }
            
            double future_ref_x = sim_foot_x;
            double future_ref_y = sim_foot_y + (sim_leg * (STEP_WIDTH / 2.0));
            
            // Push the future coordinates into the LQR preview arrays
            out_preview_x.push_back(future_ref_x);
            out_preview_y.push_back(future_ref_y);
        }
    }
            // Returns 0 (Double Support), 1 (Right Stance), or 2 (Left Stance)
    int get_current_gait_phase() const {
        // Define double support as the first 15% of the 3.33s step
        double double_support_time = T_STEP * 0.15;

        if (step_timer < double_support_time) {
            return 0; // Double Support
        } 
        else if (current_leg == 1) {
            // Left leg is swinging, meaning Right leg is the stance pillar
            return 1; 
        } 
        else {
            // Right leg is swinging, meaning Left leg is the stance pillar
            return 2; 
        }
    }
};