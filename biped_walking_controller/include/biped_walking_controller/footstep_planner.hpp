#pragma once

#include <vector>
#include <cmath>

class FootstepPlanner {
private:
    // 1. Hardware & Gait Parameters
    const double T_STEP = 3.33;      
    const double TS = 0.01;          
    const double STEP_WIDTH = 0.20;  
    const int PREVIEW_TICKS = 150;   
    const double DSP_TIME = 0.50;    // 0.5s Double Support Phase (15% of step)

    // 2. Internal State Tracking
    double current_foot_x;
    double current_foot_y;
    int current_leg;                 
    double step_timer;               

    // 3. Previous Target Tracking (Crucial for the ZMP slide)
    double prev_zmp_x;
    double prev_zmp_y;

    // Helper Function: Generates the smooth ZMP shift during Double Support
    void calculate_zmp_target(double t, double curr_x, double curr_y, int curr_leg, double p_zmp_x, double p_zmp_y, double& out_x, double& out_y) {
        double current_zmp_x = curr_x;
        double current_zmp_y = curr_y + (curr_leg * (STEP_WIDTH / 2.0));
        
        if (t < DSP_TIME) {
            // We are in Double Support. Slide smoothly from the previous ZMP to the new ZMP.
            double ratio = t / DSP_TIME;
            
            // 3rd-Order Smoothing Spline: 3t^2 - 2t^3 (Prevents harsh acceleration at the start/end of the shift)
            double S = 3.0 * std::pow(ratio, 2) - 2.0 * std::pow(ratio, 3);
            
            out_x = p_zmp_x + S * (current_zmp_x - p_zmp_x);
            out_y = p_zmp_y + S * (current_zmp_y - p_zmp_y);
        } else {
            // Single Support: The target is locked directly over the stance foot
            out_x = current_zmp_x;
            out_y = current_zmp_y;
        }
    }

public:
    FootstepPlanner() {
        current_foot_x = 0.0;
        current_foot_y = 0.0;
        current_leg = 1;             
        step_timer = 0.0;
        
        // Robot boots up with the ZMP perfectly centered between both feet (0, 0)
        prev_zmp_x = 0.0;
        prev_zmp_y = 0.0; 
    }

    // Instantly force Double Support when the SEA detects a floor strike
    void force_early_contact() {
        step_timer = 0.0; 
        
        // Lock in the current locations to start the next slide
        prev_zmp_x = current_foot_x;
        prev_zmp_y = current_foot_y + (current_leg * (STEP_WIDTH / 2.0));
        
        current_leg *= -1; // Swap legs immediately
    }

    void get_preview_window(
        double cmd_vel_x, 
        double cmd_vel_y, 
        double& out_current_ref_x, 
        double& out_current_ref_y,
        std::vector<double>& out_preview_x, 
        std::vector<double>& out_preview_y) 
    {
        out_preview_x.clear();
        out_preview_y.clear();
        out_preview_x.reserve(PREVIEW_TICKS);
        out_preview_y.reserve(PREVIEW_TICKS);
        
        step_timer += TS;

        // Physical Step Boundary
        if (step_timer >= T_STEP) {
            step_timer -= T_STEP;
            
            // Save the exact ZMP location right before the step ended
            prev_zmp_x = current_foot_x;
            prev_zmp_y = current_foot_y + (current_leg * (STEP_WIDTH / 2.0));

            current_foot_x += cmd_vel_x * T_STEP;
            current_foot_y += cmd_vel_y * T_STEP;
            current_leg *= -1;
        }

        // Output CURRENT smoothed ZMP target
        calculate_zmp_target(step_timer, current_foot_x, current_foot_y, current_leg, prev_zmp_x, prev_zmp_y, out_current_ref_x, out_current_ref_y);

        // Project the FUTURE 1.5 seconds (The Sliding Window)
        double sim_timer = step_timer;
        double sim_foot_x = current_foot_x;
        double sim_foot_y = current_foot_y;
        int sim_leg = current_leg;
        
        double sim_prev_zmp_x = prev_zmp_x;
        double sim_prev_zmp_y = prev_zmp_y;

        for (int i = 0; i < PREVIEW_TICKS; i++) {
            sim_timer += TS;
            
            if (sim_timer >= T_STEP) {
                sim_timer -= T_STEP;
                
                // Track the simulated previous ZMP for future shifts
                sim_prev_zmp_x = sim_foot_x;
                sim_prev_zmp_y = sim_foot_y + (sim_leg * (STEP_WIDTH / 2.0));

                sim_foot_x += cmd_vel_x * T_STEP;
                sim_foot_y += cmd_vel_y * T_STEP;
                sim_leg *= -1;
            }
            
            double future_ref_x, future_ref_y;
            calculate_zmp_target(sim_timer, sim_foot_x, sim_foot_y, sim_leg, sim_prev_zmp_x, sim_prev_zmp_y, future_ref_x, future_ref_y);
            
            out_preview_x.push_back(future_ref_x);
            out_preview_y.push_back(future_ref_y);
        }
    }

    int get_current_gait_phase() const {
        if (step_timer < DSP_TIME) return 0; // Double Support
        if (current_leg == 1) return 1;      // Left leg swinging
        return 2;                            // Right leg swinging
    }
};