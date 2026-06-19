#ifndef BIPED_KINEMATICS_ANKLE_MATH_HPP
#define BIPED_KINEMATICS_ANKLE_MATH_HPP

#include <cmath>
#include <algorithm>

namespace biped_kinematics {

// --- CAD GEOMETRY CONSTANTS ---
const double L_lever = 0.06025;       // Your stated 45mm lever arm (meters)
const double L_mount = 0.31875;      // Distance from ankle pivot to top motor mount (meters)
const double ANKLE_OFFSET = 1.8443; // Internal angle when foot is flat at 0 degrees (radians)

/**
 INVERSE KINEMATICS (Brain -> Motor)
 Calculates the required SEA ballscrew length using the Law of Cosines.
 Equation: c = sqrt(a^2 + b^2 - 2ab * cos(C))
 */
inline double calculate_required_actuator_length(double target_ankle_angle_rad) {
    // 1. Calculate the internal triangle angle (gamma)
    double gamma = target_ankle_angle_rad + ANKLE_OFFSET;
    
    // 2. Apply Law of Cosines
    double length_squared = (L_lever * L_lever) + (L_mount * L_mount) - 
                            (2 * L_lever * L_mount * std::cos(gamma));
                           
    return std::sqrt(length_squared);
}
/*
 * FORWARD KINEMATICS (Motor -> Brain)
 * Calculates the rigid ankle angle based on the STM32's reported ballscrew length.
 * Equation: C = acos((a^2 + b^2 - c^2) / 2ab)
 */
inline double calculate_rigid_ankle_angle(double current_actuator_length) {
    double numerator = (L_lever * L_lever) + (L_mount * L_mount) - 
                       (current_actuator_length * current_actuator_length);
    double denominator = 2 * L_lever * L_mount;
    
    // 1. Protect against math domain errors from sensor noise (domain is [-1, 1])
    double cos_gamma = numerator / denominator;
    cos_gamma = std::clamp(cos_gamma, -1.0, 1.0); 
    
    // 2. Apply Inverse Law of Cosines
    double gamma = std::acos(cos_gamma);
    
    // 3. Remove the CAD offset to get the true robot joint angle
    return gamma - ANKLE_OFFSET;
}
} // namespace biped_kinematics

#endif 