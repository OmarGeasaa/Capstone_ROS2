#ifndef BIPED_KINEMATICS_HIP_3UPR_HPP
#define BIPED_KINEMATICS_HIP_3UPR_HPP

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace biped_kinematics {
namespace hip_3upr {

// --- 1. Define Robot Geometry Constants ---
const double Rb = 83.14 / 1000.0; // Rb ---> base circle radius 
const double Rp = 30.0 / 1000.0;  // Rp ---> platform circle radius 
const double BASE_Z = 0.34;
const double PI_2_3 = 2.094395102; // 2*pi/3
const double PI_4_3 = 4.188790204; // 4*pi/3

// Hardware limits
const double L_MIN = 0.168;
const double L_MAX = 0.268;

// Helper function to get Base (A) and Platform (B) coordinate vectors
inline void get_geometry(Eigen::Vector3d A[3], Eigen::Vector3d B[3]) {
    A[0] << Rb, 0.0, BASE_Z;
    A[1] << Rb * std::cos(PI_2_3), Rb * std::sin(PI_2_3), BASE_Z;
    A[2] << Rb * std::cos(PI_4_3), Rb * std::sin(PI_4_3), BASE_Z;

    B[0] << Rp, 0.0, 0.0;
    B[1] << Rp * std::cos(PI_2_3), Rp * std::sin(PI_2_3), 0.0;
    B[2] << Rp * std::cos(PI_4_3), Rp * std::sin(PI_4_3), 0.0;
}

// --- 2. Inverse Kinematics ---
inline Eigen::Vector3d calculate_IK(double z, double roll, double pitch) {
    Eigen::Vector3d A[3], B[3];
    get_geometry(A, B);

    Eigen::Vector3d P(0.0, 0.0, z);

    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll), std::cos(roll);

    Eigen::Matrix3d Ry;
    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
          -std::sin(pitch), 0, std::cos(pitch);

    Eigen::Matrix3d R = Ry * Rx; 

    Eigen::Vector3d L;
    for (int i = 0; i < 3; i++) {
        Eigen::Vector3d L_vec = P + (R * B[i]) - A[i];
        L(i) = L_vec.norm();
    }
    return L;
}

// --- 3. Analytical Jacobian Matrix ---
inline Eigen::Matrix3d calculate_Jacobian(double z, double roll, double pitch) {
    Eigen::Vector3d A[3], B[3];
    get_geometry(A, B);

    Eigen::Vector3d P(0.0, 0.0, z);        


    Eigen::Matrix3d Rx, Ry;
    Rx << 1, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, std::sin(roll), std::cos(roll);
    Ry << std::cos(pitch), 0, std::sin(pitch), 0, 1, 0, -std::sin(pitch), 0, std::cos(pitch);
    Eigen::Matrix3d R = Ry * Rx;

    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();

    for (int i = 0; i < 3; i++) {
        Eigen::Vector3d L_vec = P + (R * B[i]) - A[i];
        Eigen::Vector3d u_i = L_vec.normalized(); // Same as L_vec / norm(L_vec)
        Eigen::Vector3d r_i = R * B[i];

        Eigen::Vector3d cross_prod = r_i.cross(u_i);

        J(i, 0) = u_i(2);             // Col 3 = Z Translation -> u_i(2)
        J(i, 1) = cross_prod(0);     // Col 4 = Roll (X rot)  -> cross_prod(0)
        J(i, 2) = cross_prod(1);    // Col 5 = Pitch (Y rot) -> cross_prod(1)

    }
    return J;
}

// --- 4. Forward Kinematics ---
inline Eigen::Vector3d calculate_FK(const Eigen::Vector3d& true_lengths, Eigen::Vector3d guess_q) {
    const int MAX_ITERATIONS = 20;
    const double TOLERANCE = 1e-5;

    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        // Step A: Calculate lengths based on current guess
        Eigen::Vector3d guess_lengths = calculate_IK(guess_q(0), guess_q(1), guess_q(2));

        // Step B: Calculate error
        Eigen::Vector3d error_L = true_lengths - guess_lengths;

        // Stop if converged
        if (error_L.norm() < TOLERANCE) {
            break; 
        }

        // Step C: Calculate the Jacobian
        Eigen::Matrix3d J = calculate_Jacobian(guess_q(0), guess_q(1), guess_q(2));

        // Step D: Newton-Raphson Update
        // colPivHouseholderQr is Eigen's fastest and most stable linear solver for 3x3
        Eigen::Vector3d delta_q = J.colPivHouseholderQr().solve(error_L); 
        
        guess_q += delta_q;
    }

    return guess_q; // Returns [Z, Roll, Pitch]
}

} 
} 

#endif 