#pragma once

#include <vector>
#include <Eigen/Dense>

// Struct to pass the final targets cleanly to the ROS 2 publisher
struct PelvisTarget {
    double x;
    double y;
};

class ZmpGenerator {
private:
    // 1. System Parameters
    const double ZC = 0.6;
    const double G = 9.81;
    const double TS = 0.01;

    // 2. Offsets
    // These adjust the pelvis position to compensate for heavy upper-body parts
    const double COM_OFFSET_X = -0.02;  
    const double COM_OFFSET_Y = 0.005;  

    // 3. State Space Matrices
    Eigen::Matrix3d A;
    Eigen::Vector3d B;
    Eigen::RowVector3d C;

    // 4. LQR Gains (Hardcoded from MATLAB output)
    double Gi;
    Eigen::RowVector3d Gx;
    std::vector<double> f; 

    // 5. System State [Position; Velocity; Acceleration]
    Eigen::Vector3d X_state;
    Eigen::Vector3d Y_state;

    // Tracking Error Accumulators
    double error_sum_X;
    double error_sum_Y;

public:
    ZmpGenerator() {
        // Initialize state to zero (Standing still)
        X_state << 0.0, 0.0, 0.0;
        Y_state << 0.0, 0.0, 0.0;
        error_sum_X = 0.0;
        error_sum_Y = 0.0;

        // Build the discrete physics matrices (Kajita Cart-Table Model)
        A << 1.0,  TS,  pow(TS, 2)/2.0,
             0.0, 1.0,  TS,
             0.0, 0.0, 1.0;

        B << pow(TS, 3)/6.0,
             pow(TS, 2)/2.0,
             TS;

        C << 1.0, 0.0, -ZC/G;

        Gi = 1.36E+03; 

        Gx << 7.17E+04,1.92E+04,1.93E+02;
        
        // The f array will contain your 150 preview weights (1.5s / 0.01s)
        f = {
            1.36E+03,2.60E+03,2.60E+03,2.48E+03,2.38E+03,2.29E+03,2.21E+03,
            2.12E+03,2.04E+03,1.96E+03,1.89E+03,1.82E+03,1.75E+03,1.68E+03,
            1.62E+03,1.56E+03,1.50E+03,1.44E+03,1.39E+03,1.33E+03,1.28E+03,
            1.23E+03,1.19E+03,1.14E+03,1.10E+03,1.06E+03,1.02E+03,9.77E+02,
            9.40E+02,9.04E+02,8.70E+02,8.36E+02,8.05E+02,7.74E+02,7.45E+02,
            7.16E+02,6.89E+02,6.63E+02,6.38E+02,6.13E+02,5.90E+02,5.68E+02,
            5.46E+02,5.25E+02,5.05E+02,4.86E+02,4.67E+02,4.50E+02,4.33E+02,
            4.16E+02,4.00E+02,3.85E+02,3.70E+02,3.56E+02,3.43E+02,3.30E+02,
            3.17E+02,3.05E+02,2.93E+02,2.82E+02,2.72E+02,2.61E+02,2.51E+02,
            2.42E+02,2.33E+02,2.24E+02,2.15E+02,2.07E+02,1.99E+02,1.92E+02,
            1.84E+02,1.77E+02,1.70E+02,1.64E+02,1.58E+02,1.52E+02,1.46E+02,
            1.40E+02,1.35E+02,1.30E+02,1.25E+02,1.20E+02,1.16E+02,1.11E+02,
            1.07E+02,1.03E+02,99.03450613,95.26601584,91.64092525,88.15377767,
            84.79932406,81.57251512,78.46849366,75.48258734,72.61030161,
            69.84731294,67.18946233,64.63274904,62.17332456,59.80748683,
            57.53167465,55.34246234,53.2365546,51.21078147,49.26209366,
            47.3875579,45.58435253,43.84976325,42.18117909,40.57608838,
            39.03207505,37.54681498,36.11807246,34.74369688,33.42161944,
            32.14985008,30.92647446,29.7496511,28.61760857,27.52864285,
            26.48111478,25.47344756,24.50412439,23.57168619,22.6747294,
            21.81190388,20.98191085,20.18350096,19.41547241,18.6766691,
            17.96597895,17.28233219,16.62469977,15.99209176,15.38355595,
            14.79817632,14.23507172,13.69339456,13.17232945,12.67109206,
            12.18892791,11.72511121,11.2789438,10.84975408,10.43689602,
            10.03974816,9.657712686,9.290214538,8.936700538,8.596638557
        }; 
    }

    // 6. The 100Hz Update Loop
    // Takes the current foot target, and the next 150 footprint targets (the preview)
    PelvisTarget update(
        double current_ref_X, 
        double current_ref_Y, 
        const std::vector<double>& preview_window_X, 
        const std::vector<double>& preview_window_Y) 
    {
        // --- X-Axis Physics ---
        double zmp_X = (C * X_state).value(); // Extract scalar from 1x1 matrix
        error_sum_X += (zmp_X - current_ref_X);

        // Calculate preview feedforward term
        double prev_X = 0.0;
        for (size_t i = 0; i < preview_window_X.size(); i++) {
            prev_X += f[i] * preview_window_X[i];
        }

        // Calculate control input (Jerk)
        double u_X = -Gi * error_sum_X - (Gx * X_state).value() + prev_X;
        
        // Integrate to next state
        X_state = A * X_state + B * u_X;

        // --- Y-Axis Physics ---
        double zmp_Y = (C * Y_state).value();
        error_sum_Y += (zmp_Y - current_ref_Y);

        double prev_Y = 0.0;
        for (size_t i = 0; i < preview_window_Y.size(); i++) {
            prev_Y += f[i] * preview_window_Y[i];
        }

        double u_Y = -Gi * error_sum_Y - (Gx * Y_state).value() + prev_Y;
        Y_state = A * Y_state + B * u_Y;

        // --- Apply SolidWorks Offsets ---
        // LQR outputs the ideal MASS location. We subtract offsets to command the PELVIS.
        PelvisTarget target;
        target.x = X_state(0) - COM_OFFSET_X;
        target.y = Y_state(0) - COM_OFFSET_Y;

        return target;
    }
};