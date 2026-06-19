uint8 joint_id              # 0=Ankle, 1=Hip_L1, 2=Hip_L2, 3=Hip_L3
float32 target_length       # Calculated Law of Cosines length (meters)
float32 target_velocity     # Feed-forward speed for the ballscrew (m/s)
float32 impedance_kp        # Virtual stiffness adjustment