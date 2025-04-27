import numpy as np

# Constants
S = 0.014  # Step length (m)
H = 0.006  # Step height (m)
f = 1.25  # Step frequency (steps/second)
T = 1 / f  # Step period (seconds)
L1 = 0.10  # Length from hip joint to knee joint (m)
L2 = 0.1029  # Length from knee joint to ankle joint (m)
g = 9.81  # Gravitational acceleration (m/s^2)

def generate_trajectory(S, H, T, time_interval, phase):
    """
    Generate single-leg gait trajectory.
    When phase = "left", the left leg moves; when phase = "right", the right leg moves.
    """
    t = np.arange(0, T, time_interval)
    x_t = S * (t / T)  # Forward movement trajectory
    z_t = H * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)  # Vertical movement trajectory
    y_t = np.zeros_like(t)  # Lateral movement trajectory
    return t, x_t, y_t, z_t

def inverse_kinematics(x, y, z):
    """
    Calculate inverse kinematics angles.
    """
    D = np.sqrt(x**2 + y**2 + z**2)
    theta_3 = np.pi - np.arccos(np.clip((L1**2 + L2**2 - D**2) / (2 * L1 * L2), -1.0, 1.0))
    theta_1 = np.arctan2(y, x)
    theta_2 = np.arctan2(z, np.sqrt(x**2 + y**2)) - np.arctan2(L2 * np.sin(theta_3), L1 + L2 * np.cos(theta_3))
    theta_4 = -(theta_2 + theta_3)
    theta_5 = -theta_1
    return theta_1, theta_2, theta_3, theta_4, theta_5

def generate_motion_data():
    """
    Generate motion data for alternating gaits.
    """
    motion_lines = ["#WEBOTS_MOTION,V1.0,LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll,RHipYawPitch,RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll\n"]
    
    time_interval = 0.040  # 40 ms
    
    # Generate trajectory for the left leg
    t, x_t, y_t, z_t = generate_trajectory(S, H, T, time_interval, phase="left")
    for i in range(len(t)):
        theta_1, theta_2, theta_3, theta_4, theta_5 = inverse_kinematics(x_t[i], y_t[i], z_t[i])
        time_str = f"00:00:{int(t[i] * 1000):03d}"
        motion_lines.append(f"{time_str},Pose{i+1},0.000,{theta_1:.3f},{theta_2:.3f},{theta_3:.3f},{theta_4:.3f},{theta_5:.3f},0.000,0,0,-0.525,1.05,-0.525,0\n")
    
    # Generate trajectory for the right leg
    t, x_t, y_t, z_t = generate_trajectory(S, H, T, time_interval, phase="right")
    for i in range(len(t)):
        theta_1, theta_2, theta_3, theta_4, theta_5 = inverse_kinematics(x_t[i], y_t[i], z_t[i])
        time_str = f"00:00:{int((T + t[i]) * 1000):03d}"
        motion_lines.append(f"{time_str},Pose{i+1+len(t)},0.000,0,0,-0.525,1.05,-0.525,0,0.000,{theta_1:.3f},{theta_2:.3f},{theta_3:.3f},{theta_4:.3f},{theta_5:.3f}\n")
    
    return motion_lines

def save_motion_file(motion_lines, filename="test.motion"):
    """
    Save motion file.
    """
    with open(filename, "w") as f:
        f.writelines(motion_lines)
    print(f"Motion data saved to {filename}")

# Execution flow
motion_lines = generate_motion_data()
save_motion_file(motion_lines)
