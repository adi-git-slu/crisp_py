
import time
import numpy as np
from pathlib import Path
from crisp_py.robot import Robot, make_robot

# Get the crisp_py package directory
CRISP_PY_DIR = Path(__file__).parent.parent
CONFIG_FILE_FOR_JIC = CRISP_PY_DIR / "config" / "control" / "tuned_joint_impedance_control.yaml"

# Initialize robot
print("Initializing robot...")
robot: Robot = make_robot("fr3")
robot.wait_until_ready()
robot.home()
time.sleep(1.0)


def generate_sinewave_joint_targets(amplitude, frequency, duration, ctrl_freq):
    """Generate sinewave joint targets for all joints.

    Args:
        amplitude: Amplitude of the sinewave (radians)
        frequency: Frequency of the sinewave (Hz)
        duration: Duration to generate targets for (seconds)
        ctrl_freq: Frequency at which to generate targets (Hz)
    
    Returns:
        List of joint target arrays, each with 7 joint positions
    """
    num_samples = int(duration * ctrl_freq)
    dt = 1.0 / ctrl_freq
    
    # Get the current joint positions as the center point
    q_center: np.ndarray = robot.joint_values
    
    targets = []
    for i in range(num_samples):
        t = i * dt
        # Generate sinewave offset for each joint
        q_target = amplitude * np.sin(2 * np.pi * frequency * t) + q_center
        
        # Apply the same sinewave to all 7 joints
        targets.append(q_target)
    
    return targets


# Example usage
amplitude = 0.25  # radians
frequency = 0.2  # rad/s
duration = 10.0  # seconds
ctrl_freq = 50.0  # Hz

print("Generating sinewave targets...")
targets = generate_sinewave_joint_targets(amplitude, frequency, duration, ctrl_freq)

print(f"Executing {len(targets)} sinewave joint targets...")
robot.joint_controller_parameters_client.load_param_config(
    file_path=str(CONFIG_FILE_FOR_JIC)
)
robot.controller_switcher_client.switch_controller("joint_impedance_controller")

rate = robot.node.create_rate(ctrl_freq)

for q_target in targets:
    robot.set_target_joint(q_target)
    rate.sleep()

robot.shutdown()
print("Done!")