import time
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

from crisp_py.robot import Robot, make_robot
from crisp_py.utils.geometry import Pose

DELTA_MOVE_X = 0.002  # [m]
DELTA_MOVE_Y = 0.002  # [m]
DELTA_MOVE_Z = -0.002  # [m]
DELTA_ANGULAR = 0.01  # [rad]
FREQUENCY = 50  # [Hz]

# Get the crisp_py package directory
CRISP_PY_DIR = Path(__file__).parent.parent
CONFIG_FILE_FOR_CI = CRISP_PY_DIR / "config" / "control" / "tuned_cartesian_impedance.yaml"
CONFIG_FILE_FOR_OSC = CRISP_PY_DIR / "config" / "control" / "tuned_operational_space_controller.yaml"

# Initialize robot
print("Initializing robot...")
robot: Robot = make_robot("fr3")
robot.wait_until_ready()
robot.home()
time.sleep(0.5)

# Comment/Uncomment following lines to use tuned CI or tuned OSC
# robot.cartesian_controller_parameters_client.load_param_config(
#     file_path=str(CONFIG_FILE_FOR_CI)
# )
robot.cartesian_controller_parameters_client.load_param_config(
    file_path=str(CONFIG_FILE_FOR_OSC)
)

robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# Create target pose from specified values
target_position = np.array([0.5, -0.25, 0.15])
target_rotation = Rotation.from_euler('xyz', [3.1415926, 0.0, 0.0])
target_pose = Pose(target_position, target_rotation)

print("Moving to target...")
robot.move_to(pose=target_pose, speed=0.1)
time.sleep(4.0)  # Wait for robot to reach first target

rate = robot.node.create_rate(FREQUENCY)
# Loop through each cartesian target (skip first one since we already moved there)
for i in range(10):
    current_tcp_pose = robot.end_effector_pose.copy()

    print(f"Position: [{current_tcp_pose.position[0]:.5f}, {current_tcp_pose.position[1]:.5f}, {current_tcp_pose.position[2]:.5f}]")
    current_tcp_pose.position[0] += DELTA_MOVE_X
    current_tcp_pose.position[1] += DELTA_MOVE_Y
    current_tcp_pose.position[2] += DELTA_MOVE_Z

    current_rpy = current_tcp_pose.orientation.as_euler('xyz')
    current_tcp_pose.orientation = Rotation.from_euler('xyz', current_rpy + DELTA_ANGULAR)
    updated_rpy = current_tcp_pose.orientation.as_euler('xyz')
    print(f"Position: [{current_tcp_pose.position[0]:.5f}, {current_tcp_pose.position[1]:.5f}, {current_tcp_pose.position[2]:.5f}]")
    print(f"Orientation (RPY): [{updated_rpy[0]:.5f}, {updated_rpy[1]:.5f}, {updated_rpy[2]:.5f}]")
    robot.set_target(pose=current_tcp_pose)
    rate.sleep()

# Go back to home
print("\n--- Going back to home position ---")
robot.home()

# Shutdown
robot.shutdown()
print("Done!")
