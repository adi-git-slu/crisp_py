"""Example moving the robot to target joint configurations using the joint trajectory controller."""

from crisp_py.robot import make_robot

robot = make_robot("fr3")
robot.wait_until_ready()

# %%
# Home the robot first
robot.home()

# %%
# Get current joint configuration and move each joint by +0.5 rad
q = robot.joint_values
robot.move_to_joint(q + 0.5, time_to_goal=5.0)

# %%
# Move each joint by -0.5 rad from current position
q = robot.joint_values
robot.move_to_joint(q - 0.5, time_to_goal=3.0)

# %%
robot.shutdown()
