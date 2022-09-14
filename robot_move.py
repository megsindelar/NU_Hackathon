from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'

while mode != 'q':
    mode = input("[h]ome, [s]leep, [q]uit, [c]lose, [o]pen, [f]orward, [b]ackward, [info]")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    elif mode == "c":
        robot.gripper.grasp()
    elif mode == "o":
        robot.gripper.release()
    elif mode == "f":
        prev_shoulder = robot.arm.get_single_joint_command('shoulder')
        robot.arm.set_single_joint_position('shoulder', prev_shoulder + 0.1)
        prev_elbow = robot.arm.get_single_joint_command('elbow')
        robot.arm.set_single_joint_position('elbow', prev_elbow - 0.1)
    elif mode == "b":
        prev_shoulder = robot.arm.get_single_joint_command('shoulder')
        robot.arm.set_single_joint_position('shoulder', prev_shoulder - 0.1)
        prev_elbow = robot.arm.get_single_joint_command('elbow')
        robot.arm.set_single_joint_position('elbow', prev_elbow + 0.1)
    elif mode == "info":
        print(robot.arm.group_info)

