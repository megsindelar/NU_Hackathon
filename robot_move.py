import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'

def convert_deg_to_rad(deg):
    rad = deg*(np.pi/180)
    return rad

while mode != 'q':
    mode = input("[h]ome, [s]leep, [q]uit, [c]lose, [o]pen, [f]orward, [b]ackward, [set_single], [info]")
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
    elif mode == "set_single":
        joint = input("Joint:  ")
        if joint in robot.arm.group_info.joint_names:
            deg_move = int(input("Degrees (relative):  "))
            rad = convert_deg_to_rad(deg_move)
            prev = robot.arm.get_single_joint_command(joint)
            ind = robot.arm.group_info.joint_names.index(joint)
            min = robot.arm.group_info.joint_lower_limits[ind]
            max = robot.arm.group_info.joint_upper_limits[ind]
            if min <= rad <= max:
                robot.arm.set_single_joint_position(joint, prev + rad)
            else:
                print("Outside of limits. Min:  " + min + "  Max:  " + max)
        else:
            print("Not a valid joint")
    elif mode == "info":
        print(robot.arm.group_info)

