from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'

while mode != 'q':
    mode = input("[h]ome, [s]leep, [q]uit, [c]lose, [o]pen")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    # elif mode == "ee_pos":
    #     x = int(input("x:  "))
    #     y = int(input("y:  "))
    #     z = int(input("z:  "))
    #     robot.arm.set_ee_pose_components(x,y,z)
    elif mode == "c":
        robot.gripper.grasp()
    elif mode == "o":
        robot.gripper.release()
    
