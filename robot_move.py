import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
import modern_robotics as mr
import realsense

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'

def convert_deg_to_rad(deg):
    rad = deg*(np.pi/180)
    return rad

"""Convert coordinate frame from camera to base of robot to end effector"""
def convert_coord_frame(x_c,y_c,z_c):
    cam = [x_c, y_c, z_c, 1]
    rot_mat_z = [[np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)), 0], [np.sin(np.deg2rad(90)), np.cos(np.deg2rad(90)), 0], [0,0,1]]
    rot_mat_x = [[1,0,0],[0, np.cos(0.2), np.sin(0.2)], [0, -np.sin(0.2), np.cos(0.2)]]
    rot_mat = np.matmul(rot_mat_x, rot_mat_z)
    disp_vec = [[0.26415], [0.26415], [0]]
    extra_row = [[0,0,0,1]]
    homogen_rc = np.concatenate((rot_mat, disp_vec), axis=1)            #put side by side
    homogen_rc = np.concatenate((homogen_rc, extra_row), axis=0)
    rob_base = np.matmul(homogen_rc, cam)
    print(rob_base)
    joints = robot.arm.get_joint_commands()
    T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
    [R,p] = mr.TransToRp(T)
    p = [[p[0]], [p[1]], [p[2]]]
    homogen_er = np.concatenate((R,p),axis=1)
    homogen_er = np.concatenate((homogen_er, extra_row), axis=0)
    ee = np.matmul(homogen_er, rob_base)
    return ee

while mode != 'q':
    mode = input("[h]ome, [s]leep, [q]uit, [c]lose, [o]pen, [f]orward, [b]ackward, [set_single], [pen], [info]")
    if mode == "h":
        robot.arm.go_to_home_pose()
        elbow = robot.arm.get_single_joint_command('elbow')
        print(elbow)
        prev_elbow = robot.arm.get_single_joint_command('elbow')
        robot.arm.set_single_joint_position('elbow', prev_elbow + 0.2)
        elbow = robot.arm.get_single_joint_command('elbow')
        # prev_waist = robot.arm.get_single_joint_command('waist')
        # robot.arm.set_single_joint_position('waist', prev_waist + 1.57)
        # print(elbow)
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
    elif mode == "pen":
        """go to home"""
        robot.arm.go_to_home_pose()
        elbow = robot.arm.get_single_joint_command('elbow')
        print(elbow)
        prev_elbow = robot.arm.get_single_joint_command('elbow')
        robot.arm.set_single_joint_position('elbow', prev_elbow + 0.2)
        elbow = robot.arm.get_single_joint_command('elbow')

        """realsense"""
        realsense.sense_pen()
        print(realsense.coord)
        x_cam = realsense.coord[0]
        y_cam = realsense.coord[1]
        z_cam = realsense.coord[2]
        end_eff = convert_coord_frame(x_cam, y_cam, z_cam)  
        waist = -np.cos(end_eff[0]/end_eff[1])
        print(f"waist: {waist}")
        robot.arm.set_single_joint_position('waist', waist)
        print(end_eff)
        
    elif mode == "info":
        print(robot.arm.group_info)

    