import cv2

import Camera
import Segmentation
import Measuring
from Robot import Robot
from UR5 import UR5_COM

import os.path
import csv
import serial
import time
import pandas as pd

if __name__ == '__main__':
    sample_succeed = False
    result_time_file = '/home/arl1/Tohar/time-data.csv'

    robot = Robot()
    ur5 = UR5_COM()
    Segmentation.prepare_data()
    # # cfg = Segmentation.training()
    # joint_angles = ur5.get_joints_pose()
    # print(joint_angles)
    home_pose = [-2.58462531, -1.06360513, -2.35941178, 0.28092825,  1.86470652,  0.07405952] # home pose
    ur5.move(home_pose, move_type='joints') # move to the home pose

    while not sample_succeed:
        tic_all_sample = time.time()
        aligned_depth_frame, color_image, depth_image = Camera.connect_realsense()
        tic_segmentation = time.time()
        Segmentation.model(color_image)
        toc_segmentation = time.time()
        props_branches = Measuring.measure()
        z, x_move, y_move, z_move, props_branches_sorted = Camera.extract_pixels(aligned_depth_frame, props_branches, depth_image)
        # width_branch, length_branch = Measuring.convert_real_world(z, props_branches_sorted)
        joint_angles = ur5.get_joints_pose()
        robot.get_fixed_axis()
        if len(props_branches_sorted) > 0:
            number_of_branches = len(props_branches_sorted)
            tic_movement = time.time()
            robot.move_axis(step_size=[x_move, y_move, z_move])  # m
            print("Arrived to the point")
            toc_movement = time.time()
            sample_succeed = True
            ser = serial.Serial("/dev/ttyUSB0", 9600)  # open serial port ("dmesg | grep tty" or "python -m serial.tools.list_ports")
            ser.write(b'a')  # write a string
            time.sleep(5)
            b = ser.readline()
            time.sleep(5)
            if isinstance(b, bytes):  # sample succeed
                print(b)
                sample_succeed = True
                toc_all_sample = time.time()
                total_sample_time = toc_all_sample - tic_all_sample
                total_segmentation_time = toc_segmentation - tic_segmentation
                total_movement_time = toc_movement - tic_movement
                df = pd.DataFrame(columns=["total_sample_time", "total_segmentation_time", "total_movement_time",
                                           "number_of_branches"])
                df = df.append(
                    {'total_sample_time': total_sample_time, 'total_segmentation_time': total_segmentation_time,
                     'total_movement_time': total_movement_time, "number_of_branches": number_of_branches},
                    ignore_index=True)
                if os.path.isfile(result_time_file):
                    df.to_csv(result_time_file, mode='a', index=True, header=False)
                else:
                    df.to_csv(result_time_file, index=True, header=True)
            else:  # sample not succeed
                print('Sample not succeed')
            ur5.move(home_pose, move_type='joints')  # move to the home pose
        else:
            continue






