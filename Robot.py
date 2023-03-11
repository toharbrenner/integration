import numpy as np
from scipy.spatial.transform import Rotation
import socket
from UR5 import UR5_COM
import time
import math


class Robot:
    def __init__(self):
        self.connect_ur5()

    def connect_ur5(self, isprint=True):
        if isprint:
            print('Connecting UR5...')
        HOST = "192.168.1.113"
        PORT_30003 = 30003
        try:
            self.ur5 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ur5.settimeout(5)
            self.ur5.connect((HOST, PORT_30003))
            self.ur5_com = UR5_COM(self.ur5)
            if isprint:
                print('UR5 Connection Accomplished')
            self.joints_pose = self.ur5_com.get_joints_pose()
            self.tcp_pose = self.ur5_com.get_tcp_position()
            return self.tcp_pose, self.joints_pose
        except:
            if isprint:
                print('UR5 Connection Failed')

    def get_fixed_axis(self, realtime=True):
        # find the axas based the base 
        self.Ef_rot_mat = self.get_cur_tcp_rot_mat(realtime=realtime)
        self.x_vec = np.dot(self.Ef_rot_mat, [1, 0, 0, 0])[0:3]
        self.y_vec = np.dot(self.Ef_rot_mat, [0, 1, 0, 0])[0:3]
        self.z_vec = np.dot(self.Ef_rot_mat, [0, 0, 1, 0])[0:3]
        return self.x_vec, self.y_vec, self.z_vec

    def get_cur_tcp_rot_mat(self, realtime):
        if realtime:
            self.connect_ur5(isprint=False)
        rotvec = self.tcp_pose[3:]
        quat = self.tcp_pose[0:3]
        Ef_rot_mat = np.zeros([4, 4])
        Ef_rot_mat[0:3, 0:3] = Rotation.from_rotvec(rotvec).as_matrix()
        Ef_rot_mat[0:3, 3] = quat
        Ef_rot_mat[3, 3] = 1
        return Ef_rot_mat

    # def move_axis(self,axis,step_size):
    #     self.connect_ur5(isprint=False)
    #     if axis == 'x':
    #         vec = self.x_vec
    #     if axis == 'y':
    #         vec = self.y_vec
    #     if axis == 'z':
    #         vec = self.z_vec
    #     temp_move = self.tcp_pose
    #     temp_move[0:3] = self.tcp_pose[0:3]+vec*step_size
    #     try:
    #         print('Moving ',step_size,' ',axis,' axis' )
    #         self.move(temp_move,type = 'pose')
    #         time.sleep(3)
    #     except:
    #         print('step_not_done')

    def move_axis(self, step_size):  # 0 - x, 1 - y, 2 - z
        end_effector = 0  ################ m (end_effector = 0.41)
        self.connect_ur5(isprint=False)
        axas = [self.x_vec, self.y_vec, self.z_vec]
        if step_size[2] != 0:
            step_size[2] = step_size[2] - end_effector
        temp_move = self.tcp_pose
        temp_move[0:3] = self.tcp_pose[0:3] + axas[0] * step_size[0] + axas[1] * step_size[1] + axas[2] * step_size[2]
        try:
            print('Moving ', step_size[0], ' in x axis')
            print('Moving ', step_size[1], ' in y axis')
            print('Moving ', step_size[2], ' in z axis')
            self.move(temp_move, type='pose')
            # time.sleep(3)
        except:
            print('step_not_done')

    def move(self, where, type='joints'):
        xR, yR, zR, rxR, ryR, rzR = where
        attampts = 0
        dis = 80
        while dis > 0.05:
            if attampts < 5:
                attampts += 1
                if type == 'joints':
                    move = ("movej([" + ("%f,%f,%f,%f,%f,%f" % (xR, yR, zR, rxR, ryR, rzR)) + "], a=0.1, v=0.1, r=0)" + "\n").encode("utf8")
                    self.ur5.send(move)
                    time.sleep(3)
                    self.connect_ur5(isprint=False)
                    real = self.joints_pose
                if type == 'pose':
                    move = ("movep(p[" + ("%f,%f,%f,%f,%f,%f" % (xR, yR, zR, rxR, ryR, rzR)) + "], a=0.1, v=0.1, r=0)" + "\n").encode("utf8")
                    self.ur5.send(move)
                    time.sleep(3)
                    self.connect_ur5(isprint=False)
                    real = self.tcp_pose

                dis = math.dist(real, where)
