#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import socket
import numpy as np

import cv2

# import PIL.Image
from PIL import Image as PILImage
import pyrealsense2 as rs

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
from camera import CamCapture
import mouse


class UR_Env:
    def __init__(self):
        os.system("python ur_script_sender_xy.py")
        self.config_ur_socket()
        self.config_thread()
        self.cam = CamCapture()
        self.cam.start()
        self.goal_pose = np.array([-0.3, -0.27])

    ### -------------------------------------------------------------
    def config_ur_socket(self):
        self.HOST = "192.168.0.120"  # 192.168.0.20 Remote host (Ubuntu PC).
        self.PORT = 30002  # UR port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.HOST, self.PORT))
        self.sock.listen(5)
        self.con, self.addr = self.sock.accept()
        self.lost_connection_stop = False

    ### -------------------------------------------------------------
    def config_thread(self):
        # Thread
        self.stop_state_thread = True
        self.stop_action_thread = True

        self.msg_state = []
        self.thread_state_receive = threading.Thread(target=self.receive_state)
        self.thread_state_receive.setDaemon(True)
        self.thread_state_receive.start()

        self.msg_action = "(0, 0, 2)"
        self.thread_action_sender = threading.Thread(target=self.send_action)
        self.thread_action_sender.setDaemon(True)
        self.thread_action_sender.start()

    def receive_state(self):
        while True:
            while self.stop_state_thread:
                try:
                    while True:
                        # self.msg_state = self.con.recv(8192)
                        msg = self.con.recv(8192)
                        msg_str = self.toStr(msg)
                        p_num = msg_str.count("p")
                        plus_num = msg_str.count("+")
                        # print("p_num",p_num)
                        if plus_num==1 and p_num == 3:
                            self.msg_state = msg
                            # print("msg: ", self.msg_state)
                            break
                        elif plus_num!=1 and p_num>3:
                            self.msg_state = msg
                        else:
                            print("unusual msg",msg)
                        # print("msg: ", self.msg_state)
                except socket.error as e:
                    self.lost_connection_stop = True
                    print("connection lost")
                    print("Repair UR5 from its pendant")
                    print("Reconnect -> Enter,  Finish -> q")
                    while not rospy.is_shutdown():
                        if input() == "q":
                            print("exit")
                            exit(1)
                        else:
                            print("You really activate UR5 ???")
                            if input() == "q":
                                print("exit")
                                exit(1)
                            else:
                                break
                    os.system("python ur_script_sender_xy.py")
                    self.config_ur_socket()
                    self.lost_connection_stop = False

    def send_action(self):
        while True:
            while self.stop_action_thread:
                try:
                    self.con.send(self.toBytes(self.msg_action))
                    time.sleep(0.002)
                except socket.error as e:
                    print("connection lost")
                    while self.lost_connection_stop:
                        time.sleep(0.5)

    ### -------------------------------------------------------------
    def reset(self):
        print("\n\n\n--------------------------------------")
        print("Reset")
        self.set_home_pose()
        # self.pause()
        # return self.get_state()

    def pause(self):
        print("Start by pushing Enter or Exit by pushing q")
        while not rospy.is_shutdown():
            if input() == "q":
                print("exit")
                exit(1)
            else:
                break

    def hard_stop(self):
        self.step([0.0] * 3, stop=1)
        self.con.send(self.toBytes(self.msg_action))
        time.sleep(1)

    def set_home_pose(self):
        print("Set home position")
        # self.soft_stop()
        self.step([0.0] * 3, stop=2)
        self.stop_action_thread = False
        self.con.send(self.toBytes(self.msg_action))
        time.sleep(3)
        self.stop_action_thread = True

    def ee_move(self, action, stop=0):
        # print("move ee",action)
        # self.soft_stop()
        ##world cordinate to ur cordinate
        action[:2] = -action[:2]
        action = self.limit_action(action)
        # print("limited action",action)
        self.step(action, stop)
        self.stop_action_thread = False
        self.con.send(self.toBytes(self.msg_action))
        time.sleep(3)
        # start = time.time()
        # while True:
        while True:
            msg = self.toStr(self.msg_state)
            plus_num = msg.count("+")
            divided_msg = msg.split("+")
            if plus_num == 1:
                msg = divided_msg[0].replace("p", "")
            else:
                msg = divided_msg[-2].replace("p", "")
            # msg = divided_msg[0].replace("p", "")
            state_list = msg.split("_")
            state_dict = {
                "ee_pose": eval(state_list[0]),
                "ee_speed": eval(state_list[1]),
                "joints": eval(state_list[2]),
                "force": eval(state_list[3]),
            }
            ee_pos = np.array(state_dict["ee_pose"][:3])
            # print("ee_pos",ee_pos)
            delta = abs(ee_pos-action)
            if all(i <0.0001 for i in delta):
                break
            else:
                print("not arrived!")
                self.step(action, stop)
                self.con.send(self.toBytes(self.msg_action))
                time.sleep(3)
                
        if self.cam.Fall_down:
            print("object fall down, press enter to continue or q to quit:")
            self.pause()
        # end = time.time()
        # print("cost time",end-start)
        self.stop_action_thread = True

    def get_state(
        self,
    ):
        msg = self.toStr(self.msg_state)
        plus_num = msg.count("+")
        divided_msg = msg.split("+")
        if plus_num == 1:
            msg = divided_msg[0].replace("p", "")
        else:
            msg = divided_msg[-2].replace("p", "")
        # msg = divided_msg[0].replace("p", "")        
        state_list = msg.split("_")
        # print("split msg: ", state_list)

        state_dict = {
            "ee_pose": eval(state_list[0]),
            "ee_speed": eval(state_list[1]),
            "joints": eval(state_list[2]),
            "force": eval(state_list[3]),
        }
        # print("state dict: ", state_dict)
        object_pos = self.cam.location
        ee_pos = np.array(state_dict["ee_pose"][:2])
        # print("pre_ee_pos",ee_pos)
        #ur cordinate to world cordinate
        ee_pos = -ee_pos
        pos = np.append(ee_pos, object_pos)
        # print("ee_pos",ee_pos)
        rel_pos_x = object_pos[0] - ee_pos[0]
        rel_pos_y = object_pos[1] - (ee_pos[1]+0.5)
        # print("ee_pos", ee_pos[0], ee_pos[1]+0.5)
        # print("object_pos",object_pos)
        goal_rel_x = self.goal_pose[0] - object_pos[0]
        goal_rel_y = self.goal_pose[1] - object_pos[1]
        state = np.array(
            [
                rel_pos_x,
                rel_pos_y,
                goal_rel_x,
                goal_rel_y,
            ]
        )
        # print(ee_pos)
        return pos, state_dict["force"], state

    def step(self, action, stop=0):
        while self.lost_connection_stop:
            time.sleep(0.5)

        cmd = f"({action[0]} ,{action[1]}, {action[2]}, {stop})"
        self.msg_action = cmd
        # print(self.msg_action)

    def change_target_pose(self, action, stop):
        while self.lost_connection_stop:
            time.sleep(0.5)

        cmd = (
            "("
            + str(action[0])
            + ","
            + str(action[1])
            + ","
            + str(action[2])
            + ","
            + str(action[3])
            + ","
            + str(stop)
            + ")"
        )
        self.msg_action = cmd
        self.stop_action_thread = False
        self.con.send(self.toBytes(self.msg_action))

    def limit_action(self, pose):
        # --- x (left, right) ---
        ## for F shape
        if pose[0] > 0.35:
            pose[0] = 0.35
        # if pose[0] > 0.5:
        #     pose[0] = 0.5

        if pose[0] < -0.40:
            pose[0] = -0.40
        # --- y (front, back) ---
        if pose[1] > 0.80:
            pose[1] = 0.80

        if pose[1] < 0.25:
            pose[1] = 0.25
        
        if pose[1] < 0.35 and pose[0]<-0.35:
            pose[0] = -0.35

        # ---task space--for F shape:
        # if pose[0] > 0.2 and pose[1] < 0.5:
        #     pose[1] = 0.5
        return pose

    ### -------------------------------------------------------------
    def toBytes(self, str):
        return bytes(str.encode())

    def toStr(self, byte_data):
        return str(byte_data.decode())


# if __name__ == "__main__":
#     test_ur_env = UR_Env()
#     test_ur_env.reset()
#     time.sleep(2)
#     try:
#         while not rospy.is_shutdown():
#             time.sleep(2)
#             state = test_ur_env.cam.get_pos()
#             print(state)
#     except KeyboardInterrupt:
#         exit()
